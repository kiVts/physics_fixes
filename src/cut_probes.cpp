
#include "call_stub.h"
#include "code_ranges.h"
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

enum Kind { kPlain, kSecondView, kCharMark, kCharSub, kAiChar, kContact };
struct Probe {
    uintptr_t address;
    const char* name;
    const char* what;
    Kind kind;
};
// Order = log order. Indented ones are called inside the one above.
const Probe kProbes[] = {
    {0x140306c50, "FUN_140306c50", "render: world particle quads", kPlain},
    {0x1404d2a70, "FUN_1404d2a70", "render: character start", kCharMark},
    {0x140297cf0, "FUN_140297cf0", "render:   skeleton pose", kCharSub},
    {0x140298110, "FUN_140298110", "render:   bone matrices", kCharSub},
    {0x1406cde70, "FUN_1406cde70", "render:   skinned draws", kCharSub},
    {0x140509180, "FUN_140509180", "render:   extra kind", kCharSub},
    {0x1406cd6b0, "FUN_1406cd6b0", "render: loose objects", kPlain},
    {0x1406ca300, "FUN_1406ca300", "render: second view (monitors)", kSecondView},
    // A: contact callback (physics thread, host)
    {0x1405ed160, "FUN_1405ed160", "A contact callback", kContact},
    // B: character containment
    {0x1405333e0, "FUN_1405333e0", "B character containment", kPlain},
    // C: zone 0x33 per-body second pass
    {0x1404baf80, "FUN_1404baf80", "C zone 0x33 pass", kPlain},
    {0x1404995b0, "FUN_1404995b0", "C   per body", kPlain},
    {0x14049bfd0, "FUN_14049bfd0", "C     doors / water", kPlain},
    {0x140499bf0, "FUN_140499bf0", "C     water", kPlain},
    {0x1404a0c50, "FUN_1404a0c50", "C     fire", kPlain},
    // D: hull task setup (water rays)
    {0x14048a610, "FUN_14048a610", "D hull setup per body", kPlain},
    {0x1402f4950, "FUN_1402f4950", "D   water mesh rays", kPlain},
    // E: logic-side characters
    {0x1405ec6b0, "FUN_1405ec6b0", "E character logic pass", kPlain},
    {0x1405447f0, "FUN_1405447f0", "E   pose + ghost transforms", kPlain},
    // F: AI character update (targeting rays)
    {0x140739370, "FUN_140739370", "F AI character update", kAiChar},
};
constexpr int kN = sizeof kProbes / sizeof kProbes[0];

// buckets: 0 not split, 1-4 walking by ring, 5-8 seated by ring, 9 ring unknown (no player), 10 second view
constexpr int kBuckets = 11, kBucketUnknown = 9, kBucketSecond = 10;
constexpr size_t kCharSpan = 0x4800; // character object size (its sub-objects the per-character calls get)
constexpr size_t kCharSeat = 0x2100, kCharPos = 0x68;

// --- accumulators -------------------------------------------------------------------------------------------
struct Acc { // one per thread; written by its thread only, read (racy, 64-bit) by the logger
    DWORD tid;
    volatile uint64_t cycles[kN][kBuckets], calls[kN][kBuckets];
};
constexpr int kMaxThreads = 128;
Acc* g_acc[kMaxThreads];
std::atomic<int> g_nacc{0};
std::atomic<uint64_t> g_lost{0}, g_mismatch{0};

struct Frame {
    uint8_t id, bucket;
    uint64_t t0;
};
constexpr int kDepth = 64;
struct Tls {
    Acc* acc = nullptr;
    bool no_acc = false;
    int depth = 0;
    Frame stack[kDepth];
    int id_depth[kN] = {};
    int second_view = 0;
    uintptr_t cur_char = 0;
    uint8_t cur_bucket = 0;
};
thread_local Tls tl;

Acc* acc()
{
    if (tl.acc || tl.no_acc) return tl.acc;
    int i = g_nacc.fetch_add(1);
    if (i >= kMaxThreads) {
        tl.no_acc = true;
        return nullptr;
    }
    auto* a = static_cast<Acc*>(VirtualAlloc(nullptr, sizeof(Acc), MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    if (!a) {
        tl.no_acc = true;
        return nullptr;
    }
    a->tid = GetCurrentThreadId();
    g_acc[i] = a;
    tl.acc = a;
    return a;
}

// manifold counts (physics thread only)
std::atomic<uint64_t> g_cb_calls{0}, g_cb_off{0}, g_cb_manifolds{0}, g_cb_owned{0}, g_cb_with_points{0}, g_cb_points{0},
    g_cb_skippable{0}, g_cb_failed{0};

const uint8_t* g_cb_flag = nullptr; // DAT_140ceb6ff (set in install)

template <typename T> T rd(uintptr_t p, size_t off) { return *reinterpret_cast<const T*>(p + off); }

uint8_t char_bucket(uintptr_t ch)
{
    __try {
        if (!ch) return 0;
        bool seated = rd<uintptr_t>(ch, kCharSeat) != 0;
        const double* p = reinterpret_cast<const double*>(ch + kCharPos);
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) return kBucketUnknown;
        double pos[3] = {p[0], p[1], p[2]};
        int ring = player_ring(pos);
        if (ring < 0) return kBucketUnknown;
        return static_cast<uint8_t>(1 + (seated ? 4 : 0) + ring);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
}

void count_manifolds(uintptr_t cb)
{
    g_cb_calls.fetch_add(1, std::memory_order_relaxed);
    __try {
        if (!g_cb_flag || !*g_cb_flag || !rd<uintptr_t>(cb, 0xa0)) {
            g_cb_off.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        void* disp = reinterpret_cast<void*>(rd<uintptr_t>(cb, 0x28));
        const uintptr_t* vt = *reinterpret_cast<const uintptr_t* const*>(disp);
        int n = reinterpret_cast<int(__fastcall*)(void*)>(vt[8])(disp);
        uint64_t owned = 0, with_points = 0, points = 0, skippable = 0;
        for (int i = 0; i < n; i++) {
            uintptr_t m = reinterpret_cast<uintptr_t(__fastcall*)(void*, int)>(vt[9])(disp, i);
            if (!m) continue;
            uint32_t pts = rd<uint32_t>(m, 0x5e0);
            uintptr_t b0 = rd<uintptr_t>(m, 0x5d0), b1 = rd<uintptr_t>(m, 0x5d8);
            bool own = b0 && b1 && rd<uintptr_t>(b0, 0x1f8) && rd<uintptr_t>(b1, 0x1f8);
            owned += own;
            with_points += pts != 0;
            points += pts;
            skippable += own && pts == 0;
        }
        g_cb_manifolds.fetch_add(n > 0 ? n : 0, std::memory_order_relaxed);
        g_cb_owned.fetch_add(owned, std::memory_order_relaxed);
        g_cb_with_points.fetch_add(with_points, std::memory_order_relaxed);
        g_cb_points.fetch_add(points, std::memory_order_relaxed);
        g_cb_skippable.fetch_add(skippable, std::memory_order_relaxed);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        g_cb_failed.fetch_add(1, std::memory_order_relaxed);
    }
}

void maybe_log();

void __fastcall cp_enter(uint32_t id, const uintptr_t* args)
{
    uint8_t bucket = 0;
    switch (kProbes[id].kind) {
    case kSecondView:
        tl.second_view++;
        break;
    case kCharMark:
        tl.cur_char = args[0];
        tl.cur_bucket = char_bucket(args[0]);
        bucket = tl.second_view ? kBucketSecond : tl.cur_bucket;
        break;
    case kCharSub:
        if (tl.second_view) bucket = kBucketSecond;
        else if (tl.cur_char && args[0] - tl.cur_char < kCharSpan) bucket = tl.cur_bucket;
        break;
    case kAiChar:
        bucket = char_bucket(args[1]);
        break;
    case kContact:
        count_manifolds(args[0]);
        break;
    default:
        break;
    }
    tl.id_depth[id]++;
    if (tl.depth < kDepth) tl.stack[tl.depth] = Frame{static_cast<uint8_t>(id), bucket, __rdtsc()};
    tl.depth++;
}

void __fastcall cp_leave(uint32_t id)
{
    uint64_t now = __rdtsc();
    tl.depth--;
    tl.id_depth[id]--;
    if (kProbes[id].kind == kSecondView) tl.second_view--;
    if (tl.depth < 0) { // can't happen (stubs always pair); reset
        tl.depth = 0;
        g_mismatch.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (tl.depth >= kDepth) {
        g_lost.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    const Frame& f = tl.stack[tl.depth];
    if (f.id != id) {
        g_mismatch.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (Acc* a = acc()) {
        a->calls[id][f.bucket]++;
        if (tl.id_depth[id] == 0) a->cycles[id][f.bucket] += now - f.t0;
    }
    maybe_log();
}

// --- stubs ----------------------------------------------------------------------------------------------------
constexpr size_t kStubSlot = 256, kBlockSize = 64 * 1024;
uint8_t* g_block = nullptr;
void* g_tramp[kN];
RUNTIME_FUNCTION* g_pdata = nullptr;
const char* g_names[kN];
std::string g_name_store[kN];
SwmlCodeRange g_range{};
std::atomic<bool> g_published{false};

// --- log --------------------------------------------------------------------------------------------------------
std::atomic<bool> g_logging{false};
ULONGLONG g_last_log = 0;
uint32_t g_last_tick = 0;
bool g_have_tick = false;
uint64_t g_prev_cycles[kN][kBuckets], g_prev_calls[kN][kBuckets];
uint64_t g_prev_cb[7];
double g_cyc_per_ms = 0;

double cycles_per_ms()
{
    LARGE_INTEGER f, q0, q1;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&q0);
    uint64_t c0 = __rdtsc();
    Sleep(20);
    QueryPerformanceCounter(&q1);
    uint64_t c1 = __rdtsc();
    double ms = double(q1.QuadPart - q0.QuadPart) * 1000.0 / double(f.QuadPart);
    return double(c1 - c0) / ms;
}

const char* bucket_name(int b)
{
    static const char* const kNames[kBuckets] = {"",
                                                 "walking <0.5 km", "walking 0.5-1 km", "walking 1-2 km", "walking 2+ km",
                                                 "seated <0.5 km", "seated 0.5-1 km", "seated 1-2 km", "seated 2+ km",
                                                 "no player known", "second view"};
    return kNames[b];
}

void log_now(ULONGLONG now)
{
    double secs = (now - g_last_log) / 1000.0;
    uint32_t tick = 0;
    bool have = host_tick(tick);
    double ticks = have && g_have_tick && tick > g_last_tick ? double(tick - g_last_tick) : 0.0;
    g_last_tick = tick;
    g_have_tick = have;
    double per = ticks > 0 ? ticks : secs; // per tick, else per second
    const char* unit = ticks > 0 ? "tick" : "s";

    uint64_t cyc[kN][kBuckets] = {}, calls[kN][kBuckets] = {};
    int threads[kN] = {};
    int na = std::min(g_nacc.load(), kMaxThreads);
    for (int t = 0; t < na; t++) {
        Acc* a = g_acc[t];
        if (!a) continue;
        for (int i = 0; i < kN; i++) {
            bool any = false;
            for (int b = 0; b < kBuckets; b++) {
                cyc[i][b] += a->cycles[i][b];
                calls[i][b] += a->calls[i][b];
                any |= a->calls[i][b] != 0;
            }
            threads[i] += any;
        }
    }
    swlog("probes 5s: %.0f host ticks in %.1f s (%.1f per s); per %s, inclusive:", ticks, secs, ticks / secs, unit);
    for (int i = 0; i < kN; i++) {
        uint64_t c = 0, n = 0;
        for (int b = 0; b < kBuckets; b++) {
            c += cyc[i][b] - g_prev_cycles[i][b];
            n += calls[i][b] - g_prev_calls[i][b];
        }
        double ms = c / g_cyc_per_ms;
        swlog("  %-32s %s  %7.3f ms  %8.1f calls  %7.2f us/call  %d thread%s", kProbes[i].what, kProbes[i].name, ms / per, n / per,
              n ? ms * 1000.0 / n : 0.0, threads[i], threads[i] == 1 ? "" : "s");
        if (kProbes[i].kind == kCharMark || kProbes[i].kind == kCharSub || kProbes[i].kind == kAiChar) {
            std::string line;
            for (int b = 1; b < kBuckets; b++) {
                uint64_t bc = cyc[i][b] - g_prev_cycles[i][b], bn = calls[i][b] - g_prev_calls[i][b];
                if (!bn) continue;
                char buf[96];
                snprintf(buf, sizeof buf, "%s%s %.3f ms / %.1f", line.empty() ? "" : ", ", bucket_name(b), bc / g_cyc_per_ms / per, bn / per);
                line += buf;
            }
            uint64_t b0 = calls[i][0] - g_prev_calls[i][0];
            if (b0) {
                char buf[96];
                snprintf(buf, sizeof buf, "%sother %.3f ms / %.1f", line.empty() ? "" : ", ", (cyc[i][0] - g_prev_cycles[i][0]) / g_cyc_per_ms / per,
                         b0 / per);
                line += buf;
            }
            if (!line.empty()) swlog("    by character (ms / calls per %s): %s", unit, line.c_str());
        }
    }
    uint64_t cb[7] = {g_cb_calls.load(), g_cb_off.load(), g_cb_manifolds.load(), g_cb_owned.load(),
                      g_cb_with_points.load(), g_cb_points.load(), g_cb_skippable.load()};
    uint64_t d[7];
    for (int k = 0; k < 7; k++) d[k] = cb[k] - g_prev_cb[k];
    if (d[0]) {
        uint64_t on = d[0] - d[1];
        swlog("  A manifolds per callback: %.0f, owners on both bodies %.0f, with contact points %.0f (%.1f%%), points %.0f;"
              " empty with owners (what a skip would drop) %.0f (%.1f%%); callbacks %llu, flag off %llu, read failures %llu",
              on ? double(d[2]) / on : 0.0, on ? double(d[3]) / on : 0.0, on ? double(d[4]) / on : 0.0, d[2] ? 100.0 * d[4] / d[2] : 0.0,
              on ? double(d[5]) / on : 0.0, on ? double(d[6]) / on : 0.0, d[2] ? 100.0 * d[6] / d[2] : 0.0,
              static_cast<unsigned long long>(d[0]), static_cast<unsigned long long>(d[1]),
              static_cast<unsigned long long>(g_cb_failed.load()));
    }
    for (int k = 0; k < 7; k++) g_prev_cb[k] = cb[k];
    memcpy(g_prev_cycles, cyc, sizeof cyc);
    memcpy(g_prev_calls, calls, sizeof calls);
    if (g_lost.load() || g_mismatch.load())
        swlog("  probes: %llu calls nested too deep, %llu unpaired (both should stay 0)", static_cast<unsigned long long>(g_lost.load()),
              static_cast<unsigned long long>(g_mismatch.load()));
}

void maybe_log()
{
    ULONGLONG now = GetTickCount64();
    if (now - g_last_log < 5000) return;
    bool expected = false;
    if (!g_logging.compare_exchange_strong(expected, true)) return;
    if (now - g_last_log >= 5000) {
        if (g_last_log) log_now(now);
        else { // first call: baseline only
            uint32_t tick = 0;
            g_have_tick = host_tick(tick);
            g_last_tick = tick;
        }
        g_last_log = now;
    }
    g_logging.store(false);
}

bool install()
{
    if (!GetPrivateProfileIntW(L"probes", L"enabled", 0, settings_ini())) return true;
    g_cyc_per_ms = cycles_per_ms();
    g_cb_flag = game_ptr<const uint8_t*>(0x140ceb6ff);
    g_block = static_cast<uint8_t*>(VirtualAlloc(nullptr, kBlockSize, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE));
    if (!g_block) {
        swlog("probes: no memory for the stubs; off");
        return true;
    }
    size_t unwind_at = kN * kStubSlot;
    size_t pdata_at = (unwind_at + 8 + 15) & ~size_t(15);
    g_pdata = reinterpret_cast<RUNTIME_FUNCTION*>(g_block + pdata_at);
    call_stub::make_call_stub_unwind(g_block + unwind_at);
    for (int i = 0; i < kN; i++) {
        size_t len = call_stub::make_call_stub(g_block + i * kStubSlot, static_cast<uint32_t>(i), &g_tramp[i],
                                                reinterpret_cast<void*>(&cp_enter), reinterpret_cast<void*>(&cp_leave));
        g_pdata[i].BeginAddress = static_cast<DWORD>(i * kStubSlot);
        g_pdata[i].EndAddress = static_cast<DWORD>(i * kStubSlot + len);
        g_pdata[i].UnwindData = static_cast<DWORD>(unwind_at);
        g_name_store[i] = std::string("probe ") + kProbes[i].name;
        g_names[i] = g_name_store[i].c_str();
    }
    FlushInstructionCache(GetCurrentProcess(), g_block, kBlockSize);
    if (!RtlAddFunctionTable(g_pdata, kN, reinterpret_cast<DWORD64>(g_block)))
        swlog("probes: RtlAddFunctionTable failed (exceptions can't unwind through the stubs)");
    g_range = SwmlCodeRange{reinterpret_cast<uintptr_t>(g_block), reinterpret_cast<uintptr_t>(g_block) + pdata_at + kN * sizeof(RUNTIME_FUNCTION),
                            g_pdata, static_cast<uint32_t>(kN), g_names, "physics_fixes probes"};
    g_published.store(true);
    int ok = 0;
    std::string failed;
    // hull_rays.cpp hooks the water rays (one hook per address): it logs their time itself
    const bool rays_deferred = GetPrivateProfileIntW(L"hull_rays", L"defer", 1, settings_ini()) != 0;
    for (int i = 0; i < kN; i++) {
        if (rays_deferred && kProbes[i].address == 0x1402f4950) {
            failed += std::string(failed.empty() ? "" : ", ") + kProbes[i].name + " (hull_rays has it)";
            continue;
        }
        if (hook_game(kProbes[i].address, g_block + i * kStubSlot, &g_tramp[i])) ok++;
        else failed += std::string(failed.empty() ? "" : ", ") + kProbes[i].name;
    }
    swlog("probes: %d of %d functions timed (\"probes 5s\" lines, log only)%s%s", ok, kN, failed.empty() ? "" : "; FAILED: ", failed.c_str());
    return true;
}

HookRegistrar reg{{"probes", /*needs_known_build=*/true, &install}};

} // namespace

extern "C" __declspec(dllexport) const SwmlCodeRange* __cdecl swml_code_ranges(uint32_t* count)
{
    if (!g_published.load()) {
        if (count) *count = 0;
        return nullptr;
    }
    if (count) *count = 1;
    return &g_range;
}
