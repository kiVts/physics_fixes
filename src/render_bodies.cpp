
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <intrin.h>
#include <cstdint>
#include <unordered_map>

namespace {

constexpr uintptr_t kBodyBuffers = 0x1404a0e80, kQuadBuild = 0x1402fbd00;
constexpr ULONGLONG kRefreshMs = 500;

using BodyBuffers_t = void(__fastcall*)(uintptr_t body, uintptr_t a2, uintptr_t a3, uintptr_t a4, uintptr_t a5, void* a6);
BodyBuffers_t g_orig = nullptr;

struct Scan {
    bool eligible;
    uint32_t quad_count, quad_cap, quad_live;
    uint32_t part_count, part_live;
};

template <typename T>
T rd(uintptr_t a)
{
    return *reinterpret_cast<const T*>(a);
}

bool scan_raw(uintptr_t body, Scan* s)
{
    __try {
        *s = Scan{};
        s->eligible = rd<uintptr_t>(body + 0x718) != 0 && rd<uint8_t>(body + 0x1478) == 0;
        if (!s->eligible) return true;
        uintptr_t q = body + 0x5d0;
        s->quad_count = rd<uint32_t>(q + 0x40);
        s->quad_cap = rd<uint32_t>(q + 0x30);
        uintptr_t data = rd<uintptr_t>(q + 0x38);
        for (uint32_t i = 0; i < s->quad_count && i < s->quad_cap; i++)
            if (rd<uint8_t>(data + uintptr_t(i) * 0x78 + 4)) s->quad_live++;
        uintptr_t p = body + 0x620;
        s->part_count = rd<uint32_t>(p + 0x10);
        uintptr_t pdata = rd<uintptr_t>(p);
        uint32_t cap = rd<uint32_t>(p + 8), head = rd<uint32_t>(p + 0xc);
        for (uint32_t i = 0; i < s->part_count && cap; i++)
            if (rd<uint32_t>(pdata + uintptr_t((head + i) % cap) * 0x38 + 0x24)) s->part_live++;
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

// --- skipping all-dead quad lists -------------------------------------------------------------------------------
using QuadBuild_t = void(__fastcall*)(uintptr_t self, uintptr_t events, void* origin);
QuadBuild_t g_quad_orig = nullptr;
bool g_skip = false;

bool quads_dead_raw(uintptr_t self, uint32_t* count, uintptr_t* geom, bool* dead)
{
    __try {
        *count = rd<uint32_t>(self + 0x40);
        *geom = rd<uintptr_t>(self + 0x20);
        uint32_t cap = rd<uint32_t>(self + 0x30);
        uintptr_t data = rd<uintptr_t>(self + 0x38);
        *dead = true;
        for (uint32_t i = 0; i < *count && i < cap; i++)
            if (rd<uint8_t>(data + uintptr_t(i) * 0x78 + 4)) {
                *dead = false;
                break;
            }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

struct ZeroPosted {
    uintptr_t geom;
    uint32_t count;
    ULONGLONG posted, seen;
};
std::unordered_map<uintptr_t, ZeroPosted> g_zero; // quad list -> its last post was all zeros
SRWLOCK g_zero_lock = SRWLOCK_INIT;                // render prep runs on the update thread; cheap insurance
uint64_t g_skipped = 0, g_zero_posts = 0;
ULONGLONG g_last_prune = 0;

void __fastcall quad_detour(uintptr_t self, uintptr_t events, void* origin)
{
    uint32_t count = 0;
    uintptr_t geom = 0;
    bool dead = false;
    if (!g_skip || !quads_dead_raw(self, &count, &geom, &dead) || !count) {
        g_quad_orig(self, events, origin);
        return;
    }
    ULONGLONG now = GetTickCount64();
    AcquireSRWLockExclusive(&g_zero_lock);
    if (!dead) {
        g_zero.erase(self);
        ReleaseSRWLockExclusive(&g_zero_lock);
        g_quad_orig(self, events, origin);
        return;
    }
    auto it = g_zero.find(self);
    if (it != g_zero.end() && it->second.geom == geom && it->second.count == count && now - it->second.posted < kRefreshMs) {
        it->second.seen = now;
        g_skipped++;
        ReleaseSRWLockExclusive(&g_zero_lock);
        return; // the render side already holds this all-zero buffer
    }
    ReleaseSRWLockExclusive(&g_zero_lock);
    g_quad_orig(self, events, origin);
    AcquireSRWLockExclusive(&g_zero_lock);
    g_zero[self] = ZeroPosted{geom, count, now, now};
    g_zero_posts++;
    if (now - g_last_prune > 5000) {
        for (auto i = g_zero.begin(); i != g_zero.end();) i = now - i->second.seen > 2000 ? g_zero.erase(i) : std::next(i);
        g_last_prune = now;
    }
    ReleaseSRWLockExclusive(&g_zero_lock);
}

// Update thread only (render scene preparation): plain counters.
struct Stats {
    uint64_t calls, eligible, bad;
    uint64_t quad_bodies, quad_count, quad_cap, quad_live, quad_dead_bodies; // bodies with count != 0 / of them nothing live
    uint64_t part_bodies, part_count, part_live, part_dead_bodies;
    uint64_t cycles, cycles_dead; // all calls / calls where neither buffer had anything live
} g_st;
ULONGLONG g_window = 0;
uint64_t g_window_tsc = 0;

void maybe_log()
{
    ULONGLONG now = GetTickCount64();
    if (!g_window) {
        g_window = now;
        g_window_tsc = __rdtsc();
        return;
    }
    if (now - g_window < 5000) return;
    double secs = (now - g_window) / 1000.0;
    double cyc_ms = double(__rdtsc() - g_window_tsc) / (secs * 1000.0);
    const Stats& s = g_st;
    auto avg = [](uint64_t a, uint64_t n) { return n ? double(a) / n : 0.0; };
    swlog("render bodies 5s: %.0f calls/s (%.0f drawn with buffers), %.2f ms/s in FUN_1404a0e80 (%.2f ms/s on bodies with nothing live)%s",
          s.calls / secs, s.eligible / secs, s.cycles / cyc_ms / secs, s.cycles_dead / cyc_ms / secs,
          s.bad ? " (some bodies unreadable)" : "");
    swlog("  quads (body+0x5d0): %.0f/s bodies with entries (%.0f/s none live), avg %.1f entries, %.1f capacity, %.1f live",
          s.quad_bodies / secs, s.quad_dead_bodies / secs, avg(s.quad_count, s.quad_bodies), avg(s.quad_cap, s.quad_bodies),
          avg(s.quad_live, s.quad_bodies));
    swlog("  particles (body+0x620): %.0f/s bodies with entries (%.0f/s none live), avg %.1f entries, %.1f live",
          s.part_bodies / secs, s.part_dead_bodies / secs, avg(s.part_count, s.part_bodies), avg(s.part_live, s.part_bodies));
    if (g_skip) {
        AcquireSRWLockExclusive(&g_zero_lock);
        swlog("  skip_dead_quads: %.0f/s skipped (render side already had the zeros), %.0f/s all-zero buffers posted, %zu lists tracked",
              g_skipped / secs, g_zero_posts / secs, g_zero.size());
        g_skipped = g_zero_posts = 0;
        ReleaseSRWLockExclusive(&g_zero_lock);
    }
    g_st = Stats{};
    g_window = now;
    g_window_tsc = __rdtsc();
}

void __fastcall detour(uintptr_t body, uintptr_t a2, uintptr_t a3, uintptr_t a4, uintptr_t a5, void* a6)
{
    // plain counters: measure on the first thread only (with parallel render prep, workers call this too)
    static DWORD s_tid = GetCurrentThreadId();
    if (GetCurrentThreadId() != s_tid) {
        g_orig(body, a2, a3, a4, a5, a6);
        return;
    }
    Scan sc;
    bool ok = scan_raw(body, &sc);
    uint64_t t0 = __rdtsc();
    g_orig(body, a2, a3, a4, a5, a6);
    uint64_t dt = __rdtsc() - t0;
    Stats& s = g_st;
    s.calls++;
    s.cycles += dt;
    if (!ok) s.bad++;
    else if (sc.eligible) {
        s.eligible++;
        if (sc.quad_count) {
            s.quad_bodies++, s.quad_count += sc.quad_count, s.quad_cap += sc.quad_cap, s.quad_live += sc.quad_live;
            if (!sc.quad_live) s.quad_dead_bodies++;
        }
        if (sc.part_count) {
            s.part_bodies++, s.part_count += sc.part_count, s.part_live += sc.part_live;
            if (!sc.part_live) s.part_dead_bodies++;
        }
        if (!sc.quad_live && !sc.part_live) s.cycles_dead += dt;
    }
    maybe_log();
}

bool install()
{
    g_skip = GetPrivateProfileIntW(L"render", L"skip_dead_quads", 1, settings_ini()) != 0;
    if (g_skip) {
        if (!hook_game(kQuadBuild, reinterpret_cast<void*>(&quad_detour), reinterpret_cast<void**>(&g_quad_orig))) {
            swlog("render bodies: hooking FUN_1402fbd00 FAILED; skip_dead_quads off");
            g_skip = false;
        } else {
            swlog("render bodies: skip_dead_quads on (all-dead quad lists posted once, then skipped while unchanged)");
        }
    }
    if (!GetPrivateProfileIntW(L"render", L"measure_body_buffers", 0, settings_ini())) {
        swlog("render bodies: measuring off in settings.ini");
        return true;
    }
    if (!hook_game(kBodyBuffers, reinterpret_cast<void*>(&detour), reinterpret_cast<void**>(&g_orig))) {
        swlog("render bodies: hooking FUN_1404a0e80 FAILED");
        return false;
    }
    swlog("render bodies: measuring the per-body effect buffers (\"render bodies 5s\" lines)");
    return true;
}

HookRegistrar reg{{"render_bodies", /*needs_known_build=*/true, &install}};

} // namespace
