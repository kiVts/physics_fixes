
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <cstdint>
#include <cstdio>


namespace {

using U = uint64_t;
constexpr uintptr_t kSimulate = 0x14073f890;   // (12 args) one game tick's simulate
constexpr uintptr_t kRenderPrep = 0x140749760; // (simulate this, its param_2, param_4, param_5): zone 0x1d
constexpr uintptr_t kSubmit = 0x14017d720;     // task submit (generic; many callers)
constexpr uintptr_t kLaunchRet = 0x1407439e4;
constexpr uintptr_t kHardImpact = 0x140559710; // contact callback, impulse > 10000 on a type-7 owner (physics thread)
constexpr uintptr_t kZoneBegin = 0x1401ad030, kZoneEnd = 0x1401ad0d0; // (simulate param_2 + 0x1a0, 0, zone)
constexpr U kPrepZone = 0x1d;

using Simulate_t = void(__fastcall*)(U, U, U, U, U, U, U, U, U, U, U, U);
using Prep_t = void(__fastcall*)(U, U, U, U);
using Submit_t = void(__fastcall*)(U, U, U, U);
using Impact_t = void(__fastcall*)(U, U, U, U, U, U);
Simulate_t o_simulate = nullptr;
Prep_t o_prep = nullptr;
Submit_t o_submit = nullptr;
Impact_t o_impact = nullptr;
using Zone_t = void(__fastcall*)(U, U, U);
Zone_t g_zone_begin = nullptr, g_zone_end = nullptr;

// update-thread callers of the submit function (first frame inside the game), for the 5 s line
constexpr int kCallers = 8;
uintptr_t g_caller[kCallers] = {};
uint64_t g_caller_n[kCallers] = {};
uintptr_t g_base = 0, g_image_end = 0;

uintptr_t g_launch_ret = 0;
int g_mode = 1;
SRWLOCK g_lock = SRWLOCK_INIT; // held while a deferred render prep runs

// update thread only
bool g_pending = false;
U g_args[4] = {};
DWORD g_update_tid = 0;

std::atomic<uint64_t> g_ticks[2], g_deferred{0}, g_at_end{0}, g_impact_waits{0};
std::atomic<uint64_t> g_prep_us[2];
LARGE_INTEGER g_freq;
ULONGLONG g_last_log = 0;

bool on_now()
{
    if (g_mode != 2) return g_mode == 1;
    return (GetTickCount64() / 30000) % 2 == 0;
}

LONGLONG qpc()
{
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
}

void run_pending()
{
    g_pending = false;
    LONGLONG t0 = qpc();
    AcquireSRWLockExclusive(&g_lock);
    // inside the game's own zone 0x1d, so its profiler still names this time
    g_zone_begin(g_args[1] + 0x1a0, 0, kPrepZone);
    o_prep(g_args[0], g_args[1], g_args[2], g_args[3]);
    g_zone_end(g_args[1] + 0x1a0, 0, kPrepZone);
    ReleaseSRWLockExclusive(&g_lock);
    g_prep_us[1].fetch_add(static_cast<uint64_t>((qpc() - t0) * 1000000 / g_freq.QuadPart), std::memory_order_relaxed);
}

void __fastcall prep_detour(U a1, U a2, U a3, U a4)
{
    if (GetCurrentThreadId() == g_update_tid && on_now() && !g_pending) {
        g_args[0] = a1, g_args[1] = a2, g_args[2] = a3, g_args[3] = a4;
        g_pending = true;
        return;
    }
    LONGLONG t0 = qpc();
    o_prep(a1, a2, a3, a4);
    g_prep_us[0].fetch_add(static_cast<uint64_t>((qpc() - t0) * 1000000 / g_freq.QuadPart), std::memory_order_relaxed);
}

// The detour may be entered through the core's hook chain, so the game's call site is found on the stack, not as
// our own return address.
uintptr_t game_caller(bool* is_launch)
{
    void* frames[8];
    USHORT n = RtlCaptureStackBackTrace(1, 8, frames, nullptr);
    uintptr_t first = 0;
    for (USHORT i = 0; i < n; i++) {
        auto a = reinterpret_cast<uintptr_t>(frames[i]);
        if (a == g_launch_ret) *is_launch = true;
        if (!first && a >= g_base && a < g_image_end) first = a;
    }
    return first;
}

void count_caller(uintptr_t a)
{
    for (int i = 0; i < kCallers; i++) {
        if (g_caller[i] == a) { g_caller_n[i]++; return; }
        if (!g_caller[i]) { g_caller[i] = a; g_caller_n[i] = 1; return; }
    }
}

void __fastcall submit_detour(U a1, U a2, U a3, U a4)
{
    o_submit(a1, a2, a3, a4);
    if (GetCurrentThreadId() != g_update_tid) return;
    bool launch = false;
    count_caller(game_caller(&launch));
    if (launch && g_pending) {
        g_deferred.fetch_add(1, std::memory_order_relaxed);
        run_pending();
    }
}

void log_maybe()
{
    ULONGLONG now = GetTickCount64();
    if (!g_last_log) g_last_log = now;
    if (now - g_last_log < 5000) return;
    double secs = (now - g_last_log) / 1000.0;
    g_last_log = now;
    uint64_t on = g_ticks[1].exchange(0), off = g_ticks[0].exchange(0);
    uint64_t us_on = g_prep_us[1].exchange(0), us_off = g_prep_us[0].exchange(0);
    swlog("overlap physics 5s: %s; overlapped %llu ticks (%.1f/s, render prep %.2f ms), serial %llu ticks (%.1f/s, render prep "
          "%.2f ms); %llu run after the launch, %llu at simulate's end, %llu hard impacts waited",
          g_mode == 2 ? "A/B" : "on", on, on / secs, on ? us_on / 1000.0 / on : 0.0, off, off / secs,
          off ? us_off / 1000.0 / off : 0.0, g_deferred.exchange(0), g_at_end.exchange(0), g_impact_waits.exchange(0));
    char buf[512];
    int len = snprintf(buf, sizeof(buf), "overlap physics 5s: task submits on the update thread by caller:");
    for (int i = 0; i < kCallers && g_caller[i] && len < 480; i++) {
        uintptr_t gh = game_ghidra(g_caller[i]);
        len += snprintf(buf + len, sizeof(buf) - len, " %llx%s x%llu", static_cast<unsigned long long>(gh ? gh : g_caller[i]),
                        g_caller[i] == g_launch_ret ? " (launch)" : "", static_cast<unsigned long long>(g_caller_n[i]));
        g_caller[i] = 0, g_caller_n[i] = 0;
    }
    swlog("%s", buf);
}

void __fastcall simulate_detour(U a1, U a2, U a3, U a4, U a5, U a6, U a7, U a8, U a9, U a10, U a11, U a12)
{
    g_update_tid = GetCurrentThreadId();
    bool on = on_now();
    o_simulate(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12);
    if (g_pending) { // no launch this tick (paused, loading, ...): still draw it
        g_at_end.fetch_add(1, std::memory_order_relaxed);
        run_pending();
    }
    g_ticks[on].fetch_add(1, std::memory_order_relaxed);
    log_maybe();
}

void __fastcall impact_detour(U a1, U a2, U a3, U a4, U a5, U a6)
{
    if (!TryAcquireSRWLockExclusive(&g_lock)) {
        g_impact_waits.fetch_add(1, std::memory_order_relaxed);
        AcquireSRWLockExclusive(&g_lock);
    }
    o_impact(a1, a2, a3, a4, a5, a6);
    ReleaseSRWLockExclusive(&g_lock);
}

bool install()
{
    g_mode = static_cast<int>(GetPrivateProfileIntW(L"render", L"overlap_physics", 0, settings_ini()));
    if (g_mode == 0) {
        swlog("overlap physics: off in settings.ini");
        return true;
    }
    QueryPerformanceFrequency(&g_freq);
    g_base = game_base();
    auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(g_base + reinterpret_cast<const IMAGE_DOS_HEADER*>(g_base)->e_lfanew);
    g_image_end = g_base + nt->OptionalHeader.SizeOfImage;
    g_zone_begin = game_ptr<Zone_t>(kZoneBegin);
    g_zone_end = game_ptr<Zone_t>(kZoneEnd);
    if (!g_zone_begin || !g_zone_end) return false;
    // the launch site: a direct call (E8 rel32) to the submit function right before the return address
    g_launch_ret = game_live(kLaunchRet);
    uintptr_t submit = game_live(kSubmit);
    auto* call = reinterpret_cast<const uint8_t*>(g_launch_ret - 5);
    if (!g_launch_ret || !submit || call[0] != 0xE8 || g_launch_ret + *reinterpret_cast<const int32_t*>(call + 1) != submit) {
        swlog("overlap physics: simulate's physics launch is not where expected; off");
        return false;
    }
    // the impact lock first: from then on the physics thread can never run that path during a deferred render prep
    if (!hook_game(kHardImpact, reinterpret_cast<void*>(&impact_detour), reinterpret_cast<void**>(&o_impact)) ||
        !hook_game(kSubmit, reinterpret_cast<void*>(&submit_detour), reinterpret_cast<void**>(&o_submit)) ||
        !hook_game(kSimulate, reinterpret_cast<void*>(&simulate_detour), reinterpret_cast<void**>(&o_simulate)) ||
        !hook_game(kRenderPrep, reinterpret_cast<void*>(&prep_detour), reinterpret_cast<void**>(&o_prep)))
        return false;
    swlog("overlap physics: %s (render prep runs after the physics launch, hard impacts wait for it)",
          g_mode == 2 ? "A/B, 30 s each way" : "on");
    return true;
}

HookRegistrar reg{{"overlap physics", /*needs_known_build=*/true, &install}};

} // namespace
