
#include "contact_guard.h"
#include "parallel.h"
#include "native_finally.h"
#include "stability_policy.h"
#include "world_lock.h"

#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "game.h"

#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <cstdint>
#include <cstring>

namespace {

constexpr uintptr_t kRayTest = 0x14015a580, kCollide = 0x1400cd3c0, kSetAabb = 0x1400ccce0,
                    kIntegrateBodies = 0x140135d70, kContactCallback = 0x1405ed160;
bool g_on = false;
WorldLock g_lock;
WorldLock::Stats g_lock_stats;
thread_local int t_exclusive = 0;
ContactGuard g_guard; // complete callback versus the newly overlapped logic phase
std::atomic<uint64_t> g_drains{0}, g_drain_wait{0}, g_drain_wait_max{0}, g_cb_waits{0}, g_cb_wait{0}, g_cb_wait_max{0};
std::atomic<uint64_t> g_reads{0};
// lock cost: the physics thread's exclusive holds (readers' and the writer's waits are in g_lock_stats)
std::atomic<uint64_t> g_excl{0}, g_excl_cycles{0};
thread_local uint64_t t_excl_t0 = 0;

struct Exclusive {
    static void begin()
    {
        if (t_exclusive++ == 0) {
            g_lock.write_begin(&g_lock_stats);
            t_excl_t0 = __rdtsc();
        }
    }
    static void end()
    {
        if (--t_exclusive == 0) {
            uint64_t d = __rdtsc() - t_excl_t0;
            g_lock.write_end();
            g_excl.fetch_add(1, std::memory_order_relaxed);
            g_excl_cycles.fetch_add(d, std::memory_order_relaxed);
        }
    }
};

using RayTest_t = void(__fastcall*)(void* world, const void* from, const void* to, void* callback);
using Collide_t = uint64_t(__fastcall*)(void* bp, void* dispatcher);
using SetAabb_t = void(__fastcall*)(void* bp, void* proxy, const double* mn, const double* mx, void* dispatcher);
using IntegrateBodies_t = void(__fastcall*)(void* world, void** bodies, int count, double time_step);
using Four_t = uint64_t(__fastcall*)(void*, void*, void*, void*);

RayTest_t g_ray = nullptr;
Collide_t g_collide = nullptr;
SetAabb_t g_set_aabb = nullptr;
IntegrateBodies_t g_integrate = nullptr;
Four_t g_callback = nullptr;

void __fastcall ray_detour(void* world, const void* from, const void* to, void* callback)
{
    world_read_begin();
    native_finally([&] { g_ray(world, from, to, callback); }, [] { world_read_end(); });
}

uint64_t __fastcall collide_detour(void* bp, void* dispatcher)
{
    if (!g_on) return g_collide(bp, dispatcher);
    Exclusive::begin();
    return native_finally([&] { return g_collide(bp, dispatcher); }, [] { Exclusive::end(); });
}

void __fastcall set_aabb_detour(void* bp, void* proxy, const double* mn, const double* mx, void* dispatcher)
{
    if (!g_on) return g_set_aabb(bp, proxy, mn, mx, dispatcher);
    Exclusive::begin();
    native_finally([&] { g_set_aabb(bp, proxy, mn, mx, dispatcher); }, [] { Exclusive::end(); });
}

void __fastcall integrate_detour(void* world, void** bodies, int count, double time_step)
{
    if (!g_on) return g_integrate(world, bodies, count, time_step);
    Exclusive::begin();
    native_finally([&] { g_integrate(world, bodies, count, time_step); }, [] { Exclusive::end(); });
}

void add_wait(std::atomic<uint64_t>& n, std::atomic<uint64_t>& sum, std::atomic<uint64_t>& max, uint64_t d)
{
    n.fetch_add(1, std::memory_order_relaxed);
    sum.fetch_add(d, std::memory_order_relaxed);
    uint64_t m = max.load(std::memory_order_relaxed);
    while (d > m && !max.compare_exchange_weak(m, d, std::memory_order_relaxed)) {}
}

uint64_t __fastcall callback_detour(void* a1, void* a2, void* a3, void* a4)
{
    if (!g_on) return g_callback(a1, a2, a3, a4);
    // Rings first: a callback still waiting here hasn't loaded any owner, so despawns needn't wait for it.
    uint64_t t0 = __rdtsc();
    if (g_guard.lock()) add_wait(g_cb_waits, g_cb_wait, g_cb_wait_max, __rdtsc() - t0);
    g_guard.callback_begin();
    return native_finally([&] { return g_callback(a1, a2, a3, a4); }, [] {
        g_guard.callback_end();
        g_guard.unlock();
    });
}

DWORD WINAPI log_loop(void*)
{
    for (;;) {
        Sleep(30000);
        double cyc_per_us = 0;
        {
            LARGE_INTEGER f, q0, q1;
            QueryPerformanceFrequency(&f);
            QueryPerformanceCounter(&q0);
            uint64_t c0 = __rdtsc();
            Sleep(20);
            QueryPerformanceCounter(&q1);
            cyc_per_us = double(__rdtsc() - c0) / (double(q1.QuadPart - q0.QuadPart) * 1e6 / double(f.QuadPart));
        }
        swlog("parallel guard 30s: %llu world reads", (unsigned long long)g_reads.exchange(0));
        WorldLock::Stats& st = g_lock_stats;
        uint64_t rb = st.read_blocked.exchange(0), rbc = st.read_blocked_cycles.exchange(0), rbm = st.read_blocked_max.exchange(0);
        uint64_t ww = st.write_waits.exchange(0), wwc = st.write_wait_cycles.exchange(0), wwm = st.write_wait_max.exchange(0);
        uint64_t co = st.courtesy.exchange(0), cot = st.courtesy_timeouts.exchange(0);
        uint64_t ex = g_excl.exchange(0), exc = g_excl_cycles.exchange(0);
        swlog("parallel guard 30s: readers blocked %llu times, %.1f ms/s blocked in total (%.1f us avg, %.1f us max); "
              "exclusive holds %llu, %.1f ms/s held (%.1f us avg)",
              (unsigned long long)rb, rbc / cyc_per_us / 1000.0 / 30.0, rb ? rbc / cyc_per_us / rb : 0.0, rbm / cyc_per_us,
              (unsigned long long)ex, exc / cyc_per_us / 1000.0 / 30.0, ex ? exc / cyc_per_us / ex : 0.0);
        swlog("parallel guard 30s: physics thread waited for readers %llu times, %.1f ms/s in total (%.1f us avg, %.1f us "
              "max); let waiting readers in first %llu times (%llu gave up waiting for a descheduled reader)",
              (unsigned long long)ww, wwc / cyc_per_us / 1000.0 / 30.0, ww ? wwc / cyc_per_us / ww : 0.0, wwm / cyc_per_us,
              (unsigned long long)co, (unsigned long long)cot);
        uint64_t dn = g_drains.exchange(0), dw = g_drain_wait.exchange(0), dm = g_drain_wait_max.exchange(0);
        uint64_t cn = g_cb_waits.exchange(0), cw = g_cb_wait.exchange(0), cm = g_cb_wait_max.exchange(0);
        swlog("contact phase 30s: logic phases %llu, waited %.1f us avg, %.1f us max; callbacks %llu, waited %.1f us avg, %.1f us max",
              (unsigned long long)dn, dn ? dw / cyc_per_us / dn : 0.0, dm / cyc_per_us, (unsigned long long)cn,
              cn ? cw / cyc_per_us / cn : 0.0, cm / cyc_per_us);
    }
}

bool install()
{
    if (!kEarlyLaunchAudited) return true; // no dormant locks in stable mode
    const bool requested = GetPrivateProfileIntW(L"parallel", L"early_launch", 1, settings_ini()) == 1;
    if (!requested) return true;
    g_on = false; // publish only after every hook and all hooks succeed
    const bool ok =
              hook_game(kRayTest, reinterpret_cast<void*>(&ray_detour), reinterpret_cast<void**>(&g_ray)) &&
              hook_game(kCollide, reinterpret_cast<void*>(&collide_detour), reinterpret_cast<void**>(&g_collide)) &&
              hook_game(kSetAabb, reinterpret_cast<void*>(&set_aabb_detour), reinterpret_cast<void**>(&g_set_aabb)) &&
              hook_game(kIntegrateBodies, reinterpret_cast<void*>(&integrate_detour), reinterpret_cast<void**>(&g_integrate)) &&
              hook_game(kContactCallback, reinterpret_cast<void*>(&callback_detour), reinterpret_cast<void**>(&g_callback));
    if (!ok) {
        // early_launch.cpp checks parallel_on() at its own install: without the guard it must not run
        g_on = false;
        swlog("parallel guard: a hook FAILED: the early launch stays off");
        return false;
    }
    g_on = true;
    if (HANDLE thread = CreateThread(nullptr, 0, log_loop, nullptr, 0, nullptr)) CloseHandle(thread);
    swlog("parallel guard: on (world query lock; contact callbacks excluded for the whole added logic window; scoped release before join)");
    return true;
}

HookRegistrar reg{{"parallel guard", /*needs_known_build=*/true, &install}};

} // namespace

bool parallel_on() { return g_on; }

void world_read_begin()
{
    if (!g_on || t_exclusive) return;
    g_reads.fetch_add(1, std::memory_order_relaxed);
    g_lock.read_begin(&g_lock_stats);
}

void world_read_end()
{
    if (!g_on || t_exclusive) return;
    g_lock.read_end();
}

// Called only by the update-thread invocation that owns the new overlap window.
bool parallel_phase_begin()
{
    if (!g_on) return false;
    const uint64_t start = __rdtsc();
    if (g_guard.lock()) add_wait(g_drains, g_drain_wait, g_drain_wait_max, __rdtsc() - start);
    return true;
}

void parallel_phase_end()
{
    g_guard.unlock();
}
