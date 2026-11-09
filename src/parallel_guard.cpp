
#include "parallel.h"
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
constexpr uintptr_t kDestroyHelpers[] = {
    0x1401a5010, // box
    0x1401a53f0, // sphere
    0x1401a5630, // capsule
    0x1402a6f20, // convex mesh
    0x1401a5a30, // triangle mesh
    0x1402c3400, // triangle mesh (tiles)
    0x1401a5e60, // compound shape
    0x1401a61f0, // ghost
    0x1401a60a0, // rope
};
constexpr int kHelpers = sizeof(kDestroyHelpers) / sizeof(kDestroyHelpers[0]);
constexpr uintptr_t kCylinderOwnerFree = 0x14020c490, kCylinderOwnerFreeRet = 0x1405330d7;
constexpr uintptr_t kLogic = 0x1405e8fb0, kZoneBegin = 0x1401ad030, kZoneEnd = 0x1401ad0d0;
constexpr uint8_t kContactDrainZone = 0x37;

bool g_on = false;
WorldLock g_lock;
WorldLock::Stats g_lock_stats;
thread_local int t_exclusive = 0;
std::atomic<int> g_in_callback{0};
SRWLOCK g_rings = SRWLOCK_INIT;
std::atomic<uint64_t> g_drains{0}, g_drain_wait{0}, g_drain_wait_max{0}, g_cb_waits{0}, g_cb_wait{0}, g_cb_wait_max{0};
std::atomic<uint64_t> g_waits{0}, g_wait_cycles{0}, g_wait_max{0}, g_reads{0};
// lock cost: the physics thread's exclusive holds (readers' and the writer's waits are in g_lock_stats)
std::atomic<uint64_t> g_excl{0}, g_excl_cycles{0};
thread_local uint64_t t_excl_t0 = 0;
uintptr_t g_cyl_ret = 0;

struct Exclusive {
    Exclusive()
    {
        if (t_exclusive++ == 0) {
            g_lock.write_begin(&g_lock_stats);
            t_excl_t0 = __rdtsc();
        }
    }
    ~Exclusive()
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
Four_t g_callback = nullptr, g_cyl_free = nullptr, g_zone_begin = nullptr, g_zone_end = nullptr;
Four_t g_helper[kHelpers]{};

void __fastcall ray_detour(void* world, const void* from, const void* to, void* callback)
{
    world_read_begin();
    g_ray(world, from, to, callback);
    world_read_end();
}

uint64_t __fastcall collide_detour(void* bp, void* dispatcher)
{
    Exclusive x;
    return g_collide(bp, dispatcher);
}

void __fastcall set_aabb_detour(void* bp, void* proxy, const double* mn, const double* mx, void* dispatcher)
{
    Exclusive x; // per call: rays may run between two objects' updates
    g_set_aabb(bp, proxy, mn, mx, dispatcher);
}

void __fastcall integrate_detour(void* world, void** bodies, int count, double time_step)
{
    Exclusive x;
    g_integrate(world, bodies, count, time_step);
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
    // Rings first: a callback still waiting here hasn't loaded any owner, so despawns needn't wait for it.
    uint64_t t0 = __rdtsc();
    AcquireSRWLockExclusive(&g_rings);
    add_wait(g_cb_waits, g_cb_wait, g_cb_wait_max, __rdtsc() - t0);
    g_in_callback.fetch_add(1, std::memory_order_seq_cst);
    uint64_t r = g_callback(a1, a2, a3, a4);
    g_in_callback.fetch_sub(1, std::memory_order_release);
    ReleaseSRWLockExclusive(&g_rings);
    return r;
}

uint64_t __fastcall drain_begin(void* a1, void* a2, void* a3, void* a4)
{
    uint64_t r = g_zone_begin(a1, a2, a3, a4);
    uint64_t t0 = __rdtsc();
    AcquireSRWLockExclusive(&g_rings);
    add_wait(g_drains, g_drain_wait, g_drain_wait_max, __rdtsc() - t0);
    return r;
}

uint64_t __fastcall drain_end(void* a1, void* a2, void* a3, void* a4)
{
    ReleaseSRWLockExclusive(&g_rings);
    return g_zone_end(a1, a2, a3, a4);
}

uint8_t* alloc_near(uintptr_t base, size_t size)
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    uintptr_t gran = si.dwAllocationGranularity;
    for (uintptr_t d = 0x10000000; d < 0x70000000; d += gran) {
        uintptr_t a = (base + d) & ~(gran - 1);
        if (void* p = VirtualAlloc(reinterpret_cast<void*>(a), size, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE))
            return static_cast<uint8_t*>(p);
    }
    return nullptr;
}

uint8_t* find_zone_call(uintptr_t target, uint8_t zone)
{
    uintptr_t logic = game_live(kLogic);
    if (!logic || !target) return nullptr;
    DWORD64 image = 0;
    PRUNTIME_FUNCTION rf = RtlLookupFunctionEntry(logic, &image, nullptr);
    if (!rf || image + rf->BeginAddress != logic) return nullptr;
    const uint8_t* end = reinterpret_cast<const uint8_t*>(image + rf->EndAddress);
    const uint8_t pat[] = {0x33, 0xD2, 0x44, 0x8D, 0x42, zone, 0x48, 0x8B, 0xCF, 0xE8};
    uint8_t* found = nullptr;
    for (auto* p = reinterpret_cast<uint8_t*>(logic); p + sizeof pat + 4 <= end; p++) {
        if (memcmp(p, pat, sizeof pat)) continue;
        int32_t rel;
        memcpy(&rel, p + sizeof pat, 4);
        uint8_t* site = p + sizeof pat - 1;
        if (reinterpret_cast<uintptr_t>(site + 5 + rel) != target) continue;
        if (found) return nullptr;
        found = site;
    }
    return found;
}

bool point_call_at(uint8_t* site, uint8_t* stub, void* detour)
{
    // mov rax, detour / jmp rax   (rax is free at a call; the detour sees the game's own arguments)
    uint8_t* p = stub;
    *p++ = 0x48, *p++ = 0xB8;
    uint64_t target = reinterpret_cast<uint64_t>(detour);
    memcpy(p, &target, 8), p += 8;
    *p++ = 0xFF, *p++ = 0xE0;
    FlushInstructionCache(GetCurrentProcess(), stub, p - stub);
    int64_t rel64 = stub - (site + 5);
    if (rel64 != static_cast<int32_t>(rel64)) return false;
    int32_t rel = static_cast<int32_t>(rel64);
    DWORD old;
    if (!VirtualProtect(site, 5, PAGE_EXECUTE_READWRITE, &old)) return false;
    memcpy(site + 1, &rel, 4);
    VirtualProtect(site, 5, old, &old);
    FlushInstructionCache(GetCurrentProcess(), site, 5);
    return true;
}

// After an owner pointer was zeroed: wait out a contact callback that may have loaded it.
void wait_callbacks()
{
    std::atomic_thread_fence(std::memory_order_seq_cst); // the zeroing store before the check
    if (!g_in_callback.load(std::memory_order_acquire)) return;
    uint64_t t0 = __rdtsc();
    while (g_in_callback.load(std::memory_order_acquire)) YieldProcessor();
    uint64_t d = __rdtsc() - t0, m = g_wait_max.load();
    g_waits.fetch_add(1);
    g_wait_cycles.fetch_add(d);
    while (d > m && !g_wait_max.compare_exchange_weak(m, d)) {}
}

template <int I> uint64_t __fastcall helper_detour(void* scene, void* obj, void* a3, void* a4)
{
    uint64_t r = g_helper[I](scene, obj, a3, a4);
    wait_callbacks();
    return r;
}

void* const* helper_detours()
{
    static void* const d[] = {(void*)&helper_detour<0>, (void*)&helper_detour<1>, (void*)&helper_detour<2>,
                              (void*)&helper_detour<3>, (void*)&helper_detour<4>, (void*)&helper_detour<5>,
                              (void*)&helper_detour<6>, (void*)&helper_detour<7>, (void*)&helper_detour<8>};
    static_assert(sizeof(d) / sizeof(d[0]) == kHelpers, "one detour per helper");
    return d;
}

uint64_t __fastcall cyl_free_detour(void* a1, void* a2, void* a3, void* a4)
{
    if (reinterpret_cast<uintptr_t>(_ReturnAddress()) == g_cyl_ret) wait_callbacks();
    return g_cyl_free(a1, a2, a3, a4);
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
        uint64_t w = g_waits.exchange(0), c = g_wait_cycles.exchange(0), m = g_wait_max.exchange(0);
        swlog("parallel guard 30s: %llu reads (rays, overlap queries), %llu despawn waits (%.1f us avg, %.1f us max)",
              (unsigned long long)g_reads.exchange(0), (unsigned long long)w, w ? c / cyc_per_us / w : 0.0, m / cyc_per_us);
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
        swlog("contact rings 30s: logic drains %llu, waited %.1f us avg, %.1f us max; callbacks %llu, waited %.1f us avg, %.1f us max",
              (unsigned long long)dn, dn ? dw / cyc_per_us / dn : 0.0, dm / cyc_per_us, (unsigned long long)cn,
              cn ? cw / cyc_per_us / cn : 0.0, cm / cyc_per_us);
    }
}

bool install()
{
    g_on = GetPrivateProfileIntW(L"parallel", L"early_launch", 0, settings_ini()) == 1;
    if (!g_on) return true;
    g_cyl_ret = game_live(kCylinderOwnerFreeRet);
    g_zone_begin = game_ptr<Four_t>(kZoneBegin);
    g_zone_end = game_ptr<Four_t>(kZoneEnd);
    uint8_t* drain_open = find_zone_call(reinterpret_cast<uintptr_t>(g_zone_begin), kContactDrainZone);
    uint8_t* drain_close = find_zone_call(reinterpret_cast<uintptr_t>(g_zone_end), kContactDrainZone);
    uint8_t* stubs = drain_open && drain_close ? alloc_near(reinterpret_cast<uintptr_t>(drain_open), 4096) : nullptr;
    if (!stubs) {
        g_on = false;
        swlog("parallel guard: the logic's contact ring drain (zone 0x%x) wasn't found as expected: the early launch stays off",
              kContactDrainZone);
        return false;
    }
    bool ok = g_cyl_ret &&
              hook_game(kRayTest, reinterpret_cast<void*>(&ray_detour), reinterpret_cast<void**>(&g_ray)) &&
              hook_game(kCollide, reinterpret_cast<void*>(&collide_detour), reinterpret_cast<void**>(&g_collide)) &&
              hook_game(kSetAabb, reinterpret_cast<void*>(&set_aabb_detour), reinterpret_cast<void**>(&g_set_aabb)) &&
              hook_game(kIntegrateBodies, reinterpret_cast<void*>(&integrate_detour), reinterpret_cast<void**>(&g_integrate)) &&
              hook_game(kContactCallback, reinterpret_cast<void*>(&callback_detour), reinterpret_cast<void**>(&g_callback)) &&
              hook_game(kCylinderOwnerFree, reinterpret_cast<void*>(&cyl_free_detour), reinterpret_cast<void**>(&g_cyl_free));
    for (int i = 0; ok && i < kHelpers; i++)
        ok = hook_game(kDestroyHelpers[i], helper_detours()[i], reinterpret_cast<void**>(&g_helper[i]));
    ok = ok && point_call_at(drain_open, stubs, reinterpret_cast<void*>(&drain_begin)) &&
         point_call_at(drain_close, stubs + 32, reinterpret_cast<void*>(&drain_end));
    if (!ok) {
        // early_launch.cpp checks parallel_on() at its own install: without the guard it must not run
        g_on = false;
        swlog("parallel guard: a hook FAILED: the early launch stays off");
        return false;
    }
    CloseHandle(CreateThread(nullptr, 0, log_loop, nullptr, 0, nullptr));
    swlog("parallel guard: on (world lock (writer gives way to waiting readers): rays shared, setAabb per call / collide / the integrate loop exclusive; "
          "despawn guard on %d destroy helpers + the cylinder; contact rings locked (drain calls %p / %p); \"parallel guard 30s\")",
          kHelpers, drain_open, drain_close);
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
