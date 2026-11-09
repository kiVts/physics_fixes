
#include "hull_rays.h"

#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace {

constexpr uintptr_t kRays = 0x1402f4950;
using Rays_t = void(__fastcall*)(uint8_t* sampler, void* ocean, const double* box);
Rays_t g_orig = nullptr;

struct Job {
    uint8_t* sampler;
    void* ocean;
    double box[6]; // the caller's 48-byte stack copy
};
std::vector<Job> g_jobs;            // filled by the update thread between joins
std::atomic<int> g_queued{0};       // published job count
std::atomic<int> g_next{0}, g_done{0};
std::atomic<bool> g_loop_join{false}, g_task_join{false}, g_on{false};
std::mutex g_mx;
std::condition_variable g_cv;
uint64_t g_gen = 0;
DWORD g_update_tid = 0;
std::atomic<uint64_t> g_casts{0}, g_deferred{0}, g_cycles{0}, g_join_wait{0};

void run_some()
{
    for (;;) {
        // claim the next job without ever overshooting: the update thread resets the counters between batches
        int i = g_next.load(std::memory_order_acquire);
        if (i >= g_queued.load(std::memory_order_acquire)) return;
        if (!g_next.compare_exchange_weak(i, i + 1, std::memory_order_acq_rel)) continue;
        uint64_t t0 = __rdtsc();
        Job& j = g_jobs[i];
        g_orig(j.sampler, j.ocean, j.box);
        g_cycles.fetch_add(__rdtsc() - t0, std::memory_order_relaxed);
        g_done.fetch_add(1, std::memory_order_acq_rel);
    }
}

void worker()
{
    uint64_t seen = 0;
    for (;;) {
        {
            std::unique_lock<std::mutex> lk(g_mx);
            g_cv.wait(lk, [&] { return g_gen != seen; });
            seen = g_gen;
        }
        run_some();
    }
}

void __fastcall rays_detour(uint8_t* sampler, void* ocean, const double* box)
{
    g_casts.fetch_add(1, std::memory_order_relaxed);
    if (!g_on || !g_loop_join || !g_task_join || GetCurrentThreadId() != g_update_tid || !box) {
        g_orig(sampler, ocean, box);
        return;
    }
    // g_jobs only grows between joins; the pool reads entries below g_queued, published after the write
    int n = g_queued.load(std::memory_order_relaxed);
    if (n == static_cast<int>(g_jobs.capacity())) hull_rays_join(); // never reallocate under running workers
    n = g_queued.load(std::memory_order_relaxed);
    g_jobs.resize(n + 1);
    Job& j = g_jobs[n];
    j.sampler = sampler;
    j.ocean = ocean;
    memcpy(j.box, box, sizeof j.box);
    g_queued.store(n + 1, std::memory_order_release);
    g_deferred.fetch_add(1, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(g_mx);
        g_gen++;
    }
    g_cv.notify_all();
}

DWORD WINAPI log_loop(void*)
{
    uint64_t lc = 0, ld = 0, lcy = 0, lw = 0;
    LARGE_INTEGER f, a, b;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&a);
    uint64_t r0 = __rdtsc();
    Sleep(50);
    QueryPerformanceCounter(&b);
    double cyc_per_ms = double(__rdtsc() - r0) / (double(b.QuadPart - a.QuadPart) * 1000.0 / double(f.QuadPart));
    for (;;) {
        Sleep(5000);
        uint64_t c = g_casts, d = g_deferred, cy = g_cycles, w = g_join_wait;
        if (c == lc) continue;
        swlog("hull rays 5s: %llu casts, %.0f%% on the pool (%.2f ms/s of casting there), joins waited %.3f ms/s in total",
              static_cast<unsigned long long>(c - lc), c > lc ? 100.0 * double(d - ld) / double(c - lc) : 0.0,
              double(cy - lcy) / cyc_per_ms / 5.0, double(w - lw) / cyc_per_ms / 5.0);
        lc = c, ld = d, lcy = cy, lw = w;
    }
}

bool install()
{
    std::wstring ini = settings_ini();
    if (!GetPrivateProfileIntW(L"hull_rays", L"defer", 1, ini.c_str())) {
        swlog("hull rays: off in settings.ini");
        return true;
    }
    int threads = static_cast<int>(GetPrivateProfileIntW(L"hull_rays", L"threads", 0, ini.c_str()));
    if (threads <= 0) threads = 3;
    if (!swml()->on_loop(swml_self(), [](void*) {
            // every iteration, not just the first: the first loop event can come from another thread (2026-10-28
            // 21:21, a slow start under memory pressure: the casts then never left the update thread)
            DWORD tid = GetCurrentThreadId();
            if (g_update_tid != tid) {
                hull_rays_join(); // anything queued under the old id
                g_update_tid = tid;
            }
            hull_rays_join(); // backstop: nothing deferred crosses a tick
        }, nullptr, nullptr, nullptr)) {
        swlog("hull rays: no game loop events; off");
        return true;
    }
    g_jobs.reserve(4096);
    if (!hook_game(kRays, reinterpret_cast<void*>(&rays_detour), reinterpret_cast<void**>(&g_orig))) return false;
    for (int i = 0; i < threads; i++) std::thread(worker).detach();
    g_on = true;
    CloseHandle(CreateThread(nullptr, 0, log_loop, nullptr, 0, nullptr));
    swlog("hull rays: on (water rays of the hull setup on %d threads, joined before hull tasks and the vehicle loop; "
          "\"hull rays 5s\" lines)", threads);
    return true;
}

HookRegistrar reg{{"hull rays", /*needs_known_build=*/true, &install}};

} // namespace

void hull_rays_join()
{
    if (g_done.load(std::memory_order_acquire) == g_queued.load(std::memory_order_acquire)) {
        // all done: the update thread starts the next batch from empty
        if (GetCurrentThreadId() == g_update_tid && g_queued.load(std::memory_order_relaxed)) {
            g_queued.store(0, std::memory_order_release); // first: a worker comparing next with queued then stops
            g_next.store(0, std::memory_order_release);
            g_done.store(0, std::memory_order_release);
            g_jobs.clear();
        }
        return;
    }
    uint64_t t0 = __rdtsc();
    run_some(); // help
    while (g_done.load(std::memory_order_acquire) < g_queued.load(std::memory_order_acquire)) YieldProcessor();
    g_join_wait.fetch_add(__rdtsc() - t0, std::memory_order_relaxed);
    if (GetCurrentThreadId() == g_update_tid) {
        g_queued.store(0, std::memory_order_release);
        g_next.store(0, std::memory_order_release);
        g_done.store(0, std::memory_order_release);
        g_jobs.clear();
    }
}

void hull_rays_loop_join_ready() { g_loop_join = true; }
void hull_rays_task_join_ready() { g_task_join = true; }
