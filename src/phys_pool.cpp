// Shared worker pool for the physics thread's parallel phases.
//
// Broadphase searches (broadphase_mt), collision pairs (mt_dispatch) and the solver (solver_log) each had their own
// pool of 5 threads sleeping on a condition variable: 9 cold wake-ups per tick (3 phases x 3 sub-steps), each tens of
// microseconds before the first worker starts, while 15 threads competed for cores with the update thread's vehicle and
// render-prep pools. Now one pool runs all three. After a phase its workers keep spinning for [physics_pool] spin_us
// (the next phase of the same sub-step usually starts within that), then sleep.
//
// Jobs are published seqlock style: the caller writes fn / ctx / helpers, then bumps the generation; a worker reads the
// generation, the job, and the generation again. Only workers with slot <= helpers run it, and the caller waits for
// exactly those, so a job can't be overwritten while a participant still needs it.

#include "phys_pool.h"
#include "join_wait.h"

#include "log.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

namespace {

int g_threads = 1; // including the physics thread
uint64_t g_spin_cycles = 0;

std::atomic<uint64_t> g_gen{0}, g_job_gen{0}; // g_job_gen: generation the job fields belong to (0 while written)
std::atomic<void (*)(int, void*)> g_fn{nullptr};
std::atomic<void*> g_ctx{nullptr};
std::atomic<int> g_helpers{0}, g_done{0}, g_sleepers{0};
std::mutex g_mu;
std::condition_variable g_cv;

void worker_main(int slot)
{
    world_mark_reader_thread(); // the solver's units call body_group()
    uint64_t seen = 0;
    for (;;) {
        uint64_t deadline = __rdtsc() + g_spin_cycles;
        while (g_gen.load(std::memory_order_acquire) == seen) {
            if (__rdtsc() < deadline) {
                _mm_pause();
                continue;
            }
            std::unique_lock<std::mutex> lock(g_mu);
            g_sleepers.fetch_add(1);
            g_cv.wait(lock, [&] { return g_gen.load() != seen; });
            g_sleepers.fetch_sub(1);
        }
        uint64_t g;
        void (*fn)(int, void*);
        void* ctx;
        int helpers;
        for (;;) {
            g = g_gen.load(std::memory_order_acquire);
            uint64_t j0 = g_job_gen.load(std::memory_order_acquire);
            fn = g_fn.load(std::memory_order_acquire);
            ctx = g_ctx.load(std::memory_order_acquire);
            helpers = g_helpers.load(std::memory_order_acquire);
            uint64_t j1 = g_job_gen.load(std::memory_order_acquire);
            if (j0 == g && j1 == g && g_gen.load(std::memory_order_acquire) == g) break;
            _mm_pause(); // the next job is being written
        }
        seen = g;
        if (slot <= helpers && fn) {
            fn(slot, ctx);
            g_done.fetch_add(1, std::memory_order_acq_rel);
        }
    }
}

std::once_flag g_started;

void start()
{
    const wchar_t* ini = settings_ini();
    int threads = static_cast<int>(GetPrivateProfileIntW(L"physics_pool", L"threads", 0, ini));
    int hw = static_cast<int>(std::thread::hardware_concurrency());
    if (threads <= 0) threads = std::min(hw - 2, 6);
    g_threads = std::max(1, std::min(threads, 16));
    int spin_us = static_cast<int>(GetPrivateProfileIntW(L"physics_pool", L"spin_us", 200, ini));
    LARGE_INTEGER f, q0, q1;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&q0);
    uint64_t t0 = __rdtsc();
    Sleep(10);
    QueryPerformanceCounter(&q1);
    double cyc_per_us = double(__rdtsc() - t0) / (double(q1.QuadPart - q0.QuadPart) * 1e6 / double(f.QuadPart));
    g_spin_cycles = static_cast<uint64_t>(std::max(0, spin_us) * cyc_per_us);
    for (int s = 1; s < g_threads; s++) std::thread(worker_main, s).detach();
    swlog("physics pool: %d threads (physics thread + %d workers) shared by broadphase, collision pairs and solver; spin %d us", g_threads,
          g_threads - 1, spin_us);
}

} // namespace

int phys_pool_threads()
{
    std::call_once(g_started, start);
    return g_threads;
}

std::atomic<bool> g_busy{false};

void phys_pool_run(int helpers, void (*fn)(int slot, void* ctx), void* ctx)
{
    helpers = std::max(0, std::min(helpers, phys_pool_threads() - 1));
    if (helpers == 0) return fn(0, ctx);
    bool expected = false;
    if (!g_busy.compare_exchange_strong(expected, true)) return fn(0, ctx); // another thread has the pool: work alone
    uint64_t next = g_gen.load(std::memory_order_relaxed) + 1;
    g_job_gen.store(0); // seq_cst: a worker still reading the previous job sees the change and retries
    g_fn.store(fn, std::memory_order_release);
    g_ctx.store(ctx, std::memory_order_release);
    g_helpers.store(helpers, std::memory_order_release);
    g_done.store(0, std::memory_order_relaxed);
    g_job_gen.store(next);
    g_gen.store(next); // seq_cst: ordered before the sleepers check below
    if (g_sleepers.load() > 0) {
        { std::lock_guard<std::mutex> lock(g_mu); }
        g_cv.notify_all();
    }
    fn(0, ctx);
    wait_for_workers("physics pool", [helpers] { return g_done.load(std::memory_order_acquire) >= helpers; });
    g_busy.store(false, std::memory_order_release);
}
