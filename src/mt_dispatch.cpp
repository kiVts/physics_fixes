
#include "mt_dispatch.h"
#include "phys_pool.h"

#include "hooks.h"
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
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uintptr_t kDispatchAll = 0x1400d8450, kGetNewManifold = 0x1400d7f40, kReleaseManifold = 0x1400d81e0,
                    kAllocAlgorithm = 0x1400d8630, kFreeAlgorithm = 0x1400d8660;
constexpr size_t kDispatcherNearCallback = 0x30;               // btCollisionDispatcher::m_nearCallback
constexpr size_t kPairCacheCount = 0xc, kPairCacheData = 0x18; // btHashedOverlappingPairCache::m_overlappingPairArray
constexpr size_t kPairSize = 0x20;                              // btBroadphasePair
constexpr size_t kObjInternalType = 0x1f0;
constexpr int kCoSoftBody = 8;

using DispatchAll_t = void(__fastcall*)(void* dispatcher, void* pair_cache, const void* info, void* dispatcher2);
using Near_t = void(__fastcall*)(void* pair, void* dispatcher, const void* info);
using GetNewManifold_t = void*(__fastcall*)(void* dispatcher, const void* b0, const void* b1);
using ReleaseManifold_t = void(__fastcall*)(void* dispatcher, void* manifold);
using AllocAlgorithm_t = void*(__fastcall*)(void* dispatcher, int size);
using FreeAlgorithm_t = void(__fastcall*)(void* dispatcher, void* ptr);
DispatchAll_t g_dispatch_all = nullptr;
GetNewManifold_t g_get_new_manifold = nullptr;
ReleaseManifold_t g_release_manifold = nullptr;
AllocAlgorithm_t g_alloc_algorithm = nullptr;
FreeAlgorithm_t g_free_algorithm = nullptr;

std::atomic<bool> g_parallel{false}; // a parallel phase is running
thread_local bool tl_worker = false;  // this thread is processing pairs of a parallel phase
SRWLOCK g_pool_lock = SRWLOCK_INIT;
int g_threads = 0, g_min_pairs = 256;

// --- locked dispatcher pools ---------------------------------------------------------------------------
struct PoolGuard {
    bool locked;
    PoolGuard() : locked(g_parallel.load(std::memory_order_acquire))
    {
        if (locked) AcquireSRWLockExclusive(&g_pool_lock);
    }
    ~PoolGuard()
    {
        if (locked) ReleaseSRWLockExclusive(&g_pool_lock);
    }
};
void* __fastcall get_new_manifold_detour(void* d, const void* b0, const void* b1)
{
    PoolGuard g;
    return g_get_new_manifold(d, b0, b1);
}
void __fastcall release_manifold_detour(void* d, void* m)
{
    PoolGuard g;
    g_release_manifold(d, m);
}
void* __fastcall alloc_algorithm_detour(void* d, int size)
{
    PoolGuard g;
    return g_alloc_algorithm(d, size);
}
void __fastcall free_algorithm_detour(void* d, void* p)
{
    PoolGuard g;
    g_free_algorithm(d, p);
}

// --- worker pool -----------------------------------------------------------------------------------------
struct Job {
    uint8_t* pairs = nullptr;
    const int* indices = nullptr; // pairs to run in parallel
    int count = 0;
    void* dispatcher = nullptr;
    const void* info = nullptr;
    Near_t near_cb = nullptr;
    std::atomic<int> next{0};
};
Job g_job;
constexpr int kChunk = 8;

void run_chunks()
{
    for (;;) {
        int start = g_job.next.fetch_add(kChunk, std::memory_order_relaxed);
        if (start >= g_job.count) return;
        int end = start + kChunk < g_job.count ? start + kChunk : g_job.count;
        for (int i = start; i < end; i++) g_job.near_cb(g_job.pairs + size_t(g_job.indices[i]) * kPairSize, g_job.dispatcher, g_job.info);
    }
}

std::vector<int> g_par, g_ser;

// Per dispatch: cycles each slot spent on pairs (the rest of the dispatch's wall time it waited or wasn't woken yet).
constexpr int kSlots = 17;
uint64_t g_busy[kSlots] = {}, g_busy_sum[kSlots] = {}, g_busy_max = 0, g_split_cycles = 0, g_job_cycles = 0;

void pairs_job(int slot, void*)
{
    tl_worker = true;
    uint64_t t0 = __rdtsc();
    if (slot == 0)
        for (int i : g_ser) g_job.near_cb(g_job.pairs + size_t(i) * kPairSize, g_job.dispatcher, g_job.info);
    run_chunks();
    if (slot >= 0 && slot < kSlots) g_busy[slot] = __rdtsc() - t0; // each slot writes its own entry
    tl_worker = false;
}

// --- stats ------------------------------------------------------------------------------------------------------
uint64_t g_dispatches = 0, g_parallel_dispatches = 0, g_pairs_total = 0, g_serial_pairs = 0, g_cycles = 0;
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
    double cyc_per_ms = double(__rdtsc() - g_window_tsc) / (secs * 1000.0);
    swlog("parallel 5s: %.0f dispatches/s (%.0f%% parallel on %d threads), %.0f pairs per dispatch, %.0f on the physics thread only"
          " (soft bodies), %.3f ms per dispatch, %.1f ms/s",
          g_dispatches / secs, g_dispatches ? 100.0 * g_parallel_dispatches / g_dispatches : 0.0, g_threads + 1,
          g_dispatches ? double(g_pairs_total) / g_dispatches : 0.0, g_dispatches ? double(g_serial_pairs) / g_dispatches : 0.0,
          g_dispatches ? g_cycles / cyc_per_ms / g_dispatches : 0.0, g_cycles / cyc_per_ms / secs);
    if (g_parallel_dispatches) {
        // Where a parallel dispatch's wall time goes: sorting out soft-body pairs (serial, before the workers start),
        // the pool run (wake, pairs, last thread finishing), and per thread the time actually on pairs.
        double pd = double(g_parallel_dispatches), busy = 0;
        for (int i = 0; i <= g_threads && i < kSlots; i++) busy += double(g_busy_sum[i]);
        swlog("parallel 5s: per parallel dispatch ms: split %.3f, pool run %.3f, on pairs per thread avg %.3f, busiest %.3f "
              "(threads busy %.0f%% of the pool run)",
              g_split_cycles / cyc_per_ms / pd, g_job_cycles / cyc_per_ms / pd, busy / (g_threads + 1) / cyc_per_ms / pd,
              g_busy_max / cyc_per_ms / pd, g_job_cycles ? 100.0 * busy / (g_threads + 1) / double(g_job_cycles) : 0.0);
    }
    for (auto& b : g_busy_sum) b = 0;
    g_busy_max = g_split_cycles = g_job_cycles = 0;
    g_dispatches = g_parallel_dispatches = g_pairs_total = g_serial_pairs = g_cycles = 0;
    g_window = now;
    g_window_tsc = __rdtsc();
}

void (*g_after_parallel[4])() = {};

bool read_pairs_raw(void* cache, int& n, uint8_t*& data)
{
    __try {
        n = *reinterpret_cast<const int*>(static_cast<uint8_t*>(cache) + kPairCacheCount);
        data = *reinterpret_cast<uint8_t* const*>(static_cast<uint8_t*>(cache) + kPairCacheData);
        return n >= 0 && n < 50000000 && (n == 0 || data);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool is_soft(const uint8_t* pair)
{
    const uint8_t* p0 = *reinterpret_cast<const uint8_t* const*>(pair);
    const uint8_t* p1 = *reinterpret_cast<const uint8_t* const*>(pair + 8);
    const uint8_t* o0 = *reinterpret_cast<const uint8_t* const*>(p0);
    const uint8_t* o1 = *reinterpret_cast<const uint8_t* const*>(p1);
    return *reinterpret_cast<const int*>(o0 + kObjInternalType) == kCoSoftBody ||
           *reinterpret_cast<const int*>(o1 + kObjInternalType) == kCoSoftBody;
}

bool split_raw(uint8_t* data, int n)
{
    __try {
        g_par.clear();
        g_ser.clear();
        for (int i = 0; i < n; i++) (is_soft(data + size_t(i) * kPairSize) ? g_ser : g_par).push_back(i);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

std::atomic<void*> g_last_pair_cache{nullptr};

void __fastcall dispatch_all_detour(void* dispatcher, void* pair_cache, const void* info, void* dispatcher2)
{
    if (GetCurrentThreadId() == world_physics_thread()) g_last_pair_cache.store(pair_cache, std::memory_order_relaxed);
    if (g_threads <= 0 || GetCurrentThreadId() != world_physics_thread() || g_parallel.load()) {
        g_dispatch_all(dispatcher, pair_cache, info, dispatcher2);
        return;
    }
    uint64_t t0 = __rdtsc();
    int n = 0;
    uint8_t* data = nullptr;
    Near_t near_cb = *reinterpret_cast<Near_t*>(static_cast<uint8_t*>(dispatcher) + kDispatcherNearCallback);
    g_dispatches++;
    bool split = near_cb && read_pairs_raw(pair_cache, n, data) && n >= g_min_pairs && split_raw(data, n);
    uint64_t t_split = __rdtsc();
    if (!split) {
        g_dispatch_all(dispatcher, pair_cache, info, dispatcher2);
        g_cycles += __rdtsc() - t0;
        maybe_log();
        return;
    }
    g_parallel_dispatches++;
    g_pairs_total += n;
    g_serial_pairs += g_ser.size();

    g_job.pairs = data;
    g_job.indices = g_par.data();
    g_job.count = static_cast<int>(g_par.size());
    g_job.dispatcher = dispatcher;
    g_job.info = info;
    g_job.near_cb = near_cb;
    g_job.next.store(0, std::memory_order_relaxed);
    for (auto& b : g_busy) b = 0;
    g_parallel.store(true, std::memory_order_release);
    uint64_t t_run = __rdtsc();
    phys_pool_run(g_threads, &pairs_job, nullptr);
    uint64_t t_done = __rdtsc();
    g_parallel.store(false, std::memory_order_release);
    g_split_cycles += t_split - t0, g_job_cycles += t_done - t_run;
    uint64_t most = 0;
    for (int i = 0; i <= g_threads && i < kSlots; i++) g_busy_sum[i] += g_busy[i], most = std::max(most, g_busy[i]);
    g_busy_max += most;
    for (auto fn : g_after_parallel)
        if (fn) fn();
    g_cycles += __rdtsc() - t0;
    maybe_log();
}

} // namespace

bool mt_dispatch_active() { return g_parallel.load(std::memory_order_relaxed); }
bool mt_is_worker() { return tl_worker; }
void* mt_last_pair_cache() { return g_last_pair_cache.load(std::memory_order_relaxed); }

void mt_set_after_parallel(void (*fn)())
{
    for (auto& slot : g_after_parallel)
        if (!slot) {
            slot = fn;
            return;
        }
}

namespace {

bool install()
{
    std::wstring ini = settings_ini();
    int threads = static_cast<int>(GetPrivateProfileIntW(L"parallel", L"threads", 0, ini.c_str()));
    g_min_pairs = static_cast<int>(GetPrivateProfileIntW(L"parallel", L"min_pairs", 256, ini.c_str()));
    int hw = static_cast<int>(std::thread::hardware_concurrency());
    if (threads <= 0) threads = hw - 2 < 6 ? hw - 2 : 6;
    g_threads = std::min(threads, phys_pool_threads()) - 1; // worker threads besides the physics thread
    if (g_threads <= 0) {
        g_threads = 0;
        swlog("parallel: off (threads=%d, %d logical processors)", threads, hw);
        return true;
    }
    if (!hook_game(kGetNewManifold, reinterpret_cast<void*>(&get_new_manifold_detour), reinterpret_cast<void**>(&g_get_new_manifold)) ||
        !hook_game(kReleaseManifold, reinterpret_cast<void*>(&release_manifold_detour), reinterpret_cast<void**>(&g_release_manifold)) ||
        !hook_game(kAllocAlgorithm, reinterpret_cast<void*>(&alloc_algorithm_detour), reinterpret_cast<void**>(&g_alloc_algorithm)) ||
        !hook_game(kFreeAlgorithm, reinterpret_cast<void*>(&free_algorithm_detour), reinterpret_cast<void**>(&g_free_algorithm)) ||
        !hook_game(kDispatchAll, reinterpret_cast<void*>(&dispatch_all_detour), reinterpret_cast<void**>(&g_dispatch_all)))
        return false;
    swlog("parallel: collision pairs on %d threads (%d logical processors), min %d pairs per dispatch (\"parallel 5s\" lines)",
          g_threads + 1, hw, g_min_pairs);
    return true;
}

HookRegistrar reg{{"parallel_narrowphase", /*needs_known_build=*/true, &install}};

} // namespace
