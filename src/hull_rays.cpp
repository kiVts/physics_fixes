
#include "hull_rays.h"
#include "join_wait.h"
#include "mt_settings.h"

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
std::mutex g_jobs_mu; // claim/copy and batch reset must be one transaction
std::atomic<int> g_queued{0};       // published job count
std::atomic<int> g_next{0}, g_done{0};
std::atomic<bool> g_loop_join{false}, g_task_join{false}, g_on{false};
std::mutex g_mx;
std::condition_variable g_cv;
uint64_t g_gen = 0;
DWORD g_update_tid = 0;
std::atomic<uint64_t> g_casts{0}, g_deferred{0}, g_cycles{0}, g_join_wait{0};
// joins that found casts outstanding, by site x (0 pool thread, 1 update thread): count, cycles in the join, cycles of
// that spent casting (the joiner helps) rather than waiting
constexpr int kSites = static_cast<int>(HullJoin::count);
std::atomic<uint64_t> g_site_n[kSites][2], g_site_cyc[kSites][2], g_site_help[kSites][2];

void run_some()
{
    for (;;) {
        Job j;
        {
            std::lock_guard lock(g_jobs_mu);
            int i = g_next.load(std::memory_order_relaxed);
            if (i >= g_queued.load(std::memory_order_relaxed)) return;
            g_next.store(i + 1, std::memory_order_relaxed);
            j = g_jobs[i]; // never retain a reference across a clear/resize
        }
        uint64_t t0 = __rdtsc();
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
    if (n == static_cast<int>(g_jobs.capacity())) hull_rays_join(HullJoin::full); // never reallocate under running workers
    n = g_queued.load(std::memory_order_relaxed);
    {
        std::lock_guard lock(g_jobs_mu);
        g_jobs.resize(n + 1);
        Job& j = g_jobs[n];
        j.sampler = sampler;
        j.ocean = ocean;
        memcpy(j.box, box, sizeof j.box);
        g_queued.store(n + 1, std::memory_order_release);
    }
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
    uint64_t ln[kSites][2] = {}, lsc[kSites][2] = {}, lsh[kSites][2] = {};
    LARGE_INTEGER f, a, b;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&a);
    uint64_t r0 = __rdtsc();
    Sleep(50);
    QueryPerformanceCounter(&b);
    double cyc_per_ms = double(__rdtsc() - r0) / (double(b.QuadPart - a.QuadPart) * 1000.0 / double(f.QuadPart));
    static const char* const kSiteName[kSites] = {"vehicle loop", "hull tasks", "tick start", "queue full"};
    for (;;) {
        Sleep(5000);
        uint64_t c = g_casts, d = g_deferred, cy = g_cycles, w = g_join_wait;
        if (c == lc) continue;
        swlog("hull rays 5s: %llu casts, %.0f%% on the pool (%.2f ms/s of casting there), joins waited %.3f ms/s in total",
              static_cast<unsigned long long>(c - lc), c > lc ? 100.0 * double(d - ld) / double(c - lc) : 0.0,
              double(cy - lcy) / cyc_per_ms / 5.0, double(w - lw) / cyc_per_ms / 5.0);
        lc = c, ld = d, lcy = cy, lw = w;
        // by site: ms/s in the join (of it casting), and how many joins found casts outstanding
        char text[768] = {};
        size_t at = 0;
        auto put = [&](const char* fmt, auto... args) {
            if (at + 1 >= sizeof text) return;
            const int r = snprintf(text + at, sizeof text - at, fmt, args...);
            if (r > 0) at = std::min(sizeof text - 1, at + size_t(r));
        };
        for (int t = 1; t >= 0; --t) {
            put("%s%s:", t ? "" : "; ", t ? "update thread" : "pool threads");
            bool any = false;
            for (int s = 0; s < kSites; ++s) {
                const uint64_t n = g_site_n[s][t].load(std::memory_order_relaxed), sc = g_site_cyc[s][t].load(std::memory_order_relaxed),
                               sh = g_site_help[s][t].load(std::memory_order_relaxed);
                if (n != ln[s][t]) {
                    put(" %s %.3f ms/s (casting %.3f) in %llu", kSiteName[s], double(sc - lsc[s][t]) / cyc_per_ms / 5.0,
                        double(sh - lsh[s][t]) / cyc_per_ms / 5.0, static_cast<unsigned long long>(n - ln[s][t]));
                    any = true;
                }
                ln[s][t] = n, lsc[s][t] = sc, lsh[s][t] = sh;
            }
            if (!any) put(" none");
        }
        swlog("hull rays joins 5s: %s", text);
    }
}

bool install()
{
    std::wstring ini = settings_ini();
    if (!hull_rays_enabled()) {
        swlog("hull rays: off ([hull_rays] defer=0 or multithreaded physics off)");
        return true;
    }
    int threads = static_cast<int>(GetPrivateProfileIntW(L"hull_rays", L"threads", 0, ini.c_str()));
    if (threads <= 0) threads = 3;
    threads = std::clamp(threads, 1, 16);
    if (!swml()->on_loop(swml_self(), [](void*) {
            DWORD tid = GetCurrentThreadId();
            if (g_update_tid != tid) {
                hull_rays_join(HullJoin::tick); // anything queued under the old id
                g_update_tid = tid;
            }
            hull_rays_join(HullJoin::tick); // backstop: nothing deferred crosses a tick
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

bool hull_rays_enabled()
{
    return mt_area(L"physics") && GetPrivateProfileIntW(L"hull_rays", L"defer", 1, settings_ini()) != 0;
}

void hull_rays_join(HullJoin site)
{
    if (g_done.load(std::memory_order_acquire) == g_queued.load(std::memory_order_acquire)) {
        // all done: the update thread starts the next batch from empty
        if (GetCurrentThreadId() == g_update_tid && g_queued.load(std::memory_order_relaxed)) {
            std::lock_guard lock(g_jobs_mu);
            g_queued.store(0, std::memory_order_release); // first: a worker comparing next with queued then stops
            g_next.store(0, std::memory_order_release);
            g_done.store(0, std::memory_order_release);
            g_jobs.clear();
        }
        return;
    }
    uint64_t t0 = __rdtsc();
    run_some(); // help
    const uint64_t t1 = __rdtsc();
    wait_for_workers("hull rays", [] { return g_done.load(std::memory_order_acquire) >= g_queued.load(std::memory_order_acquire); });
    const uint64_t t2 = __rdtsc();
    g_join_wait.fetch_add(t2 - t0, std::memory_order_relaxed);
    const bool update = GetCurrentThreadId() == g_update_tid;
    const int s = std::clamp(static_cast<int>(site), 0, kSites - 1);
    g_site_n[s][update].fetch_add(1, std::memory_order_relaxed);
    g_site_cyc[s][update].fetch_add(t2 - t0, std::memory_order_relaxed);
    g_site_help[s][update].fetch_add(t1 - t0, std::memory_order_relaxed);
    if (update) {
        std::lock_guard lock(g_jobs_mu);
        g_queued.store(0, std::memory_order_release);
        g_next.store(0, std::memory_order_release);
        g_done.store(0, std::memory_order_release);
        g_jobs.clear();
    }
}

void hull_rays_loop_join_ready() { g_loop_join = true; }
void hull_rays_task_join_ready() { g_task_join = true; }
