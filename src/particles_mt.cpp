
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace {

constexpr uintptr_t kParticles = 0x1402fd8e0, kResize = 0x1408de0b0, kFree = 0x1409b2d50;
constexpr int kMaxSlices = 16;

using Particles_t = void(__fastcall*)(uint8_t* ring);
using Resize_t = void(__fastcall*)(void* ring, uint32_t cap);
using Free_t = void(__fastcall*)(void*);
Particles_t o_particles = nullptr;
Resize_t g_resize = nullptr;
Free_t g_free = nullptr;

int g_threads = 4, g_min = 64;

struct Ring {
    uint8_t** data;
    uint32_t cap, head, count, pad;
    uint8_t rest[8];
};
static_assert(sizeof(Ring) == 0x20, "ring header");

struct Slice {
    alignas(64) Ring ring;
    uint32_t first, n;
};
Slice g_slices[kMaxSlices];
int g_nslices = 0;

// --- pool --------------------------------------------------------------------------------------------------------
std::mutex g_m;
std::condition_variable g_go, g_done;
uint64_t g_gen = 0;
std::atomic<int> g_next{0};
int g_running = 0, g_started = 0;

void run_slices()
{
    for (;;) {
        int i = g_next.fetch_add(1, std::memory_order_relaxed);
        if (i >= g_nslices) return;
        o_particles(reinterpret_cast<uint8_t*>(&g_slices[i].ring));
    }
}

void worker(int index)
{
    using SetDesc_t = HRESULT(WINAPI*)(HANDLE, PCWSTR);
    if (auto set = reinterpret_cast<SetDesc_t>(GetProcAddress(GetModuleHandleW(L"kernel32.dll"), "SetThreadDescription"))) {
        wchar_t name[32];
        swprintf(name, 32, L"particles_%d", index);
        set(GetCurrentThread(), name);
    }
    uint64_t seen = 0;
    for (;;) {
        {
            std::unique_lock<std::mutex> l(g_m);
            g_go.wait(l, [&] { return g_gen != seen; });
            seen = g_gen;
        }
        run_slices();
        std::lock_guard<std::mutex> l(g_m);
        if (--g_running == 0) g_done.notify_one();
    }
}

// stats (update thread only)
uint64_t g_calls = 0, g_split = 0, g_emitters = 0, g_removed = 0, g_cycles = 0;
ULONGLONG g_window = 0;
LARGE_INTEGER g_freq{};

void maybe_log()
{
    ULONGLONG now = GetTickCount64();
    if (!g_window) g_window = now;
    if (now - g_window < 5000) return;
    double secs = (now - g_window) / 1000.0;
    if (g_calls)
        swlog("particles 5s: %.0f updates/s (%.0f%% split over %d threads), %.0f emitters per update, %.1f removed, %.3f ms per update",
              g_calls / secs, 100.0 * g_split / g_calls, g_threads, double(g_emitters) / g_calls, double(g_removed) / g_calls,
              double(g_cycles) * 1000.0 / double(g_freq.QuadPart) / g_calls);
    g_calls = g_split = g_emitters = g_removed = g_cycles = 0;
    g_window = now;
}

void __fastcall particles_detour(uint8_t* ring_p)
{
    LARGE_INTEGER t0, t1;
    QueryPerformanceCounter(&t0);
    Ring& real = *reinterpret_cast<Ring*>(ring_p);
    uint32_t n = real.count;
    g_calls++;
    g_emitters += n;
    if (n < uint32_t(g_min) || !real.data || !real.cap) {
        o_particles(ring_p);
    } else {
        g_split++;
        int slices = std::min<int>({g_threads + 1, kMaxSlices, int(n / 16)});
        g_nslices = slices;
        for (int s = 0; s < slices; s++) {
            Slice& sl = g_slices[s];
            sl.first = uint32_t(uint64_t(n) * s / slices);
            sl.n = uint32_t(uint64_t(n) * (s + 1) / slices) - sl.first;
            memcpy(&sl.ring, &real, sizeof(Ring));
            sl.ring.data = nullptr, sl.ring.cap = 0, sl.ring.head = 0, sl.ring.count = 0;
            g_resize(&sl.ring, std::max(sl.n, 1u)); // the game's allocation: its frees and shrinks apply to it
            for (uint32_t i = 0; i < sl.n; i++) sl.ring.data[i] = real.data[(real.head + sl.first + i) % real.cap];
            sl.ring.count = sl.n;
        }
        g_next.store(0, std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> l(g_m);
            while (g_started < g_threads) std::thread(worker, g_started++).detach();
            g_running = g_threads;
            g_gen++;
        }
        g_go.notify_all();
        run_slices();
        {
            std::unique_lock<std::mutex> l(g_m);
            g_done.wait(l, [] { return g_running == 0; });
        }
        // survivors, in order, back into the real ring
        uint32_t k = 0;
        for (int s = 0; s < slices; s++) {
            Ring& r = g_slices[s].ring;
            for (uint32_t i = 0; i < r.count; i++) real.data[(real.head + k++) % real.cap] = r.data[(r.head + i) % r.cap];
            if (r.cap) g_free(r.data);
            r.data = nullptr, r.cap = 0;
        }
        g_removed += n - k;
        real.count = k;
        if (k == 0) { // as the game does for an empty ring; head to 0 too (see fast_spawn 1.7.1's ring note)
            g_free(real.data);
            real.data = nullptr, real.cap = 0, real.head = 0;
        }
    }
    QueryPerformanceCounter(&t1);
    g_cycles += uint64_t(t1.QuadPart - t0.QuadPart);
    maybe_log();
}

bool install()
{
    const wchar_t* ini = settings_ini();
    if (!GetPrivateProfileIntW(L"particles", L"parallel", 1, ini)) {
        swlog("particles: parallel=0, the game's loop");
        return true;
    }
    int threads = static_cast<int>(GetPrivateProfileIntW(L"particles", L"threads", 0, ini));
    g_threads = std::clamp(threads > 0 ? threads : 4, 1, kMaxSlices - 1);
    g_min = std::max(16, static_cast<int>(GetPrivateProfileIntW(L"particles", L"min_emitters", 64, ini)));
    QueryPerformanceFrequency(&g_freq);
    g_resize = game_ptr<Resize_t>(kResize);
    g_free = game_ptr<Free_t>(kFree);
    if (!hook_game(kParticles, reinterpret_cast<void*>(&particles_detour), reinterpret_cast<void**>(&o_particles))) return false;
    swlog("particles: world particle emitters updated in slices on %d threads + the update thread (min %d emitters; \"particles 5s\" lines)",
          g_threads, g_min);
    return true;
}

HookRegistrar reg{{"particles mt", /*needs_known_build=*/true, &install}};

} // namespace
