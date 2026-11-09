
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <vector>

namespace {

constexpr uintptr_t kHullSupport = 0x1400ff190; // (out, dir, vertices, count, scaling) -> out

using Support_t = double*(__fastcall*)(double* out, const double* dir, const double* pts, int n, const double* scaling);
Support_t g_orig = nullptr;

constexpr int kBuckets = 8; // vertices: <=4, <=8, <=16, <=32, <=64, <=128, <=256, more
const char* const kBucketNames[kBuckets] = {"<=4", "5-8", "9-16", "17-32", "33-64", "65-128", "129-256", ">256"};
constexpr int kHullSlots = 8192;

struct Stats {
    uint64_t calls = 0, verts = 0, cycles = 0, bucket[kBuckets] = {};
    uint64_t bucket_cycles[kBuckets] = {};
    // distinct vertex arrays this window (open addressing; window id per slot, so no clearing)
    const double* hull[kHullSlots] = {};
    uint32_t hull_win[kHullSlots] = {};
    uint64_t hulls = 0, hull_full = 0;
};
thread_local Stats* tl = nullptr;
std::mutex g_mu;
std::vector<Stats*> g_all;
volatile uint32_t g_win = 1;

Stats& stats()
{
    if (!tl) {
        tl = new Stats();
        std::lock_guard<std::mutex> lock(g_mu);
        g_all.push_back(tl);
    }
    return *tl;
}

int bucket_of(int n)
{
    int b = 0;
    for (int lim = 4; b < kBuckets - 1 && n > lim; lim *= 2) b++;
    return b;
}

double* __fastcall detour(double* out, const double* dir, const double* pts, int n, const double* sc)
{
    uint64_t t0 = __rdtsc();
    double* r = g_orig(out, dir, pts, n, sc);
    uint64_t c = __rdtsc() - t0;
    Stats& s = stats();
    int b = bucket_of(n);
    s.calls++;
    s.verts += n > 0 ? uint64_t(n) : 0;
    s.cycles += c;
    s.bucket[b]++;
    s.bucket_cycles[b] += c;
    uint32_t win = g_win;
    size_t h = (reinterpret_cast<uintptr_t>(pts) >> 4) * 0x9E3779B97F4A7C15ull >> 51; // 13 bits
    for (int i = 0; i < 8; i++) {
        size_t k = (h + i) & (kHullSlots - 1);
        if (s.hull_win[k] == win && s.hull[k] == pts) break;
        if (s.hull_win[k] != win) {
            s.hull_win[k] = win;
            s.hull[k] = pts;
            s.hulls++;
            break;
        }
        if (i == 7) s.hull_full++;
    }
    return r;
}

DWORD WINAPI log_loop(void*)
{
    struct Sum {
        uint64_t calls, verts, cycles, bucket[kBuckets], bucket_cycles[kBuckets], hulls, hull_full;
    } last{};
    for (;;) {
        Sleep(5000);
        Sum now{};
        size_t threads;
        {
            std::lock_guard<std::mutex> lock(g_mu);
            threads = g_all.size();
            for (Stats* s : g_all) {
                now.calls += s->calls, now.verts += s->verts, now.cycles += s->cycles, now.hulls += s->hulls, now.hull_full += s->hull_full;
                for (int b = 0; b < kBuckets; b++) now.bucket[b] += s->bucket[b], now.bucket_cycles[b] += s->bucket_cycles[b];
            }
        }
        g_win = g_win + 1;
        uint64_t calls = now.calls - last.calls;
        if (calls) {
            char hist[512];
            int len = 0;
            for (int b = 0; b < kBuckets; b++) {
                uint64_t n = now.bucket[b] - last.bucket[b];
                if (!n) continue;
                len += snprintf(hist + len, sizeof hist - len, " %s:%.1f%% (%.0f cyc)", kBucketNames[b], 100.0 * double(n) / double(calls),
                                double(now.bucket_cycles[b] - last.bucket_cycles[b]) / double(n));
                if (len >= int(sizeof hist)) break;
            }
            swlog("hull support 5s: %.0f calls/s, %.1f vertices per call, %.0f cycles per call (%.2f per vertex), "
                  "%llu distinct hulls seen (summed over %zu threads%s); vertices per call:%s",
                  double(calls) / 5.0, double(now.verts - last.verts) / double(calls), double(now.cycles - last.cycles) / double(calls),
                  double(now.cycles - last.cycles) / double(std::max<uint64_t>(1, now.verts - last.verts)),
                  (unsigned long long)(now.hulls - last.hulls), threads, now.hull_full != last.hull_full ? ", table full" : "", hist);
        }
        last = now;
    }
}

bool install()
{
    if (!GetPrivateProfileIntW(L"bullet", L"hull_count", 0, settings_ini())) return true;
    if (!hook_game(kHullSupport, reinterpret_cast<void*>(&detour), reinterpret_cast<void**>(&g_orig))) return false;
    CloseHandle(CreateThread(nullptr, 0, log_loop, nullptr, 0, nullptr));
    swlog("hull support counter: on (\"hull support 5s\" lines)");
    return true;
}

HookRegistrar reg{{"hull support counter", /*needs_known_build=*/true, &install}};

} // namespace
