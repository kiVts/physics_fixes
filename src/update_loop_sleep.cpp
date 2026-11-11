// Game-logic loop: sleep instead of busy-waiting.
//
// The game's update thread runs one tick, then spins on its ms clock until 16 ms have passed since
// the iteration started, so one core always shows 100% even when the tick took 2 ms. On the core's
// game-loop "idle" event (right before that spin) this sleeps on a high-resolution timer until
// 14 ms after the start; the game then spins only the last 1-2 ms and checks its own condition as
// before, so tick timing is unchanged. If the tick already took longer, nothing happens.
//
// The game compares whole milliseconds, so the loop can end as early as 15.0 ms after the start;
// the wake time backs off automatically if the timer ever wakes that late.
// Settings: settings.ini [cpu] update_loop_sleep. Logs "loop 5s:" lines.

#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

namespace {

bool g_sleep = true;
bool g_owner_checked=false,g_legacy_owner=false;
bool legacy_owner()
{
    // Loop callbacks begin only after all mods have initialized. Checking here
    // works regardless of their load order, including older standalone DLLs.
    if(!g_owner_checked){
        g_owner_checked=true;
        g_legacy_owner=GetModuleHandleW(L"update_loop_sleep.dll")!=nullptr;
        if(g_legacy_owner)swlog("update loop: standalone Update Loop Sleep loaded; delegating sleep/timings to it. Disable it to use Physics Fixes' cpu settings.");
    }
    return g_legacy_owner;
}
LARGE_INTEGER g_freq{};
HANDLE g_timer = nullptr;
double g_wake_ms = 14.0;
constexpr double kLatest = 15.0;
constexpr double kWakeMax = 14.0, kWakeMin = 10.0;

// Update thread only.
uint64_t g_loops = 0;
int g_late = 0;
double g_tick_sum = 0, g_tick_max = 0, g_slept_sum = 0, g_window_ms = 0;
// Per-tick work times of the window (for percentiles) and the wall clock it covers, so "ticks/s" is real.
std::vector<float> g_ticks;
ULONGLONG g_window_start = 0;
int g_over16 = 0;

double ms_since(LONGLONG t0)
{
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return double(now.QuadPart - t0) * 1000.0 / double(g_freq.QuadPart);
}

void on_end(double tick, void*)
{
    if(legacy_owner())return;
    g_loops++;
    g_tick_sum += tick;
    g_tick_max = std::max(g_tick_max, tick);
    if (tick > 16.0) g_over16++;
    if (g_ticks.size() < 4096) g_ticks.push_back(static_cast<float>(tick));
}

float pct(std::vector<float>& v, double q)
{
    if (v.empty()) return 0;
    size_t k = std::min(v.size() - 1, static_cast<size_t>(q * double(v.size())));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return v[k];
}

void on_idle(int64_t start, void*)
{
    if(legacy_owner())return;
    double tick = ms_since(start);
    if (g_sleep && !g_timer) {
        // Created on the update thread itself; Windows 10 1803+ (and Wine) support high resolution.
        g_timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
        if (!g_timer) {
            swlog("update loop: high-resolution timer unavailable (%lu) - leaving the spin as is", GetLastError());
            g_sleep = false;
        }
    }
    if (g_sleep && g_timer && tick < g_wake_ms - 0.2) {
        LARGE_INTEGER due;
        due.QuadPart = -static_cast<LONGLONG>((g_wake_ms - tick) * 10000.0); // relative, 100 ns units
        if (SetWaitableTimerEx(g_timer, &due, 0, nullptr, nullptr, nullptr, 0)) {
            WaitForSingleObject(g_timer, 50);
            double woke = ms_since(start);
            g_slept_sum += woke - tick;
            if (woke > kLatest) {
                g_late++;
                g_wake_ms = std::max(kWakeMin, g_wake_ms - 0.5);
            } else if (g_wake_ms < kWakeMax && woke < g_wake_ms + 0.3) {
                g_wake_ms = std::min(kWakeMax, g_wake_ms + 0.01);
            }
        }
    }
    g_window_ms += 16.0;
    if (!g_window_start) g_window_start = GetTickCount64();
    if (g_window_ms >= 5000.0 && g_loops) {
        double wall = double(GetTickCount64() - g_window_start) / 1000.0;
        // tick = the game's own work per loop iteration (start -> idle), the sleep / spin excluded. A client that
        // can't keep up shows p95 near or over 16 ms and ticks/s under 62.5; "busy" is work / wall time.
        float p50 = pct(g_ticks, 0.5), p95 = pct(g_ticks, 0.95), p99 = pct(g_ticks, 0.99);
        swlog("loop 5s: %llu ticks (%.1f/s)  tick avg %.2f ms  p50 %.2f  p95 %.2f  p99 %.2f  worst %.1f ms  over 16 ms %d  "
              "busy %.0f%%  slept %.1f ms/tick  late wakes %d  wake at %.2f ms",
              static_cast<unsigned long long>(g_loops), wall > 0 ? g_loops / wall : 0, g_tick_sum / g_loops, p50, p95, p99,
              g_tick_max, g_over16, wall > 0 ? g_tick_sum / (wall * 10.0) : 0, g_slept_sum / g_loops, g_late, g_wake_ms);
        g_loops = 0;
        g_late = 0;
        g_over16 = 0;
        g_ticks.clear();
        g_window_start = GetTickCount64();
        g_tick_sum = g_tick_max = g_slept_sum = g_window_ms = 0;
    }
}

bool install()
{
    g_sleep = GetPrivateProfileIntW(L"cpu", L"update_loop_sleep", 1, settings_ini()) != 0;
    QueryPerformanceFrequency(&g_freq);
    if (!swml()->on_loop(swml_self(), nullptr, &on_end, &on_idle, nullptr)) {
        swlog("update loop: unavailable (the core didn't recognise the game loop, see swml.log)");
        return false;
    }
    swlog("update loop: %s (timing logged every 5 s as \"loop 5s:\")", g_sleep ? "sleep instead of spin" : "spin left as is, timing only");
    return true;
}

HookRegistrar reg{{"update loop sleep", false, &install}};

} // namespace
