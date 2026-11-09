
#include "microopt_ab.h"

#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <atomic>
#include <cstdint>
#include <cstring>

std::atomic<bool> g_microopt_off{false};

namespace {

constexpr uintptr_t kBoxAabb = 0x1401851c0;

using BoxAabb_t = double*(__fastcall*)(double* out, const double* m, const double* box);
BoxAabb_t g_orig = nullptr;
int g_mode = 2;

std::atomic<uint64_t> g_calls{0}, g_checked{0}, g_diff{0};
std::atomic<int> g_diff_logged{0};
std::atomic<uint64_t> g_last_log{0};

struct Counts {
    uint32_t calls = 0, checked = 0, diff = 0;
};
thread_local Counts tl;

inline void axis(double a, double b, double c, double t, const double* box, int ia, int ib, int ic, double& lo, double& hi)
{
    double p0 = box[ia] * a, p1 = box[ia + 3] * a;
    double q0 = box[ib] * b, q1 = box[ib + 3] * b;
    double r0 = box[ic] * c, r1 = box[ic + 3] * c;
    lo = ((p0 < p1 ? p0 : p1) + (q0 < q1 ? q0 : q1) + (r0 < r1 ? r0 : r1)) + t;
    hi = ((p0 > p1 ? p0 : p1) + (q0 > q1 ? q0 : q1) + (r0 > r1 ? r0 : r1)) + t;
}

void fast(double* out, const double* m, const double* box)
{
    // terms in the game's order: x' = x*m0 + y*m4 + z*m8; y' = y*m5 + x*m1 + z*m9; z' = y*m6 + x*m2 + z*m10
    axis(m[0], m[4], m[8], m[12], box, 0, 1, 2, out[0], out[3]);
    axis(m[5], m[1], m[9], m[13], box, 1, 0, 2, out[1], out[4]);
    axis(m[6], m[2], m[10], m[14], box, 1, 0, 2, out[2], out[5]);
}

bool same(const double* a, const double* b)
{
    for (int i = 0; i < 6; i++)
        if (!(a[i] == b[i]) && !(a[i] != a[i] && b[i] != b[i])) return false; // == treats +-0 alike; NaN vs NaN alike
    return true;
}

void flush()
{
    g_calls.fetch_add(tl.calls, std::memory_order_relaxed);
    g_checked.fetch_add(tl.checked, std::memory_order_relaxed);
    g_diff.fetch_add(tl.diff, std::memory_order_relaxed);
    tl = Counts{};
    uint64_t now = GetTickCount64(), last = g_last_log.load(std::memory_order_relaxed);
    if (!last) g_last_log.compare_exchange_strong(last, now);
    else if (now - last >= 30000 && g_last_log.compare_exchange_strong(last, now)) {
        uint64_t c = g_calls.exchange(0), k = g_checked.exchange(0), d = g_diff.exchange(0);
        swlog("box aabb 30s: %llu calls (%.0f/s), %llu checked against the game's, %llu different", (unsigned long long)c,
              c / 30.0, (unsigned long long)k, (unsigned long long)d);
    }
}

double* __fastcall detour(double* out, const double* m, const double* box)
{
    if (!microopt_on()) return g_orig(out, m, box);
    double mine[6];
    fast(mine, m, box);
    bool check = g_mode == 1 || (tl.calls & 4095) == 0;
    tl.calls++;
    if (check) {
        g_orig(out, m, box);
        tl.checked++;
        if (!same(mine, out)) {
            tl.diff++;
            if (g_diff_logged.fetch_add(1) < 5)
                swlog("box aabb: differs: game (%g %g %g)-(%g %g %g), ours (%g %g %g)-(%g %g %g)", out[0], out[1], out[2],
                      out[3], out[4], out[5], mine[0], mine[1], mine[2], mine[3], mine[4], mine[5]);
        }
        if (g_mode == 1) {
            if (tl.calls >= 4096) flush();
            return out;
        }
    }
    memcpy(out, mine, sizeof mine);
    if (tl.calls >= 4096) flush();
    return out;
}

DWORD WINAPI ab_loop(void*)
{
    bool off = false;
    swlog("micro-opts: ON window");
    for (;;) {
        Sleep(30000);
        off = !off;
        g_microopt_off.store(off);
        swlog("micro-opts: %s window", off ? "OFF" : "ON");
    }
}

bool install()
{
    if (GetPrivateProfileIntW(L"microopt", L"ab", 0, settings_ini()))
        CloseHandle(CreateThread(nullptr, 0, ab_loop, nullptr, 0, nullptr));
    g_mode = static_cast<int>(GetPrivateProfileIntW(L"box_aabb", L"mode", 0, settings_ini()));
    if (g_mode <= 0) {
        swlog("box aabb: off in settings.ini");
        return true;
    }
    if (!hook_game(kBoxAabb, reinterpret_cast<void*>(&detour), reinterpret_cast<void**>(&g_orig))) return false;
    swlog("box aabb: %s (box through a transform, separable min / max, \"box aabb 30s\" lines)",
          g_mode == 1 ? "check (the game's answer kept)" : "on");
    return true;
}

HookRegistrar reg{{"box aabb", /*needs_known_build=*/true, &install}};

} // namespace
