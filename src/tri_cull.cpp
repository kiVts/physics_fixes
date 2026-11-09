
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace {

constexpr uintptr_t kProcessTriangle = 0x1400e84c0;
constexpr size_t kAabbMin = 0x10, kAabbMax = 0x30;

using ProcessTriangle_t = void(__fastcall*)(void* self, double* tri, int part, int index);
ProcessTriangle_t o_process = nullptr;

double g_slack = 0.2;
std::atomic<uint64_t> g_calls{0}, g_culled{0};
std::atomic<uint64_t> g_last_log{0};

struct Counts {
    uint32_t calls = 0, culled = 0;
};
thread_local Counts tl;

void flush()
{
    g_calls.fetch_add(tl.calls, std::memory_order_relaxed);
    g_culled.fetch_add(tl.culled, std::memory_order_relaxed);
    tl = Counts{};
    uint64_t now = GetTickCount64(), last = g_last_log.load(std::memory_order_relaxed);
    if (now - last >= 5000 && g_last_log.compare_exchange_strong(last, now) && last) {
        uint64_t c = g_calls.exchange(0), k = g_culled.exchange(0);
        swlog("tri cull 5s: %.0f triangles/s reached GJK's door, %.0f/s skipped (%.0f%%): the convex's box is off the triangle's plane",
              c / 5.0, k / 5.0, c ? 100.0 * k / c : 0.0);
    }
}

// tri: three vertices, 4 doubles each (x, y, z, pad), in mesh space.
void __fastcall process_detour(void* self, double* tri, int part, int index)
{
    auto* s = static_cast<const uint8_t*>(self);
    const double* mn = reinterpret_cast<const double*>(s + kAabbMin);
    const double* mx = reinterpret_cast<const double*>(s + kAabbMax);
    const double *a = tri, *b = tri + 4, *c = tri + 8;
    double e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    double n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
    double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    bool cull = false;
    if (len > 1e-9) {
        double dist = 0, r = 0;
        for (int i = 0; i < 3; i++) {
            double ctr = (mn[i] + mx[i]) * 0.5, half = (mx[i] - mn[i]) * 0.5, ni = n[i] / len;
            dist += ni * (ctr - a[i]);
            r += std::fabs(ni) * half;
        }
        cull = std::fabs(dist) > r + g_slack;
    }
    tl.calls++;
    if (cull) tl.culled++;
    else o_process(self, tri, part, index);
    if (tl.calls >= 4096) flush();
}

bool install()
{
    const wchar_t* ini = settings_ini();
    if (!GetPrivateProfileIntW(L"narrowphase", L"tri_cull", 1, ini)) {
        swlog("tri cull: off in settings.ini");
        return true;
    }
    g_slack = GetPrivateProfileIntW(L"narrowphase", L"tri_cull_slack_cm", 20, ini) / 100.0;
    if (!hook_game(kProcessTriangle, reinterpret_cast<void*>(&process_detour), reinterpret_cast<void**>(&o_process))) return false;
    swlog("tri cull: convex vs mesh triangles off the convex's box plane by > %.2f m skipped (\"tri cull 5s\" lines)", g_slack);
    return true;
}

HookRegistrar reg{{"tri_cull", /*needs_known_build=*/true, &install}};

} // namespace
