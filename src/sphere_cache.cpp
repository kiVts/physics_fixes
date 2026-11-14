
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <vector>

namespace {

constexpr uintptr_t kTest = 0x14048e810;
constexpr int kSlots = 2048, kProbe = 8, kCheckEvery = 512;

// returns a bool in al (callers test the low byte only: the upper bits of rax are left over on the miss path)
using Test_t = uint8_t(__fastcall*)(const uint8_t* shape, const double* point, float* value, uint32_t* cell);
Test_t g_orig = nullptr;
std::atomic<bool> g_on{false};
std::atomic<uint32_t> g_tick{1};
double g_margin = 1.0;

struct Slot {
    const uint8_t* q;
    uint32_t tick;
    double t[3]; // the matrix translation the sphere was read with
    double c[3];
    double lim2; // (2r + 2 |local offset| + margin)^2
};
struct Cache {
    Slot slot[kSlots];
    // stats, read racily by the logger
    uint64_t calls = 0, rejects = 0, fills = 0, checks = 0, overflow = 0, moved = 0, raced = 0;
};
thread_local Cache* tl_cache = nullptr;
std::mutex g_caches_mu;
std::vector<Cache*> g_caches;

template <typename T> T rd(const uint8_t* p, size_t off)
{
    T v;
    memcpy(&v, p + off, sizeof v);
    return v;
}

Cache* cache()
{
    if (tl_cache) return tl_cache;
    Cache* c = new Cache();
    memset(c->slot, 0, sizeof c->slot);
    std::lock_guard<std::mutex> lock(g_caches_mu);
    g_caches.push_back(c);
    return tl_cache = c;
}

void sphere(const uint8_t* q, Slot& s)
{
    double l0 = rd<double>(q, 0x148), l1 = rd<double>(q, 0x150), l2 = rd<double>(q, 0x158);
    for (int k = 0; k < 3; k++) {
        s.t[k] = rd<double>(q, 0x568 + 8 * k);
        s.c[k] = l0 * rd<double>(q, 0x508 + 8 * k) + l1 * rd<double>(q, 0x528 + 8 * k) + l2 * rd<double>(q, 0x548 + 8 * k) + s.t[k];
    }
    // a rotation alone (same translation) moves the centre by at most 2 |l|
    double lim = 2.0 * std::fabs(static_cast<double>(rd<float>(q, 0x160))) + 2.0 * std::sqrt(l0 * l0 + l1 * l1 + l2 * l2) + g_margin;
    s.lim2 = std::isfinite(lim) && std::isfinite(s.c[0] + s.c[1] + s.c[2]) ? lim * lim : INFINITY;
}

void check_failed(const uint8_t* q, const double* p, const Slot& s)
{
    if (!g_on.exchange(false)) return;
    double d2 = 0;
    for (int k = 0; k < 3; k++) d2 += (p[k] - s.c[k]) * (p[k] - s.c[k]);
    swlog("sphere cache: the game's test hit a point the cache rejected (body shape %p, distance %.2f m, limit %.2f m); "
          "cache off for the session", q, std::sqrt(d2), std::sqrt(s.lim2));
}

uint8_t __fastcall test_detour(const uint8_t* q, const double* p, float* value, uint32_t* cell)
{
    if (!g_on.load(std::memory_order_relaxed) || !q || !p) return g_orig(q, p, value, cell);
    Cache* c = cache();
    c->calls++;
    uint32_t tick = g_tick.load(std::memory_order_relaxed);
    size_t h = static_cast<size_t>((reinterpret_cast<uintptr_t>(q) >> 4) * 0x9E3779B97F4A7C15ull >> 53); // 11 bits
    Slot* s = nullptr;
    for (int i = 0; i < kProbe; i++) {
        Slot& x = c->slot[(h + i) & (kSlots - 1)];
        if (x.q == q && x.tick == tick) {
            if (memcmp(x.t, q + 0x568, sizeof x.t) != 0) { // the body moved / was replaced since: read it again
                sphere(q, x);
                c->moved++;
            }
            s = &x;
            break;
        }
        if (x.tick != tick) { // empty or from an earlier tick: take it
            x.q = q;
            x.tick = tick;
            sphere(q, x);
            c->fills++;
            s = &x;
            break;
        }
    }
    if (!s) {
        c->overflow++;
        return g_orig(q, p, value, cell);
    }
    double dx = p[0] - s->c[0], dy = p[1] - s->c[1], dz = p[2] - s->c[2];
    if (!(dx * dx + dy * dy + dz * dz > s->lim2)) return g_orig(q, p, value, cell);
    c->rejects++;
    if (c->rejects % kCheckEvery == 0) {
        c->checks++;
        float v;
        uint32_t cl;
        if (g_orig(q, p, &v, &cl)) {
            Slot a = *s, b = *s;
            sphere(q, a);
            bool hit = g_orig(q, p, &v, &cl) != 0;
            sphere(q, b);
            double fx = p[0] - a.c[0], fy = p[1] - a.c[1], fz = p[2] - a.c[2];
            bool still = memcmp(a.c, b.c, sizeof a.c) == 0 && a.lim2 == b.lim2;
            if (still && hit && fx * fx + fy * fy + fz * fz > a.lim2) check_failed(q, p, a);
            else {
                c->raced++;
                *s = b;
            }
        }
    }
    return 0;
}

DWORD WINAPI log_loop(void*)
{
    uint64_t last[7] = {};
    for (;;) {
        Sleep(5000);
        if (!g_on) continue;
        uint64_t sum[7] = {};
        {
            std::lock_guard<std::mutex> lock(g_caches_mu);
            for (Cache* c : g_caches) {
                sum[0] += c->calls, sum[1] += c->rejects, sum[2] += c->fills, sum[3] += c->checks, sum[4] += c->overflow, sum[5] += c->moved, sum[6] += c->raced;
            }
        }
        uint64_t d[7];
        for (int i = 0; i < 7; i++) d[i] = sum[i] - last[i], last[i] = sum[i];
        if (!d[0]) continue;
        swlog("sphere cache 5s: %llu tests, %.1f%% rejected from the cache (%llu sphere reads, %llu table full), "
              "%llu re-read after the body moved, %llu self-checks (%llu of them a body that moved between the decision and the check)",
              static_cast<unsigned long long>(d[0]), 100.0 * double(d[1]) / double(d[0]), static_cast<unsigned long long>(d[2]),
              static_cast<unsigned long long>(d[4]), static_cast<unsigned long long>(d[5]), static_cast<unsigned long long>(d[3]), static_cast<unsigned long long>(d[6]));
    }
}

bool install()
{
    std::wstring ini = settings_ini();
    if (!GetPrivateProfileIntW(L"contain", L"sphere_cache", 0, ini.c_str())) {
        swlog("sphere cache: off ([contain] sphere_cache=1 to try it; measured no gain, see RENDER_PREP_NEXT.md)");
        return true;
    }
    g_margin = std::max(1, static_cast<int>(GetPrivateProfileIntW(L"contain", L"sphere_margin_m", 1, ini.c_str())));
    if (!swml()->on_loop(swml_self(), [](void*) { g_tick.fetch_add(1, std::memory_order_relaxed); }, nullptr, nullptr, nullptr)) {
        swlog("sphere cache: game loop events unavailable; off");
        return true;
    }
    if (!hook_game(kTest, reinterpret_cast<void*>(&test_detour), reinterpret_cast<void**>(&g_orig))) return false;
    g_on = true;
    CloseHandle(CreateThread(nullptr, 0, log_loop, nullptr, 0, nullptr));
    swlog("sphere cache: on (the body containment test's sphere early-out from a per-tick cache, margin %.0f m; "
          "\"sphere cache 5s\" lines)", g_margin);
    return true;
}

HookRegistrar reg{{"sphere cache", /*needs_known_build=*/true, &install}};

} // namespace
