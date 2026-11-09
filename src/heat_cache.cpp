
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "phys_pool.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

namespace {

constexpr uintptr_t kBox = 0x1404bf330, kHeatFieldRet = 0x14054a322, kZone12 = 0x1401a3b60, kGhostContacts = 0x14019e070;

using Box_t = void(__fastcall*)(void* ring, double* out);
using Zone12_t = void(__fastcall*)(void* scene, void* a2, void* a3, void* a4);
Box_t o_box = nullptr;
Zone12_t o_zone12 = nullptr;

using Ghost_t = bool(__fastcall*)(void* ghost, void* world);
Ghost_t o_ghost = nullptr;
bool g_par_ghosts = true, g_deferring = false;
struct GhostCall {
    void *ghost, *world;
};
std::vector<GhostCall> g_ghosts;
std::atomic<int> g_ghost_next{0};
uint64_t g_ghost_batches = 0, g_ghost_calls = 0, g_ghost_cyc = 0;

bool __fastcall ghost_detour(void* ghost, void* world)
{
    if (puppet_skip_ghost(ghost)) {
        *reinterpret_cast<uint64_t*>(static_cast<uint8_t*>(ghost) + 0x164) = 0;
        return true;
    }
    if (!g_deferring) return o_ghost(ghost, world);
    g_ghosts.push_back({ghost, world});
    return true; // the caller ignores it
}

void ghost_job(int, void*)
{
    int n = static_cast<int>(g_ghosts.size());
    for (;;) {
        int i = g_ghost_next.fetch_add(4, std::memory_order_relaxed);
        if (i >= n) return;
        for (int k = i; k < i + 4 && k < n; k++) o_ghost(g_ghosts[k].ghost, g_ghosts[k].world);
    }
}
uintptr_t g_ret = 0;

struct Slot {
    const void* key;
    uint32_t gen;
    double box[6];
};
std::vector<Slot> g_tab(1 << 10);
uint32_t g_gen = 1;
size_t g_used = 0;
std::mutex g_mx;
int g_verify_every = 64;
bool g_off = false;
uint64_t g_calls = 0, g_hits = 0, g_verified = 0, g_diff = 0, g_last_log = 0;

void maybe_log()
{
    uint64_t now = GetTickCount64();
    if (now - g_last_log < 5000) return;
    if (g_last_log && g_calls)
        swlog("heat boxes 5s: %.0f vehicle boxes/s asked by the heat field, %.0f/s from this tick's cache (%.0f%%), %llu re-checked, %llu"
              " different%s",
              g_calls / 5.0, g_hits / 5.0, 100.0 * g_hits / g_calls, static_cast<unsigned long long>(g_verified),
              static_cast<unsigned long long>(g_diff), g_off ? " (cache off)" : "");
    if (g_last_log && g_ghost_batches) {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        static double cyc_per_ms = 0;
        if (!cyc_per_ms) {
            LARGE_INTEGER a, b;
            QueryPerformanceCounter(&a);
            uint64_t t = __rdtsc();
            Sleep(2);
            QueryPerformanceCounter(&b);
            cyc_per_ms = double(__rdtsc() - t) / (double(b.QuadPart - a.QuadPart) * 1000.0 / double(f.QuadPart));
        }
        swlog("ghost contacts 5s: %.0f ghosts per tick on the physics pool, %.3f ms per tick", double(g_ghost_calls) / g_ghost_batches,
              g_ghost_cyc / cyc_per_ms / g_ghost_batches);
    }
    g_ghost_batches = g_ghost_calls = g_ghost_cyc = 0;
    g_last_log = now;
    g_calls = g_hits = g_verified = 0;
}

Slot* find(const void* key)
{
    size_t mask = g_tab.size() - 1;
    size_t h = ((reinterpret_cast<uintptr_t>(key) >> 4) * 0x9E3779B97F4A7C15ull) >> 22;
    for (size_t i = 0; i <= mask; i++) {
        Slot& s = g_tab[(h + i) & mask];
        if (s.gen != g_gen || s.key == key) return &s;
    }
    return nullptr;
}

void __fastcall box_detour(void* ring, double* out)
{
    if (g_off || reinterpret_cast<uintptr_t>(_ReturnAddress()) != g_ret) return o_box(ring, out);
    std::lock_guard<std::mutex> lk(g_mx);
    g_calls++;
    if (g_used * 2 >= g_tab.size()) { // grow: keep the load under half
        std::vector<Slot> old;
        old.swap(g_tab);
        g_tab.assign(old.size() * 2, Slot{});
        g_used = 0;
        for (const Slot& s : old)
            if (s.gen == g_gen) {
                *find(s.key) = s;
                g_used++;
            }
    }
    Slot* s = find(ring);
    if (s && s->gen == g_gen && s->key == ring) {
        g_hits++;
        if (g_verify_every > 0 && g_hits % g_verify_every == 0) {
            double fresh[6];
            memcpy(fresh, out, sizeof fresh); // the caller pre-zeroes its buffer
            o_box(ring, fresh);
            g_verified++;
            if (memcmp(fresh, s->box, sizeof fresh) != 0 && ++g_diff >= 100 && !g_off) {
                g_off = true;
                swlog("heat boxes: 100 cached boxes differed from a fresh one; cache off");
            }
            memcpy(out, fresh, sizeof fresh);
        } else {
            memcpy(out, s->box, sizeof s->box);
        }
    } else {
        o_box(ring, out);
        if (s) {
            s->key = ring;
            s->gen = g_gen;
            memcpy(s->box, out, sizeof s->box);
            g_used++;
        }
    }
    maybe_log();
}

void __fastcall zone12_detour(void* scene, void* a2, void* a3, void* a4)
{
    {
        std::lock_guard<std::mutex> lk(g_mx);
        if (++g_gen == 0) {
            for (auto& s : g_tab) s.gen = 0;
            g_gen = 1;
        }
        g_used = 0;
    }
    if (!g_par_ghosts) return o_zone12(scene, a2, a3, a4);
    g_ghosts.clear();
    g_deferring = true;
    o_zone12(scene, a2, a3, a4);
    g_deferring = false;
    uint64_t t0 = __rdtsc();
    g_ghost_next.store(0, std::memory_order_relaxed);
    phys_pool_run(g_ghosts.size() >= 32 ? phys_pool_threads() - 1 : 0, &ghost_job, nullptr);
    g_ghost_cyc += __rdtsc() - t0;
    g_ghost_batches++;
    g_ghost_calls += g_ghosts.size();
}

bool install()
{
    const wchar_t* ini = settings_ini();
    bool puppet_ghosts = GetPrivateProfileIntW(L"puppets", L"cull", 1, ini) && GetPrivateProfileIntW(L"puppets", L"cull_ghosts", 1, ini);
    if (!GetPrivateProfileIntW(L"heat", L"cache_boxes", 1, ini)) {
        swlog("heat boxes: off in settings.ini");
        // The ghost hook alone, for the puppet cull (ghosts run in the game's own order, not deferred).
        return !puppet_ghosts || hook_game(kGhostContacts, reinterpret_cast<void*>(&ghost_detour), reinterpret_cast<void**>(&o_ghost));
    }
    g_verify_every = static_cast<int>(GetPrivateProfileIntW(L"heat", L"verify_every", 64, ini));
    g_par_ghosts = GetPrivateProfileIntW(L"heat", L"parallel_ghosts", 1, ini) != 0;
    g_ret = reinterpret_cast<uintptr_t>(game_ptr(kHeatFieldRet));
    if (!hook_game(kBox, reinterpret_cast<void*>(&box_detour), reinterpret_cast<void**>(&o_box)) ||
        !hook_game(kZone12, reinterpret_cast<void*>(&zone12_detour), reinterpret_cast<void**>(&o_zone12)) ||
        ((g_par_ghosts || puppet_ghosts) && !hook_game(kGhostContacts, reinterpret_cast<void*>(&ghost_detour), reinterpret_cast<void**>(&o_ghost))))
        return false;
    if (g_par_ghosts) phys_pool_threads();
    swlog("heat boxes: vehicle bounding boxes for temperature queries cached per tick, re-checked every %d hits (\"heat boxes 5s\" lines)",
          g_verify_every);
    return true;
}

HookRegistrar reg{{"heat_boxes", /*needs_known_build=*/true, &install}};

} // namespace
