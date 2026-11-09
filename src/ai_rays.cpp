

#include "ai_rays_stub.h"
#include "parallel.h"

#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "game.h"

#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <initializer_list>

namespace {

constexpr uintptr_t kWrapper = 0x1401a4710, kMmRayVtable = 0x140b06158, kGameFree = 0x1409b2d50;
constexpr uintptr_t kSiteRets[] = {0x14073aa41, 0x14073bafa, 0x14073bed1, 0x14073c24e,
                                   0x14073d334, 0x14073d661, 0x14073da50, 0x14073dd19};
constexpr int kSites = sizeof(kSiteRets) / sizeof(kSiteRets[0]);
constexpr size_t kEntry = 0x30, kSceneOwner = 0xc0, kObjUser = 0x1f8, kOwnerId = 4;
constexpr int kOwnerCharacter = 2;

struct Ring { uint8_t* data; uint32_t cap, head, count; };
using Wrapper_t = uint64_t(__fastcall*)(void* self, const double* ray, Ring* ring, uint32_t group, uint32_t mask);
using Needs_t = bool(__fastcall*)(const void* callback, const void* proxy);
using Free_t = void(__cdecl*)(void*);

Wrapper_t g_wrap = nullptr;
Needs_t g_needs = nullptr;
Free_t g_free = nullptr;
int g_mode = 2;
uint32_t g_id_off = 0;

struct Filter {
    bool active = false;
    int veh_id = 0;
    uint32_t skipped = 0;
};
thread_local Filter tl_f;
thread_local uint32_t tl_seq = 0;

std::atomic<uint64_t> g_calls{0}, g_skipped{0}, g_cycles{0}, g_checked{0}, g_check_cycles_full{0}, g_check_cycles_ours{0},
    g_diff{0}, g_failed_reads{0};
std::atomic<int> g_diff_logged{0};
std::atomic<uint64_t> g_last_log{0};

// The consumers' test, on an entry's scene object. false = every consumer skips it.
bool keep_scene(uintptr_t scene, int veh_id)
{
    if (!scene) return false;
    uintptr_t owner = *reinterpret_cast<const uintptr_t*>(scene + kSceneOwner);
    if (!owner) return true;
    if (*reinterpret_cast<const int*>(owner) == kOwnerCharacter) return false;
    return !veh_id || *reinterpret_cast<const int*>(owner + kOwnerId) != veh_id;
}

// proxy -> client object -> user pointer; 1 keep, 0 skip, -1 couldn't read (keep)
int keep_proxy(const void* proxy, int veh_id)
{
    __try {
        uintptr_t obj = *static_cast<const uintptr_t*>(proxy);
        if (!obj) return 1;
        return keep_scene(*reinterpret_cast<const uintptr_t*>(obj + kObjUser), veh_id) ? 1 : 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return -1;
    }
}

bool __fastcall needs_detour(const void* callback, const void* proxy)
{
    bool r = g_needs(callback, proxy);
    if (!r || !tl_f.active) return r;
    int k = keep_proxy(proxy, tl_f.veh_id);
    if (k < 0) g_failed_reads.fetch_add(1, std::memory_order_relaxed);
    if (k != 0) return true;
    tl_f.skipped++;
    return false;
}

uint8_t* entry(const Ring& r, uint32_t i) { return r.data + size_t((r.head + i) % r.cap) * kEntry; }

// The entries of `full` (from index `from`) every consumer keeps == the entries of `ours` (from `ours_from`)?
bool same_kept(const Ring& full, uint32_t from, const Ring& ours, uint32_t ours_from, int veh_id, uint32_t* kept)
{
    __try {
        uint32_t j = ours_from;
        *kept = 0;
        for (uint32_t i = from; i < full.count; i++) {
            const uint8_t* e = entry(full, i);
            if (!keep_scene(*reinterpret_cast<const uintptr_t*>(e), veh_id)) continue;
            ++*kept;
            if (j >= ours.count || memcmp(e, entry(ours, j), kEntry) != 0) return false;
            j++;
        }
        return j == ours.count;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

int read_id(uintptr_t veh)
{
    __try {
        return veh ? *reinterpret_cast<const int*>(veh + g_id_off) : 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
}

void maybe_log()
{
    uint64_t now = GetTickCount64(), last = g_last_log.load(std::memory_order_relaxed);
    if (!last) {
        g_last_log.compare_exchange_strong(last, now);
        return;
    }
    if (now - last < 30000 || !g_last_log.compare_exchange_strong(last, now)) return;
    static double cyc_per_us = 0;
    if (!cyc_per_us) {
        LARGE_INTEGER f, q0, q1;
        QueryPerformanceFrequency(&f);
        QueryPerformanceCounter(&q0);
        uint64_t c0 = __rdtsc();
        Sleep(10);
        QueryPerformanceCounter(&q1);
        cyc_per_us = double(__rdtsc() - c0) / (double(q1.QuadPart - q0.QuadPart) * 1e6 / double(f.QuadPart));
    }
    uint64_t c = g_calls.exchange(0), s = g_skipped.exchange(0), cy = g_cycles.exchange(0), k = g_checked.exchange(0),
             kf = g_check_cycles_full.exchange(0), ko = g_check_cycles_ours.exchange(0), d = g_diff.exchange(0),
             fr = g_failed_reads.exchange(0);
    double ours_us = c ? cy / cyc_per_us / c : 0, full_us = k ? kf / cyc_per_us / k : 0, ours_k_us = k ? ko / cyc_per_us / k : 0;
    swlog("ai rays 30s: %llu calls (%.0f/s), %.1f objects skipped per call, %.2f us per call; %llu checked against the full "
          "ray: full %.2f us vs ours %.2f us on those, %llu different%s",
          (unsigned long long)c, c / 30.0, c ? double(s) / c : 0.0, ours_us, (unsigned long long)k, full_us, ours_k_us,
          (unsigned long long)d, fr ? " [some objects unreadable: kept]" : "");
}

// One wrapper call, filtered or not; returns its cycles.
uint64_t timed(bool filtered, int id, void* self, const double* ray, Ring* ring, uint32_t group, uint32_t mask, uint64_t& r)
{
    uint64_t t0 = __rdtsc();
    if (filtered) {
        tl_f.active = true;
        tl_f.veh_id = id;
        tl_f.skipped = 0;
    }
    r = g_wrap(self, ray, ring, group, mask);
    if (filtered) {
        tl_f.active = false;
        g_skipped.fetch_add(tl_f.skipped, std::memory_order_relaxed);
    }
    return __rdtsc() - t0;
}

// The 8 sites' calls land here (through the stub: veh = the caller's rbx, the AI's vehicle).
uint64_t __fastcall site_detour(void* self, const double* ray, Ring* ring, uint32_t group, uint32_t mask, uintptr_t veh)
{
    int id = read_id(veh);
    bool check = g_mode == 1 || (++tl_seq & 255) == 0;
    uint32_t before = ring->count;
    uint64_t r, r_side;
    // a compared pair sees one world: both rays inside one read of the world lock (the step may be running)
    if (check) world_read_begin();
    // the caller's ring gets the game's full ray in check mode, ours when on
    uint64_t d = timed(g_mode != 1, id, self, ray, ring, group, mask, r);
    if (g_mode == 1) g_check_cycles_full.fetch_add(d, std::memory_order_relaxed);
    else {
        g_calls.fetch_add(1, std::memory_order_relaxed);
        g_cycles.fetch_add(d, std::memory_order_relaxed);
        if (check) g_check_cycles_ours.fetch_add(d, std::memory_order_relaxed);
    }
    if (check) {
        Ring side{nullptr, 0, 0, 0}; // the other variant
        uint64_t ds = timed(g_mode == 1, id, self, ray, &side, group, mask, r_side);
        world_read_end();
        uint32_t kept = 0;
        bool same;
        if (g_mode == 1) {
            g_calls.fetch_add(1, std::memory_order_relaxed);
            g_cycles.fetch_add(ds, std::memory_order_relaxed);
            g_check_cycles_ours.fetch_add(ds, std::memory_order_relaxed);
            same = same_kept(*ring, before, side, 0, id, &kept);
        } else {
            g_check_cycles_full.fetch_add(ds, std::memory_order_relaxed);
            same = same_kept(side, 0, *ring, before, id, &kept);
        }
        g_checked.fetch_add(1, std::memory_order_relaxed);
        if (!same) {
            g_diff.fetch_add(1, std::memory_order_relaxed);
            if (g_diff_logged.fetch_add(1) < 10)
                swlog("ai rays: differs from the full ray (vehicle id %d, %u kept entries in the full ray, %u from ours)", id,
                      kept, g_mode == 1 ? side.count : ring->count - before);
        }
        if (side.cap && side.data) g_free(side.data);
    }
    maybe_log();
    return r;
}

uint8_t* alloc_near(uintptr_t target, size_t size)
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    uintptr_t gran = si.dwAllocationGranularity;
    for (uintptr_t d = gran; d < 0x70000000; d += gran)
        for (uintptr_t a : {target - d, target + d}) {
            a &= ~(gran - 1);
            if (void* m = VirtualAlloc(reinterpret_cast<void*>(a), size, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE))
                return static_cast<uint8_t*>(m);
        }
    return nullptr;
}

bool write_code(void* at, const void* bytes, size_t n)
{
    DWORD old;
    if (!VirtualProtect(at, n, PAGE_EXECUTE_READWRITE, &old)) return false;
    memcpy(at, bytes, n); // mod init: before the game's loop runs
    VirtualProtect(at, n, old, &old);
    FlushInstructionCache(GetCurrentProcess(), at, n);
    return true;
}

// The vehicle id offset from "mov ecx|edx, [rbx+disp32]" (8B 8B / 8B 93) within 0x80 bytes after the call.
uint32_t id_offset_after(const uint8_t* ret)
{
    for (int i = 0; i < 0x80; i++)
        if (ret[i] == 0x8B && (ret[i + 1] == 0x8B || ret[i + 1] == 0x93)) {
            uint32_t d;
            memcpy(&d, ret + i + 2, 4);
            if (d >= 0x100 && d < 0x1000) return d;
        }
    return 0;
}

bool install()
{
    g_mode = static_cast<int>(GetPrivateProfileIntW(L"ai_rays", L"mode", 2, settings_ini()));
    if (g_mode <= 0) {
        swlog("ai rays: off in settings.ini");
        return true;
    }
    uintptr_t wrapper = game_live(kWrapper);
    auto** vtable = reinterpret_cast<void**>(game_live(kMmRayVtable));
    g_free = game_ptr<Free_t>(kGameFree);
    if (!wrapper || !vtable || !g_free) return false;
    uint8_t* sites[kSites];
    for (int i = 0; i < kSites; i++) {
        auto* ret = reinterpret_cast<uint8_t*>(game_live(kSiteRets[i]));
        if (!ret || ret[-5] != 0xE8) {
            swlog("ai rays: call %d isn't a call; off", i);
            return true;
        }
        int32_t rel;
        memcpy(&rel, ret - 4, 4);
        if (reinterpret_cast<uintptr_t>(ret) + rel != wrapper) {
            swlog("ai rays: call %d doesn't reach the ray wrapper; off", i);
            return true;
        }
        uint32_t off = id_offset_after(ret);
        if (!off || (g_id_off && off != g_id_off)) {
            swlog("ai rays: call %d: vehicle id offset not found or different (%#x vs %#x); off", i, off, g_id_off);
            return true;
        }
        g_id_off = off;
        sites[i] = ret - 5;
    }
    uint8_t* stub = alloc_near(wrapper, 4096);
    if (!stub) {
        swlog("ai rays: no memory within 2 GB of the game; off");
        return true;
    }
    if (!make_rbx_stub(stub, reinterpret_cast<const void*>(&site_detour))) {
        swlog("ai rays: unwind data not accepted; off");
        return true;
    }

    g_wrap = reinterpret_cast<Wrapper_t>(wrapper); // the wrapper's entry: through any hooks on it
    g_needs = reinterpret_cast<Needs_t>(vtable[1]);
    void* nd = reinterpret_cast<void*>(&needs_detour);
    if (!write_code(&vtable[1], &nd, sizeof nd)) {
        swlog("ai rays: the callback's vtable can't be written; off");
        return true;
    }
    for (int i = 0; i < kSites; i++) {
        int64_t rel = reinterpret_cast<int64_t>(stub) - reinterpret_cast<int64_t>(sites[i] + 5);
        if (rel != int32_t(rel)) {
            swlog("ai rays: stub out of reach of call %d; calls before it patched, the rest left", i);
            return true;
        }
        int32_t r32 = int32_t(rel);
        if (!write_code(sites[i] + 1, &r32, 4)) return false;
    }
    swlog("ai rays: %s (the AI's obstacle rays skip its own vehicle and characters: %d calls, vehicle id at +%#x; "
          "\"ai rays 30s\" lines)",
          g_mode == 1 ? "check (the game's full ray kept, ours compared)" : "on", kSites, g_id_off);
    return true;
}

HookRegistrar reg{{"ai rays", /*needs_known_build=*/true, &install}};

} // namespace
