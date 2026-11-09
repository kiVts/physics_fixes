
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>

namespace {

constexpr uintptr_t kCallSite = 0x140742fcc, kRayTest = 0x1401a4160;
constexpr size_t kCharPos = 0x68, kSheltered = 0x24d3;

using Ray_t = uint64_t(__fastcall*)(void* world, double* ray, void* result, void** hit, int mask, int group);
Ray_t g_ray = nullptr;

// Written by the stub (update thread only: simulate is single-threaded here).
struct StubData {
    uint8_t* character;
};
StubData* g_stub_data = nullptr;

struct Entry {
    double pos[3];
    uint64_t cast_ms, seen_ms;
};
std::unordered_map<uint8_t*, Entry> g_chars;

int g_refresh_ms = 300;
double g_move2 = 1.0, g_max_ray = 2000.0;
uint64_t g_calls = 0, g_casts = 0, g_last_log = 0, g_last_prune = 0;

uint64_t __fastcall rain_ray(void* world, double* ray, void* result, void** hit, int mask, int group)
{
    uint8_t* ch = g_stub_data->character;
    uint64_t now = GetTickCount64();
    g_calls++;
    const double* to = ray + 3;
    Entry& e = g_chars[ch];
    double dx = to[0] - e.pos[0], dy = to[1] - e.pos[1], dz = to[2] - e.pos[2];
    bool fresh = e.cast_ms && now - e.cast_ms < static_cast<uint64_t>(g_refresh_ms) && dx * dx + dy * dy + dz * dz < g_move2;
    e.seen_ms = now;
    uint64_t r;
    if (fresh) {
        *hit = nullptr;
        r = ch[kSheltered];
    } else {
        double len = std::sqrt((ray[0] - to[0]) * (ray[0] - to[0]) + (ray[1] - to[1]) * (ray[1] - to[1]) + (ray[2] - to[2]) * (ray[2] - to[2]));
        if (len > g_max_ray) {
            double s = g_max_ray / len;
            for (int i = 0; i < 3; i++) ray[i] = to[i] + (ray[i] - to[i]) * s;
        }
        r = g_ray(world, ray, result, hit, mask, group);
        // spread the next casts over ticks so all characters don't refresh together
        e.cast_ms = now - (reinterpret_cast<uintptr_t>(ch) >> 4) % 64;
        memcpy(e.pos, to, sizeof e.pos);
        g_casts++;
    }
    if (now - g_last_prune > 10000) {
        g_last_prune = now;
        for (auto it = g_chars.begin(); it != g_chars.end();)
            it = now - it->second.seen_ms > 10000 ? g_chars.erase(it) : std::next(it);
    }
    if (now - g_last_log >= 5000) {
        if (g_last_log && g_calls)
            swlog("rain shelter 5s: %.0f checks/s, %.0f rays cast/s (%.0f%% reused), %zu characters", g_calls / 5.0, g_casts / 5.0,
                  100.0 * (g_calls - g_casts) / g_calls, g_chars.size());
        g_last_log = now;
        g_calls = g_casts = 0;
    }
    return r;
}

uint8_t* alloc_near(uintptr_t base, size_t size)
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    uintptr_t gran = si.dwAllocationGranularity;
    for (uintptr_t d = 0x10000000; d < 0x70000000; d += gran) {
        uintptr_t a = (base + d) & ~(gran - 1);
        if (void* p = VirtualAlloc(reinterpret_cast<void*>(a), size, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE))
            return static_cast<uint8_t*>(p);
    }
    return nullptr;
}

bool install()
{
    const wchar_t* ini = settings_ini();
    if (!GetPrivateProfileIntW(L"rain", L"cache_shelter", 1, ini)) {
        swlog("rain shelter: off in settings.ini");
        return true;
    }
    g_refresh_ms = static_cast<int>(GetPrivateProfileIntW(L"rain", L"refresh_ms", 300, ini));
    double move = GetPrivateProfileIntW(L"rain", L"move_cm", 100, ini) / 100.0;
    g_move2 = move * move;
    g_max_ray = GetPrivateProfileIntW(L"rain", L"max_ray_m", 2000, ini);
    if (g_max_ray <= 0) g_max_ray = 1e9;

    auto site = reinterpret_cast<uint8_t*>(game_ptr(kCallSite));
    int32_t rel;
    memcpy(&rel, site + 1, 4);
    if (site[0] != 0xE8 || reinterpret_cast<uintptr_t>(site + 5 + rel) != reinterpret_cast<uintptr_t>(game_ptr(kRayTest))) {
        swlog("rain shelter: call site 0x%llx is not the expected call; off", static_cast<unsigned long long>(kCallSite));
        return true;
    }
    uint8_t* stub = alloc_near(reinterpret_cast<uintptr_t>(site), 4096);
    if (!stub) {
        swlog("rain shelter: no memory within 2 GB of the game; off");
        return true;
    }
    g_ray = game_ptr<Ray_t>(kRayTest);
    g_stub_data = reinterpret_cast<StubData*>(stub + 2048);

    // mov [rip + data], rsi / mov rax, rain_ray / jmp rax   (tail jump: rain_ray sees the game's own arguments)
    uint8_t* p = stub;
    *p++ = 0x48, *p++ = 0x89, *p++ = 0x35;
    int32_t d = static_cast<int32_t>(reinterpret_cast<uint8_t*>(g_stub_data) - (p + 4));
    memcpy(p, &d, 4), p += 4;
    *p++ = 0x48, *p++ = 0xB8;
    uint64_t target = reinterpret_cast<uint64_t>(&rain_ray);
    memcpy(p, &target, 8), p += 8;
    *p++ = 0xFF, *p++ = 0xE0;
    FlushInstructionCache(GetCurrentProcess(), stub, p - stub);

    int64_t nrel = reinterpret_cast<int64_t>(stub) - reinterpret_cast<int64_t>(site + 5);
    if (nrel != static_cast<int32_t>(nrel)) {
        swlog("rain shelter: stub out of call range; off");
        return true;
    }
    rel = static_cast<int32_t>(nrel);
    DWORD old;
    if (!VirtualProtect(site, 5, PAGE_EXECUTE_READWRITE, &old)) return false;
    memcpy(site + 1, &rel, 4);
    VirtualProtect(site, 5, old, &old);
    FlushInstructionCache(GetCurrentProcess(), site, 5);
    swlog("rain shelter: per-character ray reused for %d ms / %.2f m, ray cut to %.0f m (\"rain shelter 5s\" lines)", g_refresh_ms, move,
          g_max_ray);
    return true;
}

HookRegistrar reg{{"rain_shelter", /*needs_known_build=*/true, &install}};

} // namespace
