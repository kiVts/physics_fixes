
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr uintptr_t kJumpTable = 0x1408639dc, kCase94 = 0x14086113a, kCaseDone = 0x14085b184, kApplyMessages = 0x14085aeb0;
constexpr int kType = 0x94;
constexpr uintptr_t kFindVehicle = 0x140549fc0, kVehicleValid = 0x140546af0;
size_t game_vehicles() { return per_build<size_t>(0x16a48, 0x16a50); }
constexpr size_t kMsgVehicleId = 0x10, kVehicleBodies = 8, kBodyComponents = 0x3f0;
constexpr size_t kCompCastSlot = 0x118, kRadarDef = 0x58, kDefType = 0xe70, kRadarFlag = 0x6d8;
constexpr int kRadarType = 0x38;

using Find_t = uint8_t*(__fastcall*)(uint8_t* table, int id);
using Valid_t = bool(__fastcall*)(uint8_t* vehicle);
using Cast_t = uint8_t*(__fastcall*)(uint8_t* component);
using Apply_t = void(__fastcall*)(void*, void*, void*, void*, void*, void*);

Find_t g_find = nullptr;
Valid_t g_valid = nullptr;
Apply_t o_apply = nullptr;

struct Ring {
    uint8_t** data;
    uint32_t cap, first, count;
};

Ring ring_at(uint8_t* p)
{
    Ring r;
    r.data = *reinterpret_cast<uint8_t***>(p);
    r.cap = *reinterpret_cast<uint32_t*>(p + 8);
    r.first = *reinterpret_cast<uint32_t*>(p + 0xc);
    r.count = *reinterpret_cast<uint32_t*>(p + 0x10);
    return r;
}
uint8_t* ring_get(const Ring& r, uint32_t i) { return r.data[(r.first + i) % r.cap]; }

struct Cached {
    std::vector<uint8_t*> components; // every body's components in walk order, with a nullptr after each body
    std::vector<uint8_t*> radars;     // the cast results that are radars, in walk order
    uint32_t hits = 0;
    std::vector<uint64_t> shape; // per body: ring data pointer, capacity, first, count (cheap change check)
    uint64_t batch = 0;          // the message batch in which the radars were last flagged
};

std::unordered_map<uint8_t*, Cached> g_cache;
bool g_on = true, g_cache_off = false;
int g_verify_every = 8;
std::vector<uint8_t*> g_scratch, g_scratch_radars;

// stats
uint64_t g_msgs = 0, g_hits = 0, g_walks = 0, g_components_walked = 0, g_cycles = 0, g_verified = 0, g_batches = 0, g_repeat = 0;
std::unordered_set<uint8_t*> g_batch_seen;
uint64_t g_batch_id = 0;
uint64_t g_skipped = 0;
std::vector<uint64_t> g_shape;
ULONGLONG g_window = 0;
uint64_t g_window_tsc = 0;

// O(bodies): every body's component ring header. Equal shape within one batch = flags still set, nothing to do.
void read_shape(uint8_t* bodies_ring, std::vector<uint64_t>& out)
{
    out.clear();
    Ring bodies = ring_at(bodies_ring);
    out.push_back(reinterpret_cast<uint64_t>(bodies.data) ^ (uint64_t(bodies.count) << 48));
    for (uint32_t b = 0; b < bodies.count; b++) {
        Ring comps = ring_at(ring_get(bodies, b) + kBodyComponents);
        out.push_back(reinterpret_cast<uint64_t>(comps.data));
        out.push_back(uint64_t(comps.cap) | uint64_t(comps.first) << 32);
        out.push_back(comps.count);
    }
}

void read_components(uint8_t* bodies_ring, std::vector<uint8_t*>& out)
{
    out.clear();
    Ring bodies = ring_at(bodies_ring);
    for (uint32_t b = 0; b < bodies.count; b++) {
        Ring comps = ring_at(ring_get(bodies, b) + kBodyComponents);
        for (uint32_t c = 0; c < comps.count; c++) out.push_back(ring_get(comps, c));
        out.push_back(nullptr);
    }
}

// The game's walk: radar cast results in order (flags not set here).
void walk(const std::vector<uint8_t*>& components, std::vector<uint8_t*>& radars)
{
    radars.clear();
    for (uint8_t* c : components) {
        if (!c) continue;
        auto cast = (*reinterpret_cast<Cast_t**>(c))[kCompCastSlot / 8];
        uint8_t* r = cast(c);
        if (r && *reinterpret_cast<int*>(*reinterpret_cast<uint8_t**>(r + kRadarDef) + kDefType) == kRadarType) radars.push_back(r);
    }
    g_walks++;
    g_components_walked += components.size();
}

void maybe_log()
{
    ULONGLONG now = GetTickCount64();
    if (!g_window) {
        g_window = now;
        g_window_tsc = __rdtsc();
        return;
    }
    if (now - g_window < 5000) return;
    double secs = (now - g_window) / 1000.0;
    double cyc_per_ms = double(__rdtsc() - g_window_tsc) / (secs * 1000.0);
    swlog("radar msgs 5s: %.0f messages/s (%.1f per tick, %.0f%% repeat a vehicle already flagged that tick, %.0f%% skipped), %.0f%% cache hits,"
          " %.0f walks/s of %.0f components avg, %.3f ms/s, %llu verified, %zu vehicles cached%s",
          g_msgs / secs, g_batches ? double(g_msgs) / g_batches : 0.0, g_msgs ? 100.0 * g_repeat / g_msgs : 0.0,
          g_msgs ? 100.0 * g_skipped / g_msgs : 0.0, g_msgs ? 100.0 * g_hits / g_msgs : 0.0, g_walks / secs, g_walks ? double(g_components_walked) / g_walks : 0.0,
          g_cycles / cyc_per_ms / secs, static_cast<unsigned long long>(g_verified), g_cache.size(), g_cache_off ? " (cache off)" : "");
    g_skipped = 0;
    g_msgs = g_hits = g_walks = g_components_walked = g_cycles = g_batches = g_repeat = 0;
    g_window = now;
    g_window_tsc = __rdtsc();
}

void __fastcall radar_message(uint8_t* game, uint8_t* msg)
{
    uint64_t t0 = __rdtsc();
    g_msgs++;
    uint8_t* vehicle = g_find(game + game_vehicles(),*reinterpret_cast<int*>(msg + kMsgVehicleId));
    if (!vehicle || !g_valid(vehicle)) return;
    uint8_t* bodies = *reinterpret_cast<uint8_t**>(vehicle + kVehicleBodies);
    if (!bodies) return;
    if (!g_batch_seen.insert(vehicle).second) g_repeat++;

    read_shape(bodies, g_shape);
    if (!g_cache_off) {
        auto it = g_cache.find(vehicle);
        if (it != g_cache.end() && it->second.batch == g_batch_id && it->second.shape == g_shape) {
            g_skipped++; // flagged earlier in this batch, same components: setting 1 again changes nothing
            g_cycles += __rdtsc() - t0;
            return;
        }
    }
    read_components(bodies, g_scratch);
    std::vector<uint8_t*>* radars = nullptr;
    if (!g_cache_off) {
        auto it = g_cache.find(vehicle);
        if (it != g_cache.end() && it->second.components == g_scratch) {
            Cached& c = it->second;
            if (g_verify_every > 0 && ++c.hits % g_verify_every == 0) {
                walk(g_scratch, g_scratch_radars);
                g_verified++;
                if (g_scratch_radars != c.radars) {
                    swlog("radar msgs: cached radar list of a vehicle differs from a fresh walk (%zu vs %zu); cache off",
                          c.radars.size(), g_scratch_radars.size());
                    g_cache_off = true;
                    g_cache.clear();
                    radars = &g_scratch_radars;
                }
            } else {
                g_hits++;
            }
            if (!radars) radars = &c.radars;
        } else {
            Cached& c = g_cache[vehicle];
            c.components = g_scratch;
            walk(g_scratch, c.radars);
            c.hits = 0;
            radars = &c.radars;
        }
    } else {
        walk(g_scratch, g_scratch_radars);
        radars = &g_scratch_radars;
    }
    for (uint8_t* r : *radars) r[kRadarFlag] = 1;
    if (!g_cache_off) {
        auto it = g_cache.find(vehicle);
        if (it != g_cache.end()) {
            it->second.shape = g_shape;
            it->second.batch = g_batch_id;
        }
    }
    g_cycles += __rdtsc() - t0;
}

// One message batch per call: drop cache entries of vehicles that are gone (pointer reuse) once a batch.
void __fastcall apply_detour(void* a, void* b, void* c, void* d, void* e, void* f)
{
    g_batch_seen.clear();
    g_batches++;
    g_batch_id++;
    o_apply(a, b, c, d, e, f);
    if (g_cache.size() > 4096) g_cache.clear();
    maybe_log();
}

uint8_t* alloc_above(uintptr_t base, size_t size)
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    uintptr_t gran = si.dwAllocationGranularity;
    for (uintptr_t d = 0x10000000; d < 0xF0000000; d += gran) {
        uintptr_t a = (base + d) & ~(gran - 1);
        if (void* p = VirtualAlloc(reinterpret_cast<void*>(a), size, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE))
            return static_cast<uint8_t*>(p);
    }
    return nullptr;
}

bool install()
{
    g_on = GetPrivateProfileIntW(L"radar", L"cache_messages", 1, settings_ini()) != 0;
    g_verify_every = static_cast<int>(GetPrivateProfileIntW(L"radar", L"verify_every", 64, settings_ini()));
    if (!g_on) {
        swlog("radar msgs: off in settings.ini");
        return true;
    }
    uintptr_t base = game_base();
    // translate the table, then index it (only the table's start is in the port table)
    // build 0x6ae218ec inserted a message type before it: the radar message is 0x95 there
    const int type = per_build(kType, kType + 1);
    auto entry = reinterpret_cast<uint32_t*>(game_ptr<uint8_t*>(kJumpTable) + (type - 1) * 4);
    if (*entry != static_cast<uint32_t>(game_live(kCase94) - base)) {
        swlog("radar msgs: jump table entry for 0x94 is 0x%x, not the expected case; off", *entry);
        return true;
    }
    uint8_t* stub = alloc_above(base, 4096);
    if (!stub || reinterpret_cast<uintptr_t>(stub) - base > 0xFFFFFF00ull) {
        swlog("radar msgs: no memory within 4 GB above the game; off");
        return true;
    }
    g_find = game_ptr<Find_t>(kFindVehicle);
    g_valid = game_ptr<Valid_t>(kVehicleValid);
    if (!hook_game(kApplyMessages, reinterpret_cast<void*>(&apply_detour), reinterpret_cast<void**>(&o_apply))) return false;

    uint8_t* p = stub;
    auto emit = [&](std::initializer_list<uint8_t> bytes) {
        for (uint8_t x : bytes) *p++ = x;
    };
    auto emit64 = [&](uint64_t v) {
        memcpy(p, &v, 8);
        p += 8;
    };
    emit({0x4C, 0x89, 0xE9});
    emit({0x48, 0x89, 0xC2});
    emit({0x48, 0x83, 0xEC, 0x20});
    emit({0x48, 0xB8});
    emit64(reinterpret_cast<uint64_t>(&radar_message));
    emit({0xFF, 0xD0});
    emit({0x48, 0x83, 0xC4, 0x20});
    emit({0x48, 0xB8});
    emit64(reinterpret_cast<uint64_t>(game_ptr(kCaseDone)));
    emit({0xFF, 0xE0});
    FlushInstructionCache(GetCurrentProcess(), stub, p - stub);

    DWORD old;
    if (!VirtualProtect(entry, 4, PAGE_EXECUTE_READWRITE, &old)) return false;
    *entry = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(stub) - base);
    VirtualProtect(entry, 4, old, &old);
    FlushInstructionCache(GetCurrentProcess(), entry, 4);
    swlog("radar msgs: tick message 0x94 handled with a per-vehicle radar list (verify every %d hits; \"radar msgs 5s\" lines)",
          g_verify_every);
    return true;
}

HookRegistrar reg{{"radar_msg", /*needs_known_build=*/true, &install}};

} // namespace
