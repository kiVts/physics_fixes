
#include "parallel.h"

#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "game.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

constexpr uintptr_t kQuery = 0x1401a4b20, kRingGrow = 0x1408de0b0;
constexpr size_t kSceneWorld = 0x20, kWorldBroadphase = 0x78, kBpSet0 = 0x08, kBpSet1 = 0x48;
constexpr size_t kNodeMin = 0x00, kNodeMax = 0x20, kNodeChild0 = 0x48, kNodeChild1 = 0x50; // leaf: child1 == 0, data = child0
constexpr size_t kProxyObject = 0x0, kProxyGroup = 0x8, kProxyMask = 0xc, kObjUser = 0x1f8;

struct Ring { uint64_t* data; uint32_t cap, head, count; };

using Query_t = bool(__fastcall*)(void* scene, const double* pos, float radius, Ring* out, int group, int mask);
using Grow_t = void(__fastcall*)(Ring*, uint32_t new_cap);
Query_t g_orig = nullptr;
Grow_t g_grow = nullptr;
int g_mode = 1;

std::atomic<uint64_t> g_calls{0}, g_same{0}, g_order{0}, g_diff{0}, g_failed{0};
std::atomic<int> g_diff_logged{0};
ULONGLONG g_last_log = 0;

template <class T> T rd(uintptr_t a) { return *reinterpret_cast<const T*>(a); }

bool overlaps(uintptr_t node, const double* mn, const double* mx)
{
    const double* nmn = reinterpret_cast<const double*>(node + kNodeMin);
    const double* nmx = reinterpret_cast<const double*>(node + kNodeMax);
    return nmn[0] <= mx[0] && nmx[0] >= mn[0] && nmn[1] <= mx[1] && nmx[1] >= mn[1] && nmn[2] <= mx[2] && nmx[2] >= mn[2];
}

constexpr int kMaxObjects = 2048;

// The overlapping objects' user pointers, in the original's order (no C++ objects here: SEH); false if the layout
// looks wrong or there are too many.
bool walk(void* scene, const double* pos, float radius, int group, int mask, uint64_t* users, int& n_users)
{
    n_users = 0;
    __try {
        uintptr_t world = rd<uintptr_t>(reinterpret_cast<uintptr_t>(scene) + kSceneWorld);
        if (!world) return false;
        uintptr_t bp = rd<uintptr_t>(world + kWorldBroadphase);
        if (!bp) return false;
        double r = radius, mn[3] = {pos[0] - r, pos[1] - r, pos[2] - r}, mx[3] = {pos[0] + r, pos[1] + r, pos[2] + r};
        static thread_local uintptr_t objects[kMaxObjects]; // once each, in pairing order
        int n_objects = 0;
        uintptr_t stack[256];
        for (size_t set : {kBpSet0, kBpSet1}) {
            uintptr_t root = rd<uintptr_t>(bp + set);
            if (!root) continue;
            int n = 0;
            stack[n++] = root;
            while (n) {
                uintptr_t node = stack[--n];
                if (!overlaps(node, mn, mx)) continue;
                uintptr_t c1 = rd<uintptr_t>(node + kNodeChild1);
                if (c1) {
                    if (n + 2 > 256) return false;
                    stack[n++] = rd<uintptr_t>(node + kNodeChild0);
                    stack[n++] = c1;
                    continue;
                }
                uintptr_t proxy = rd<uintptr_t>(node + kNodeChild0);
                if (!proxy) continue;
                int g = rd<int>(proxy + kProxyGroup), m = rd<int>(proxy + kProxyMask);
                if (!(group & m) || !(g & mask)) continue;
                uintptr_t obj = rd<uintptr_t>(proxy + kProxyObject);
                if (!obj || std::find(objects, objects + n_objects, obj) != objects + n_objects) continue;
                if (n_objects == kMaxObjects) return false;
                objects[n_objects++] = obj;
            }
        }
        for (int i = 0; i < n_objects; i++)
            if (uint64_t user = rd<uint64_t>(objects[i] + kObjUser)) users[n_users++] = user;
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool query(void* scene, const double* pos, float radius, int group, int mask, std::vector<uint64_t>& out)
{
    static thread_local uint64_t users[kMaxObjects];
    int n = 0;
    world_read_begin(); // the step may be running (early launch)
    bool ok = walk(scene, pos, radius, group, mask, users, n);
    world_read_end();
    out.assign(users, users + (ok ? n : 0));
    return ok;
}

void push(Ring* ring, uint64_t v)
{
    if (ring->count == ring->cap) g_grow(ring, ring->cap ? ring->cap * 2 : 16);
    ring->count++;
    ring->data[(ring->head + ring->count - 1) % ring->cap] = v;
}

void log_maybe()
{
    ULONGLONG now = GetTickCount64();
    if (!g_last_log) g_last_log = now;
    if (now - g_last_log < 30000) return;
    g_last_log = now;
    swlog("overlap query 30s: %llu calls, %llu same, %llu same set in another order, %llu different, %llu not answered",
          (unsigned long long)g_calls.load(), (unsigned long long)g_same.load(), (unsigned long long)g_order.load(),
          (unsigned long long)g_diff.load(), (unsigned long long)g_failed.load());
}

bool __fastcall query_detour(void* scene, const double* pos, float radius, Ring* out, int group, int mask)
{
    g_calls.fetch_add(1, std::memory_order_relaxed);
    std::vector<uint64_t> ours;
    bool ok = query(scene, pos, radius, group, mask, ours);
    if (g_mode == 2 && ok) {
        for (uint64_t v : ours) push(out, v);
        log_maybe();
        return out->count != 0;
    }
    if (!ok) g_failed.fetch_add(1, std::memory_order_relaxed);
    uint32_t before = out->count;
    bool r = g_orig(scene, pos, radius, out, group, mask);
    if (ok) {
        std::vector<uint64_t> theirs;
        for (uint32_t i = before; i < out->count; i++) theirs.push_back(out->data[(out->head + i) % out->cap]);
        if (theirs == ours) g_same.fetch_add(1, std::memory_order_relaxed);
        else {
            std::vector<uint64_t> a = theirs, b = ours;
            std::sort(a.begin(), a.end());
            std::sort(b.begin(), b.end());
            if (a == b) g_order.fetch_add(1, std::memory_order_relaxed);
            else {
                g_diff.fetch_add(1, std::memory_order_relaxed);
                if (g_diff_logged.fetch_add(1) < 10)
                    swlog("overlap query: differs at (%.1f, %.1f, %.1f) r %.2f group %#x mask %#x: game %zu objects, ours %zu",
                          pos[0], pos[1], pos[2], radius, group, mask, theirs.size(), ours.size());
            }
        }
    }
    log_maybe();
    return r;
}

bool install()
{
    g_mode = static_cast<int>(GetPrivateProfileIntW(L"overlap_query", L"mode", 1, settings_ini()));
    if (g_mode <= 0) return true;
    g_grow = game_ptr<Grow_t>(kRingGrow);
    if (!g_grow || !hook_game(kQuery, reinterpret_cast<void*>(&query_detour), reinterpret_cast<void**>(&g_orig))) return false;
    swlog("overlap query: %s (sphere overlap from the broadphase trees, read only)",
          g_mode == 2 ? "on" : "check (the game's answer is kept, ours compared)");
    return true;
}

HookRegistrar reg{{"overlap query", /*needs_known_build=*/true, &install}};

} // namespace
