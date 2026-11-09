
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "phys_pool.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace {

constexpr uintptr_t kUpdateAabbs = 0x1400da740, kSetAabb = 0x1400ccce0, kCollideTT = 0x1400ce8c0;
constexpr size_t kWorldBroadphase = 0x78, kWorldForceAll = 0x88;
constexpr size_t kBpSet0 = 0x08, kBpSet1 = 0x48, kBpPairCache = 0xa0, kBpPrediction = 0xa8, kBpNewPairs = 0xc0, kBpDeferred = 0xe5;
constexpr size_t kProxyLeaf = 0x60, kProxyStage = 0x78;
constexpr size_t kNodeData = 0x48, kNodeChild1 = 0x50;
constexpr int kStageFixed = 2;

using UpdateAabbs_t = void(__fastcall*)(void* world);
using SetAabb_t = void(__fastcall*)(void* bp, void* proxy, const double* aabb_min, const double* aabb_max, void* dispatcher);
using AddPair_t = void*(__fastcall*)(void* cache, void* a, void* b);

UpdateAabbs_t o_update_aabbs = nullptr;
SetAabb_t o_set_aabb = nullptr;

bool g_on = true, g_broken = false;
int g_verify_every = 64;
uint64_t g_verified = 0;
int g_threads = 1;
double g_prediction = -1;
bool g_prediction_logged = false;

bool g_collecting = false;      // inside our updateAabbs on the physics thread
std::vector<uint8_t*> g_moved;  // proxies whose search was deferred, in the game's order

// --- read-only collideTTpersistentStack ---------------------------------------------------------------------------
struct Node {
    double min[4], max[4];
    Node* parent;
    Node* c0; // childs[0] / data
    Node* c1; // childs[1]; 0 for a leaf
};
static_assert(offsetof(Node, c0) == kNodeData && offsetof(Node, c1) == kNodeChild1, "btDbvtNode layout");
inline bool internal(const Node* n) { return n->c1 != nullptr; }
inline bool intersect(const Node* a, const Node* b)
{
    return a->min[0] <= b->max[0] && b->min[0] <= a->max[0] && a->min[1] <= b->max[1] && b->min[1] <= a->max[1] &&
           a->min[2] <= b->max[2] && b->min[2] <= a->max[2];
}

struct Found {
    void *a, *b;
};

// Pushes found pairs to out, in the game's order.
void search(const Node* root, const Node* leaf, std::vector<std::pair<const Node*, const Node*>>& stack, std::vector<Found>& out)
{
    if (!root || !leaf) return;
    stack.clear();
    stack.push_back({root, leaf});
    while (!stack.empty()) {
        auto [a, b] = stack.back();
        stack.pop_back();
        if (a == b) {
            if (internal(a)) {
                stack.push_back({a->c0, a->c0});
                stack.push_back({a->c1, a->c1});
                stack.push_back({a->c0, a->c1});
            }
        } else if (intersect(a, b)) {
            if (internal(a)) {
                if (internal(b)) {
                    stack.push_back({a->c0, b->c0});
                    stack.push_back({a->c1, b->c0});
                    stack.push_back({a->c0, b->c1});
                    stack.push_back({a->c1, b->c1});
                } else {
                    stack.push_back({a->c0, b});
                    stack.push_back({a->c1, b});
                }
            } else if (internal(b)) {
                stack.push_back({a, b->c0});
                stack.push_back({a, b->c1});
            } else {
                out.push_back({a->c0, b->c0});
            }
        }
    }
}

// --- workers ----------------------------------------------------------------------------------------------------------
struct Slot { // one per thread
    std::vector<Found> found;
    std::vector<std::pair<const Node*, const Node*>> stack;
};
struct Result { // one per moved proxy
    uint32_t slot, begin, count;
};

Slot g_slots[16];
std::vector<Result> g_results;
const Node *g_root0 = nullptr, *g_root1 = nullptr;
std::atomic<int> g_next{0};
constexpr int kChunk = 32;

void run_searches(int slot)
{
    Slot& s = g_slots[slot];
    int n = static_cast<int>(g_moved.size());
    for (;;) {
        int start = g_next.fetch_add(kChunk, std::memory_order_relaxed);
        if (start >= n) return;
        int end = std::min(start + kChunk, n);
        for (int i = start; i < end; i++) {
            auto leaf = *reinterpret_cast<const Node* const*>(g_moved[i] + kProxyLeaf);
            uint32_t begin = static_cast<uint32_t>(s.found.size());
            search(g_root1, leaf, s.stack, s.found);
            search(g_root0, leaf, s.stack, s.found);
            g_results[i] = {static_cast<uint32_t>(slot), begin, static_cast<uint32_t>(s.found.size()) - begin};
        }
    }
}

void run_searches_job(int slot, void*) { run_searches(slot); }

using CollideTT_t = void(__fastcall*)(void* tree, const void* root, const void* leaf, void** collider);
std::vector<Found> g_check;
void* __fastcall record_pair(void*, void* a, void* b)
{
    g_check.push_back({a, b});
    return nullptr;
}
void* g_fake_vtable[4] = {nullptr, reinterpret_cast<void*>(&record_pair), nullptr, nullptr};
struct FakeCache {
    void** vtable = g_fake_vtable;
} g_fake_cache;
alignas(16) uint8_t g_fake_bp[0x100];

bool verify(uint8_t* bp)
{
    auto collide = game_ptr<CollideTT_t>(kCollideTT);
    *reinterpret_cast<void**>(g_fake_bp + kBpPairCache) = &g_fake_cache;
    void* collider = g_fake_bp;
    size_t n = std::min<size_t>(g_moved.size(), 16);
    for (size_t i = 0; i < n; i++) {
        auto leaf = *reinterpret_cast<const void* const*>(g_moved[i] + kProxyLeaf);
        g_check.clear();
        collide(bp + kBpSet1, *reinterpret_cast<void* const*>(bp + kBpSet1), leaf, &collider);
        collide(bp + kBpSet0, *reinterpret_cast<void* const*>(bp + kBpSet0), leaf, &collider);
        const Result& r = g_results[i];
        const Found* f = g_slots[r.slot].found.data() + r.begin;
        bool same = g_check.size() == r.count;
        for (uint32_t k = 0; same && k < r.count; k++) same = g_check[k].a == f[k].a && g_check[k].b == f[k].b;
        if (!same) {
            swlog("broadphase mt: search of moved object %zu differs from the game's (%zu vs %u pairs); off", i, g_check.size(), r.count);
            return false;
        }
    }
    g_verified++;
    return true;
}

// --- stats --------------------------------------------------------------------------------------------------------------
bool g_aabbs_on = true, g_aabbs_broken = false;
uint64_t g_aabb_steps = 0, g_aabb_objs = 0, g_aabb_checked = 0, g_cyc_boxes = 0, g_cyc_apply = 0;
uint64_t g_steps = 0, g_moved_total = 0, g_pairs_total = 0, g_cyc_update = 0, g_cyc_search = 0, g_cyc_add = 0;
ULONGLONG g_window = 0;
uint64_t g_window_tsc = 0;

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
    double cpm = double(__rdtsc() - g_window_tsc) / (secs * 1000.0);
    double ss = g_steps ? double(g_steps) : 1.0;
    swlog("broadphase mt 5s: %.0f sub-steps/s, %llu verified, %.0f moved proxies and %.0f pairs reported per sub-step; per sub-step:"
          " leaf updates %.3f ms, searches %.3f ms (%d threads), adding pairs %.3f ms",
          g_steps / secs, static_cast<unsigned long long>(g_verified), g_moved_total / ss, g_pairs_total / ss, g_cyc_update / cpm / ss, g_cyc_search / cpm / ss, g_threads,
          g_cyc_add / cpm / ss);
    if (g_aabb_steps)
        swlog("broadphase mt 5s: AABBs %s: %.0f objects per sub-step, boxes %.3f ms (pool) + setAabb in order %.3f ms per sub-step, "
              "%llu self-checks ok",
              g_aabbs_broken ? "OFF after a self-check" : "on the pool", double(g_aabb_objs) / g_aabb_steps, g_cyc_boxes / cpm / g_aabb_steps,
              g_cyc_apply / cpm / g_aabb_steps, static_cast<unsigned long long>(g_aabb_checked));
    g_aabb_steps = g_aabb_objs = g_aabb_checked = g_cyc_boxes = g_cyc_apply = 0;
    g_steps = g_moved_total = g_pairs_total = g_cyc_update = g_cyc_search = g_cyc_add = 0;
    g_window = now;
    g_window_tsc = __rdtsc();
}

constexpr uintptr_t kSingleAabb = 0x1400da450, kProfStart = 0x140163440, kProfStop = 0x140163360, kUpdateAabbsName = 0x140ae0c08;
constexpr size_t kWorldObjCount = 0xc, kWorldObjs = 0x18, kWorldDispatcher = 0x28, kWorldInterp = 0x48;
constexpr size_t kObjXform = 0x10, kObjInterpXform = 0x90, kObjProxy = 0x180, kObjShape = 0x188, kObjFlags = 0x1a0,
                 kObjActivation = 0x1b0, kObjType = 0x1f0;
using GetAabb_t = void(__fastcall*)(const void* shape, const void* xform, double* mn, double* mx);
using SingleAabb_t = void(__fastcall*)(void* world, void* obj);
using ProfStart_t = void(__fastcall*)(const char*);
using ProfStop_t = void(__fastcall*)();

struct ObjBox {
    alignas(16) double mn[4]; // btVector3s: setAabb and getAabb use movaps
    alignas(16) double mx[4];
    uint8_t* obj;
};
std::vector<ObjBox> g_boxes;
uint8_t* g_box_world = nullptr;
std::atomic<int> g_box_next{0};

void box_of(const uint8_t* world, ObjBox& b)
{
    const uint8_t* obj = b.obj;
    const void* shape = *reinterpret_cast<void* const*>(obj + kObjShape);
    auto get = (*reinterpret_cast<GetAabb_t* const*>(shape))[1];
    get(shape, obj + kObjXform, b.mn, b.mx);
    b.mx[0] = b.mx[0] + 0.02, b.mx[1] = b.mx[1] + 0.02, b.mx[2] = b.mx[2] + 0.02;
    b.mn[1] = b.mn[1] + -0.02, b.mn[2] = b.mn[2] + -0.02, b.mn[0] = b.mn[0] + -0.02;
    if (world[kWorldInterp] != 0 && *reinterpret_cast<const int*>(obj + kObjType) == 2 && (obj[kObjFlags] & 3) == 0) {
        alignas(16) double m2[4], x2[4];
        get(shape, obj + kObjInterpXform, m2, x2);
        if (m2[0] - 0.02 < b.mn[0]) b.mn[0] = m2[0] - 0.02;
        if (m2[1] - 0.02 < b.mn[1]) b.mn[1] = m2[1] - 0.02;
        if (m2[2] - 0.02 < b.mn[2]) b.mn[2] = m2[2] - 0.02;
        if (m2[3] < b.mn[3]) b.mn[3] = m2[3];
        if (b.mx[0] < x2[0] + 0.02) b.mx[0] = x2[0] + 0.02;
        if (b.mx[1] < x2[1] + 0.02) b.mx[1] = x2[1] + 0.02;
        if (b.mx[2] < x2[2] + 0.02) b.mx[2] = x2[2] + 0.02;
        if (b.mx[3] < x2[3]) b.mx[3] = x2[3];
    }
}

void boxes_job(int, void*)
{
    int n = static_cast<int>(g_boxes.size());
    for (;;) {
        int start = g_box_next.fetch_add(64, std::memory_order_relaxed);
        if (start >= n) return;
        int end = std::min(start + 64, n);
        for (int i = start; i < end; i++) box_of(g_box_world, g_boxes[i]);
    }
}

void update_aabbs_parallel(uint8_t* world)
{
    game_ptr<ProfStart_t>(kProfStart)(reinterpret_cast<const char*>(game_ptr(kUpdateAabbsName)));
    uint64_t t0 = __rdtsc();
    int count = *reinterpret_cast<const int*>(world + kWorldObjCount);
    auto objs = *reinterpret_cast<uint8_t* const* const*>(world + kWorldObjs);
    bool force = world[kWorldForceAll] != 0;
    g_boxes.clear();
    for (int i = 0; i < count; i++) {
        uint8_t* obj = objs[i];
        int act = *reinterpret_cast<const int*>(obj + kObjActivation);
        if (!force && (act == 2 || act == 5)) continue;
        g_boxes.push_back(ObjBox{{}, {}, obj});
    }
    g_box_world = world;
    g_box_next.store(0, std::memory_order_relaxed);
    phys_pool_run(g_boxes.size() >= 256 ? g_threads - 1 : 0, &boxes_job, nullptr);
    uint64_t t1 = __rdtsc();
    bool check = g_verify_every > 0 && (g_aabb_steps % g_verify_every) == 0;
    if (check) {
        for (ObjBox& b : g_boxes) {
            ObjBox again{{}, {}, b.obj};
            box_of(world, again);
            if (memcmp(again.mn, b.mn, sizeof b.mn) != 0 || memcmp(again.mx, b.mx, sizeof b.mx) != 0) {
                swlog("broadphase mt: box of object %p computed on the pool differs from the one computed on the physics thread "
                      "(%.6f %.6f %.6f .. %.6f %.6f %.6f vs %.6f %.6f %.6f .. %.6f %.6f %.6f); parallel AABBs off",
                      b.obj, b.mn[0], b.mn[1], b.mn[2], b.mx[0], b.mx[1], b.mx[2], again.mn[0], again.mn[1], again.mn[2], again.mx[0],
                      again.mx[1], again.mx[2]);
                g_aabbs_broken = true;
                b = again; // the physics thread's own box for the rest of this step
            }
        }
        g_aabb_checked++;
    }
    auto bp = *reinterpret_cast<uint8_t**>(world + kWorldBroadphase);
    auto set = reinterpret_cast<SetAabb_t>((*reinterpret_cast<void* const* const*>(bp))[3]);
    void* dispatcher = *reinterpret_cast<void**>(world + kWorldDispatcher);
    auto single = game_ptr<SingleAabb_t>(kSingleAabb);
    constexpr size_t kAhead = 8;
    size_t nb = g_boxes.size();
    for (size_t k = 0; k < nb && k < kAhead; k++) {
        auto p = *reinterpret_cast<const char* const*>(g_boxes[k].obj + kObjProxy);
        _mm_prefetch(p, _MM_HINT_T0), _mm_prefetch(p + 64, _MM_HINT_T0);
    }
    for (size_t k = 0; k < nb; k++) {
        ObjBox& b = g_boxes[k];
        if (k + kAhead < nb) {
            auto p = *reinterpret_cast<const char* const*>(g_boxes[k + kAhead].obj + kObjProxy);
            _mm_prefetch(p, _MM_HINT_T0), _mm_prefetch(p + 64, _MM_HINT_T0);
        }
        double dx = b.mx[0] - b.mn[0], dy = b.mx[1] - b.mn[1], dz = b.mx[2] - b.mn[2];
        if ((b.obj[kObjFlags] & 1) == 0 && 1000000000000.0 <= dy * dy + dx * dx + dz * dz) {
            single(world, b.obj); // the game's overflow handling (it computes the same box again)
            continue;
        }
        set(bp, *reinterpret_cast<void**>(b.obj + kObjProxy), b.mn, b.mx, dispatcher);
    }
    uint64_t t2 = __rdtsc();
    game_ptr<ProfStop_t>(kProfStop)();
    g_aabb_steps++;
    g_aabb_objs += g_boxes.size();
    g_cyc_boxes += t1 - t0, g_cyc_apply += t2 - t1;
}

// --- detours ---------------------------------------------------------------------------------------------------------------
void __fastcall set_aabb_detour(void* bp, void* proxy_v, const double* mn, const double* mx, void* dispatcher)
{
    if (!g_collecting) return o_set_aabb(bp, proxy_v, mn, mx, dispatcher);
    auto proxy = static_cast<uint8_t*>(proxy_v);
    auto leaf = *reinterpret_cast<uint8_t**>(proxy + kProxyLeaf);
    int stage = *reinterpret_cast<int*>(proxy + kProxyStage);
    double before[8];
    if (leaf) memcpy(before, leaf, sizeof before);
    o_set_aabb(bp, proxy_v, mn, mx, dispatcher);
    auto leaf_after = *reinterpret_cast<uint8_t**>(proxy + kProxyLeaf);
    if (stage == kStageFixed || leaf_after != leaf || !leaf || memcmp(before, leaf_after, sizeof before) != 0) g_moved.push_back(proxy);
}

bool is_dbvt(uint8_t* bp)
{
    __try {
        auto vtable = *reinterpret_cast<uintptr_t* const*>(bp);
        return vtable && vtable[3] == reinterpret_cast<uintptr_t>(game_ptr(kSetAabb));
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void __fastcall update_aabbs_detour(void* world_v)
{
    auto world = static_cast<uint8_t*>(world_v);
    auto bp = *reinterpret_cast<uint8_t**>(world + kWorldBroadphase);
    if (!g_on || g_broken || GetCurrentThreadId() != world_physics_thread() || !bp || bp[kBpDeferred] || !is_dbvt(bp))
        return o_update_aabbs(world_v);

    double& pred = *reinterpret_cast<double*>(bp + kBpPrediction);
    if (!g_prediction_logged) {
        g_prediction_logged = true;
        swlog("broadphase mt: game's leaf prediction %.3f%s", pred, g_prediction >= 0 ? "" : " (kept)");
        if (g_prediction >= 0) swlog("broadphase mt: leaf prediction set to %.3f", g_prediction);
    }
    if (g_prediction >= 0) pred = g_prediction;

    uint64_t t0 = __rdtsc();
    g_moved.clear();
    g_collecting = true;
    bp[kBpDeferred] = 1;
    if (g_aabbs_on && !g_aabbs_broken) update_aabbs_parallel(world);
    else o_update_aabbs(world_v);
    bp[kBpDeferred] = 0;
    g_collecting = false;
    uint64_t t1 = __rdtsc();

    g_root0 = *reinterpret_cast<const Node* const*>(bp + kBpSet0);
    g_root1 = *reinterpret_cast<const Node* const*>(bp + kBpSet1);
    g_results.assign(g_moved.size(), Result{});
    for (int s = 0; s < g_threads; s++) g_slots[s].found.clear();
    g_next.store(0, std::memory_order_relaxed);
    phys_pool_run(g_moved.size() >= 64 ? g_threads - 1 : 0, &run_searches_job, nullptr);
    uint64_t t2 = __rdtsc();
    if (g_verify_every > 0 && (g_steps % g_verify_every) == 0 && !g_moved.empty() && !verify(bp)) {
        g_broken = true; // ours can't be trusted: add this step's pairs with the game's own search, then stay off
        for (uint8_t* proxy : g_moved) {
            auto leaf = *reinterpret_cast<const void* const*>(proxy + kProxyLeaf);
            void* real = bp;
            auto collide = game_ptr<CollideTT_t>(kCollideTT);
            collide(bp + kBpSet1, *reinterpret_cast<void* const*>(bp + kBpSet1), leaf, &real);
            collide(bp + kBpSet0, *reinterpret_cast<void* const*>(bp + kBpSet0), leaf, &real);
        }
        return;
    }

    auto cache = *reinterpret_cast<void**>(bp + kBpPairCache);
    auto add = reinterpret_cast<AddPair_t>((*reinterpret_cast<void***>(cache))[1]);
    int& newpairs = *reinterpret_cast<int*>(bp + kBpNewPairs);
    uint64_t pairs = 0;
    for (const Result& r : g_results) {
        const Found* f = g_slots[r.slot].found.data() + r.begin;
        for (uint32_t k = 0; k < r.count; k++) add(cache, f[k].a, f[k].b);
        newpairs += static_cast<int>(r.count);
        pairs += r.count;
    }
    uint64_t t3 = __rdtsc();
    g_steps++;
    g_moved_total += g_moved.size();
    g_pairs_total += pairs;
    g_cyc_update += t1 - t0;
    g_cyc_search += t2 - t1;
    g_cyc_add += t3 - t2;
    maybe_log();
}

bool install()
{
    const wchar_t* ini = settings_ini();
    g_on = GetPrivateProfileIntW(L"broadphase", L"parallel_pairs", 1, ini) != 0;
    g_verify_every = static_cast<int>(GetPrivateProfileIntW(L"broadphase", L"verify_every", 64, ini));
    g_aabbs_on = GetPrivateProfileIntW(L"broadphase", L"parallel_aabbs", 1, ini) != 0;
    {
        wchar_t buf[32];
        GetPrivateProfileStringW(L"broadphase", L"prediction", L"0.25", buf, 32, ini);
        g_prediction = _wtof(buf);
    }
    int threads = static_cast<int>(GetPrivateProfileIntW(L"broadphase", L"threads", 0, ini));
    int hw = static_cast<int>(std::thread::hardware_concurrency());
    if (threads <= 0) threads = std::min(hw - 2, 6);
    g_threads = std::max(1, std::min(threads, phys_pool_threads()));
    if (!g_on) {
        swlog("broadphase mt: off in settings.ini");
        return true;
    }
    if (!hook_game(kSetAabb, reinterpret_cast<void*>(&set_aabb_detour), reinterpret_cast<void**>(&o_set_aabb)) ||
        !hook_game(kUpdateAabbs, reinterpret_cast<void*>(&update_aabbs_detour), reinterpret_cast<void**>(&o_update_aabbs)))
        return false;
    swlog("broadphase mt: pair searches of moved objects on %d threads (\"broadphase mt 5s\" lines)", g_threads);
    return true;
}

HookRegistrar reg{{"broadphase_mt", /*needs_known_build=*/true, &install}};

} // namespace
