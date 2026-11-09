
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "mt_dispatch.h"
#include "world.h"

#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr uintptr_t kTreeWalk = 0x1400e5990, kLeafProcess = 0x1400e4d60;
constexpr uintptr_t kCompoundCtor = 0x1400fa100, kCompoundAddChild = 0x1400fa390;
constexpr size_t kCbWrap0 = 0x08, kCbWrap1 = 0x10, kWrapShape = 0x08, kShapeTree = 0x80, kShapeRevision = 0x88, kDbvtLeaves = 0x14;

std::atomic<uint64_t> g_gen{1}; // compound constructions + addChildShape calls

using Ctor_t = void*(__fastcall*)(void*, uintptr_t, uintptr_t, uintptr_t);
using AddChild_t = void(__fastcall*)(void*, const void*, void*);
Ctor_t g_ctor = nullptr;
AddChild_t g_add_child = nullptr;
void* __fastcall ctor_detour(void* self, uintptr_t a, uintptr_t b, uintptr_t c)
{
    g_gen.fetch_add(1);
    return g_ctor(self, a, b, c);
}
void __fastcall add_child_detour(void* self, const void* xform, void* shape)
{
    g_gen.fetch_add(1);
    g_add_child(self, xform, shape);
}

struct ShapeState {
    const void* shape;
    const void* tree;
    const void* root;
    int revision, leaves;
    bool operator==(const ShapeState& o) const
    {
        return shape == o.shape && tree == o.tree && root == o.root && revision == o.revision && leaves == o.leaves;
    }
};

bool shape_state_raw(const uint8_t* cb, size_t wrap_off, ShapeState& st)
{
    __try {
        const uint8_t* wrap = *reinterpret_cast<const uint8_t* const*>(cb + wrap_off);
        const uint8_t* shape = *reinterpret_cast<const uint8_t* const*>(wrap + kWrapShape);
        const uint8_t* tree = *reinterpret_cast<const uint8_t* const*>(shape + kShapeTree);
        st.shape = shape;
        st.tree = tree;
        st.root = tree ? *reinterpret_cast<const void* const*>(tree) : nullptr;
        st.revision = *reinterpret_cast<const int*>(shape + kShapeRevision);
        st.leaves = tree ? *reinterpret_cast<const int*>(tree + kDbvtLeaves) : -1;
        return tree != nullptr;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

struct Node {
    double mi[4], mx[4];
    const Node* parent;
    const Node* child[2];
};
bool is_leaf(const Node* n) { return n->child[1] == nullptr; }

using Walk_t = void(__fastcall*)(const Node* root0, const Node* root1, const double* xform, void* callback, double threshold);
using Process_t = void(__fastcall*)(void* callback, const Node* leaf0, const Node* leaf1);
Walk_t g_walk = nullptr;
Process_t g_process = nullptr;

std::atomic<bool> g_on{true};
double g_margin = 0.05;

// --- recording the leaf pairs a walk reaches -----------------------------------------------------------
// per thread: pairs may run on several threads at once (mt_dispatch.cpp)
thread_local bool g_recording = false;
thread_local DWORD g_record_thread = 0;
thread_local std::vector<std::pair<const Node*, const Node*>> g_rec;

void __fastcall process_detour(void* callback, const Node* l0, const Node* l1)
{
    if (g_recording && GetCurrentThreadId() == g_record_thread) {
        g_rec.emplace_back(l0, l1);
        return;
    }
    g_process(callback, l0, l1);
}

void record_walk(const Node* r0, const Node* r1, const double* xform, void* cb, double threshold)
{
    g_rec.clear();
    g_record_thread = GetCurrentThreadId();
    g_recording = true;
    g_walk(r0, r1, xform, cb, threshold);
    g_recording = false;
}

// --- MycollideTT's overlap test (btTransformAabb of b by xform, widened by the threshold) --------------
// xform: btTransform, 3 basis rows of 4 doubles then the origin (doubles 12..14).
bool overlap(const Node* a, const Node* b, const double* m, double d)
{
    double hx = (b->mx[0] - b->mi[0]) * 0.5 + 0.0, hy = (b->mx[1] - b->mi[1]) * 0.5 + 0.0, hz = (b->mx[2] - b->mi[2]) * 0.5 + 0.0;
    double cx = (b->mi[0] + b->mx[0]) * 0.5, cy = (b->mi[1] + b->mx[1]) * 0.5, cz = (b->mi[2] + b->mx[2]) * 0.5;
    double ex = hx * std::fabs(m[0]) + hy * std::fabs(m[1]) + hz * std::fabs(m[2]);
    double ey = hx * std::fabs(m[4]) + hy * std::fabs(m[5]) + hz * std::fabs(m[6]);
    double ez = hx * std::fabs(m[8]) + hy * std::fabs(m[9]) + hz * std::fabs(m[10]);
    double px = cx * m[0] + cy * m[1] + cz * m[2] + m[12];
    double py = cx * m[4] + cy * m[5] + cz * m[6] + m[13];
    double pz = cx * m[8] + cy * m[9] + cz * m[10] + m[14];
    return a->mi[0] <= px + ex + d && px - ex - d <= a->mx[0] && a->mi[1] <= py + ey + d && py - ey - d <= a->mx[1] &&
           a->mi[2] <= pz + ez + d && pz - ez - d <= a->mx[2];
}

// --- tree identity: hash of every node's address + AABB (backstop check) ----------------------------------
struct TreeInfo {
    uint64_t hash = 0;
    int leaves = 0;
    double radius = 0; // farthest corner of the root AABB from the compound's origin
    bool ok = false;
};
thread_local std::vector<const Node*> g_stack;

uint64_t mix(uint64_t h, uint64_t v)
{
    h ^= v + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
    return h * 0xff51afd7ed558ccdull;
}

bool hash_tree_raw(const Node* root, TreeInfo& t)
{
    __try {
        uint64_t h = 0;
        int leaves = 0, visited = 0;
        g_stack.clear();
        g_stack.push_back(root);
        while (!g_stack.empty()) {
            const Node* n = g_stack.back();
            g_stack.pop_back();
            if (++visited > 2000000) return false;
            h = mix(h, reinterpret_cast<uintptr_t>(n));
            uint64_t w[6];
            memcpy(&w[0], &n->mi[0], 24);
            memcpy(&w[3], &n->mx[0], 24);
            for (uint64_t x : w) h = mix(h, x);
            if (is_leaf(n)) {
                leaves++;
            } else {
                g_stack.push_back(n->child[0]);
                g_stack.push_back(n->child[1]);
            }
        }
        double r2 = 0;
        for (int c = 0; c < 8; c++) {
            double x = (c & 1) ? root->mx[0] : root->mi[0], y = (c & 2) ? root->mx[1] : root->mi[1], z = (c & 4) ? root->mx[2] : root->mi[2];
            r2 = std::fmax(r2, x * x + y * y + z * z);
        }
        t.hash = h;
        t.leaves = leaves;
        t.radius = std::sqrt(r2);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

TreeInfo tree_info(const Node* root)
{
    TreeInfo t;
    t.ok = hash_tree_raw(root, t);
    return t;
}

// --- the cache ---------------------------------------------------------------------------------------------
struct Key {
    const Node *r0, *r1;
    bool operator==(const Key& o) const { return r0 == o.r0 && r1 == o.r1; }
};
struct KeyHash {
    size_t operator()(const Key& k) const { return std::hash<uintptr_t>()(reinterpret_cast<uintptr_t>(k.r0) * 31 ^ reinterpret_cast<uintptr_t>(k.r1)); }
};
struct Entry {
    uint64_t hash0, hash1, last_step, built_step, checked_step, gen;
    ShapeState s0, s1;
    double radius1;
    double xform[16];
    double threshold;
    std::vector<std::pair<const Node*, const Node*>> pairs;
};
std::unordered_map<Key, Entry, KeyHash> g_cache; // entry references stay valid across inserts
std::mutex g_cache_mu;                           // find / insert / erase (one pair = one thread per step)

// stats (5 s windows)
std::atomic<uint64_t> g_calls{0}, g_hits{0}, g_rebuilds_move{0}, g_rebuilds_tree{0}, g_rebuilds_new{0}, g_verified{0}, g_mismatch{0};
std::atomic<uint64_t> g_hit_cycles{0}, g_miss_cycles{0}, g_cached_pairs{0}, g_processed{0}, g_backstop_changes{0};
ULONGLONG g_window = 0;
uint64_t g_window_tsc = 0, g_last_prune = 0;

double movement_bound(const double* a, const double* b, double radius)
{
    double db2 = 0;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            double d = a[r * 4 + c] - b[r * 4 + c];
            db2 += d * d;
        }
    double dx = a[12] - b[12], dy = a[13] - b[13], dz = a[14] - b[14];
    return std::sqrt(dx * dx + dy * dy + dz * dz) + 2.0 * radius * std::sqrt(db2);
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
    uint64_t rebuilds = g_rebuilds_move.load() + g_rebuilds_tree.load() + g_rebuilds_new.load();
    swlog("compound cache 5s: %.0f walks/s, %.1f%% replayed from cache (%.2f us each), rebuilds/s: %.0f moved %.0f changed %.0f new"
          " (%.2f us each); %.1f cached leaf pairs per replay, %.1f processed; self-check %llu ok / %llu different; %zu entries;"
          " backstop caught %llu",
          g_calls.load() / secs, g_calls ? 100.0 * g_hits.load() / g_calls.load() : 0.0, g_hits ? g_hit_cycles.load() / cyc_per_ms * 1000.0 / g_hits.load() : 0.0,
          g_rebuilds_move.load() / secs, g_rebuilds_tree.load() / secs, g_rebuilds_new.load() / secs,
          rebuilds ? g_miss_cycles.load() / cyc_per_ms * 1000.0 / rebuilds : 0.0, g_hits ? double(g_cached_pairs.load()) / g_hits.load() : 0.0,
          g_calls ? double(g_processed.load()) / g_calls.load() : 0.0, static_cast<unsigned long long>(g_verified.load() - g_mismatch.load()),
          static_cast<unsigned long long>(g_mismatch.load()), g_cache.size(), static_cast<unsigned long long>(g_backstop_changes.load()));
    for (auto* c : {&g_calls, &g_hits, &g_rebuilds_move, &g_rebuilds_tree, &g_rebuilds_new, &g_verified, &g_mismatch, &g_hit_cycles,
                    &g_miss_cycles, &g_cached_pairs, &g_processed, &g_backstop_changes})
        c->store(0);
    g_window = now;
    g_window_tsc = __rdtsc();
}

void prune(uint64_t step)
{
    if (step - g_last_prune < 300) return;
    g_last_prune = step;
    for (auto it = g_cache.begin(); it != g_cache.end();) it = step - it->second.last_step > 300 ? g_cache.erase(it) : ++it;
}

void __fastcall walk_detour(const Node* r0, const Node* r1, const double* xform, void* cb, double threshold)
{
    if (!g_on || !r0 || !r1 || (GetCurrentThreadId() != world_physics_thread() && !mt_is_worker()) || g_recording) {
        g_walk(r0, r1, xform, cb, threshold);
        return;
    }
    uint64_t t0 = __rdtsc();
    uint64_t step = world_step_id();
    uint64_t call_no = ++g_calls;
    ShapeState s0{}, s1{};
    if (!shape_state_raw(static_cast<const uint8_t*>(cb), kCbWrap0, s0) || !shape_state_raw(static_cast<const uint8_t*>(cb), kCbWrap1, s1) ||
        s0.root != r0 || s1.root != r1) {
        g_walk(r0, r1, xform, cb, threshold); // not the layout we expect: leave it alone
        return;
    }
    uint64_t gen = g_gen.load();
    Entry* ep;
    {
        std::lock_guard<std::mutex> lock(g_cache_mu);
        ep = &g_cache[Key{r0, r1}];
    }
    Entry& e = *ep;
    bool fresh = e.built_step == 0;
    bool changed = !fresh && (e.gen != gen || !(e.s0 == s0) || !(e.s1 == s1) || e.threshold != threshold);
    if (!fresh && !changed && step - e.checked_step >= 120) {
        // backstop: full hash of both trees
        TreeInfo ta = tree_info(r0), tb = tree_info(r1);
        e.checked_step = step;
        if (!ta.ok || !tb.ok || ta.hash != e.hash0 || tb.hash != e.hash1) {
            changed = true;
            g_backstop_changes++;
        }
    }
    bool moved = !fresh && !changed && movement_bound(xform, e.xform, e.radius1) > g_margin;
    bool hit = !fresh && !changed && !moved;
    if (!hit) {
        TreeInfo ta = tree_info(r0), tb = tree_info(r1);
        if (!ta.ok || !tb.ok) {
            e.built_step = 0; // rebuilt next time
            g_walk(r0, r1, xform, cb, threshold);
            return;
        }
        record_walk(r0, r1, xform, cb, threshold + g_margin);
        e.pairs = g_rec;
        e.hash0 = ta.hash;
        e.hash1 = tb.hash;
        e.radius1 = tb.radius;
        e.s0 = s0;
        e.s1 = s1;
        e.gen = gen;
        e.threshold = threshold;
        e.built_step = e.checked_step = step;
        memcpy(e.xform, xform, sizeof e.xform);
        if (fresh) g_rebuilds_new++;
        else if (changed) g_rebuilds_tree++;
        else g_rebuilds_move++;
    }
    e.last_step = step;

    // Self-check on every 64th call: the original walk's pairs must equal the filtered cached ones.
    if ((call_no & 63) == 0) {
        record_walk(r0, r1, xform, cb, threshold);
        std::vector<std::pair<const Node*, const Node*>> truth = g_rec;
        size_t k = 0;
        bool same = true;
        for (auto& p : e.pairs) {
            if (!overlap(p.first, p.second, xform, threshold)) continue;
            if (k >= truth.size() || truth[k] != p) {
                same = false;
                break;
            }
            k++;
        }
        same = same && k == truth.size();
        g_verified++;
        if (!same) {
            g_mismatch++;
            g_on = false;
            swlog("compound cache: self-check FAILED (cached %zu pairs, walk %zu); cache switched off for this session",
                  e.pairs.size(), truth.size());
            for (auto& p : truth) g_process(cb, p.first, p.second);
            g_processed += truth.size();
            return;
        }
    }

    size_t processed = 0;
    for (auto& p : e.pairs) {
        if (overlap(p.first, p.second, xform, threshold)) {
            g_process(cb, p.first, p.second);
            processed++;
        }
    }
    g_processed += processed;
    uint64_t cycles = __rdtsc() - t0;
    if (hit) {
        g_hits++;
        g_hit_cycles += cycles;
        g_cached_pairs += e.pairs.size();
    } else {
        g_miss_cycles += cycles;
    }
    if (!mt_dispatch_active()) {
        prune(step);
        maybe_log();
    }
}

void after_parallel() // physics thread, no pairs running
{
    prune(world_step_id());
    maybe_log();
}

bool install()
{
    std::wstring ini = settings_ini();
    g_on.store(GetPrivateProfileIntW(L"compound", L"cache", 1, ini.c_str()) != 0);
    wchar_t buf[32];
    GetPrivateProfileStringW(L"compound", L"margin", L"0.05", buf, 32, ini.c_str());
    g_margin = _wtof(buf);
    if (!(g_margin > 0.0 && g_margin < 5.0)) g_margin = 0.05;
    if (!g_on) {
        swlog("compound cache: off in settings.ini");
        return true;
    }
    if (!hook_game(kCompoundCtor, reinterpret_cast<void*>(&ctor_detour), reinterpret_cast<void**>(&g_ctor))) return false;
    if (!hook_game(kCompoundAddChild, reinterpret_cast<void*>(&add_child_detour), reinterpret_cast<void**>(&g_add_child))) return false;
    if (!hook_game(kLeafProcess, reinterpret_cast<void*>(&process_detour), reinterpret_cast<void**>(&g_process))) return false;
    if (!hook_game(kTreeWalk, reinterpret_cast<void*>(&walk_detour), reinterpret_cast<void**>(&g_walk))) return false;
    mt_set_after_parallel(&after_parallel);
    swlog("compound cache: on, margin %.3f m (\"compound cache 5s\" lines; self-check every 64th walk)", g_margin);
    return true;
}

HookRegistrar reg{{"compound_cache", /*needs_known_build=*/true, &install}};

} // namespace
