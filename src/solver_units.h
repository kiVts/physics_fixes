// Parallel solver units (solver_log.cpp): merges units that reference the same dynamic body.
//
// The units are built from the islands Bullet's island manager hands to processIsland, and islands share no dynamic
// body, so neither should the units. On a client in multiplayer they did on nearly every step (friend's client
// 2026-10-27: 200+ bodies per step, each listed by two islands), and every such step went back to the game's
// single-threaded path. Instead the units that share a body are merged into one (union-find over units), and each
// merged unit keeps every body, manifold and constraint once: Bullet's setup applies a body's forces once per entry
// in the body list, so a body listed twice in one solveGroup would get gravity twice. Units without a shared body
// keep running in parallel.
#pragma once

#include <cstddef>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace solver_units {

// Accessors the merge needs (game memory in solver_log.cpp, fakes in the test).
struct Access {
    bool (*is_dynamic)(const void* obj);
    const void* (*man_body0)(const void* manifold);
    const void* (*man_body1)(const void* manifold);
    const void* (*con_a)(const void* constraint); // null constraint: both return null
    const void* (*con_b)(const void* constraint);
};

struct MergeStats {
    size_t shared_bodies = 0; // references to a body another unit already had
    size_t merged_units = 0;  // units folded into another
};

inline int uf_find(std::vector<int>& p, int i)
{
    while (p[i] != i) i = p[i] = p[p[i]];
    return i;
}

template <typename T> void dedupe(std::vector<T>& v, std::unordered_set<T>& seen)
{
    seen.clear();
    size_t out = 0;
    for (size_t i = 0; i < v.size(); i++)
        if (seen.insert(v[i]).second) v[out++] = v[i];
    v.resize(out);
}

// WorkT has vectors bodies / manifolds / constraints (void*), int islands and uint64_t cost. Merges in place: the
// surviving units stay in `order` (in their first position), folded ones are removed.
template <typename WorkT> MergeStats merge_shared(std::vector<WorkT*>& order, const Access& a)
{
    MergeStats st;
    const int n = static_cast<int>(order.size());
    if (n < 2) return st;
    std::vector<int> parent(n);
    for (int i = 0; i < n; i++) parent[i] = i;
    std::unordered_map<const void*, int> owner;
    size_t refs = 0;
    for (const WorkT* w : order) refs += w->bodies.size() + 2 * (w->manifolds.size() + w->constraints.size());
    owner.reserve(refs);
    auto mark = [&](const void* obj, int wi) {
        if (!obj || !a.is_dynamic(obj)) return;
        auto [it, fresh] = owner.emplace(obj, wi);
        if (fresh) return;
        int ra = uf_find(parent, it->second), rb = uf_find(parent, wi);
        if (ra == rb) return;
        st.shared_bodies++;
        if (ra < rb) parent[rb] = ra;
        else parent[ra] = rb;
    };
    for (int wi = 0; wi < n; wi++) {
        const WorkT* w = order[wi];
        for (void* b : w->bodies) mark(b, wi);
        for (void* m : w->manifolds) {
            mark(a.man_body0(m), wi);
            mark(a.man_body1(m), wi);
        }
        for (void* c : w->constraints) {
            if (!c) continue;
            mark(a.con_a(c), wi);
            mark(a.con_b(c), wi);
        }
    }
    if (!st.shared_bodies) return st;

    std::vector<bool> touched(n, false);
    for (int wi = 0; wi < n; wi++) {
        int r = uf_find(parent, wi);
        if (r == wi) continue;
        WorkT* dst = order[r];
        WorkT* src = order[wi];
        dst->bodies.insert(dst->bodies.end(), src->bodies.begin(), src->bodies.end());
        dst->manifolds.insert(dst->manifolds.end(), src->manifolds.begin(), src->manifolds.end());
        dst->constraints.insert(dst->constraints.end(), src->constraints.begin(), src->constraints.end());
        dst->islands += src->islands;
        dst->cost += src->cost;
        touched[r] = true;
        st.merged_units++;
    }
    std::unordered_set<void*> seen;
    size_t out = 0;
    for (int wi = 0; wi < n; wi++) {
        if (uf_find(parent, wi) != wi) continue;
        if (touched[wi]) {
            dedupe(order[wi]->bodies, seen);
            dedupe(order[wi]->manifolds, seen);
            dedupe(order[wi]->constraints, seen);
        }
        order[out++] = order[wi];
    }
    order.resize(out);
    return st;
}

} // namespace solver_units
