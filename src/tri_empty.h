// "No triangles here" cache for btBvhTriangleMeshShape::processAllTriangles (tri_empty.cpp): the policy, with the
// walk and the game memory behind callables so the test can drive it.
//
// A walk over the mesh's BVH with box B reaches every leaf whose (quantized) box overlaps B, and calls the callback
// once per leaf. Quantization (min floored, max ceiled) and the overlap test only grow with B, so a box inside a box E
// that reached no leaf reaches none either. When a walk with a callback that filters triangles by its own box comes
// back empty, it is run with that box grown by a margin instead; if that reached nothing too, the grown box E is kept
// for the callback (one per pair, in mesh space, so it holds however the mesh object is placed). A later call whose
// box lies inside its E skips the walk: the callback would not have been called. Every 64th skip walks anyway and must
// find nothing, else the cache is switched off. A pair whose grown walk found triangles (it touches the mesh) waits
// kCooldown calls before it is tried again, so pairs in contact don't pay for the bigger walk every time.
#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace tri_empty {

constexpr int kCooldown = 16;

struct Box {
    double lo[3], hi[3];
    bool contains(const double* mn, const double* mx) const
    {
        for (int k = 0; k < 3; k++)
            if (!(mn[k] >= lo[k] && mx[k] <= hi[k])) return false; // NaN: not contained
        return true;
    }
};

struct Entry {
    const void* shape = nullptr;
    uint64_t gen = 0;
    bool empty = false; // box reached no leaf
    Box box{};
    int cooldown = 0;   // calls to walk normally before trying a grown box again
};

// One entry per callback (= per convex-concave pair), striped locks: the narrowphase walks from several threads.
class Cache {
public:
    static constexpr int kStripes = 64;
    // Runs f(Entry*) under the stripe's lock; the entry is null when there is none for this shape and generation.
    template <typename F> auto with(const void* cb, const void* shape, uint64_t gen, F&& f)
    {
        Stripe& s = stripe(cb);
        std::lock_guard lock(s.mu);
        auto it = s.map.find(cb);
        Entry* e = it != s.map.end() && it->second.shape == shape && it->second.gen == gen ? &it->second : nullptr;
        return f(e);
    }
    void put(const void* cb, const Entry& e)
    {
        Stripe& s = stripe(cb);
        std::lock_guard lock(s.mu);
        s.map[cb] = e;
    }
    size_t size()
    {
        size_t n = 0;
        for (auto& s : stripes_) {
            std::lock_guard lock(s.mu);
            n += s.map.size();
        }
        return n;
    }
    void prune(uint64_t gen) // entries of older generations can't match any more
    {
        for (auto& s : stripes_) {
            std::lock_guard lock(s.mu);
            for (auto it = s.map.begin(); it != s.map.end();) it = it->second.gen != gen ? s.map.erase(it) : ++it;
        }
    }

private:
    struct Stripe {
        std::mutex mu;
        std::unordered_map<const void*, Entry> map;
    };
    Stripe& stripe(const void* cb) { return stripes_[(reinterpret_cast<uintptr_t>(cb) >> 4) % kStripes]; }
    std::array<Stripe, kStripes> stripes_;
};

enum class Outcome { kSkipped, kWalked, kStored, kChecked, kMismatch, kTouching };

// One processAllTriangles call. walk(lo, hi) runs the game's walk with that box (the real callback still gets every
// leaf it reaches) and returns the number of leaves. inflatable: the callback drops triangles outside its own box, and
// mn / mx are that box. check: a skip that is due for its self-check.
template <typename Walk>
Outcome process(Cache& cache, const void* cb, const void* shape, uint64_t gen, const double* mn, const double* mx, bool inflatable,
                double margin, bool check, Walk&& walk)
{
    if (!inflatable) {
        walk(mn, mx);
        return Outcome::kWalked;
    }
    enum { kHit, kWait, kTry } what = cache.with(cb, shape, gen, [&](Entry* e) {
        if (e && e->empty && e->box.contains(mn, mx)) return kHit;
        if (e && e->cooldown > 0) {
            e->cooldown--;
            return kWait;
        }
        return kTry;
    });
    if (what == kHit) {
        if (!check) return Outcome::kSkipped;
        return walk(mn, mx) == 0 ? Outcome::kChecked : Outcome::kMismatch;
    }
    if (what == kWait) {
        walk(mn, mx);
        return Outcome::kWalked;
    }
    Entry e;
    e.shape = shape;
    e.gen = gen;
    alignas(16) double glo[4] = {}, ghi[4] = {};
    for (int k = 0; k < 3; k++) glo[k] = mn[k] - margin, ghi[k] = mx[k] + margin;
    for (int k = 0; k < 3; k++) e.box.lo[k] = glo[k], e.box.hi[k] = ghi[k];
    if (walk(glo, ghi) == 0) {
        e.empty = true;
        cache.put(cb, e);
        return Outcome::kStored;
    }
    e.empty = false;
    e.cooldown = kCooldown;
    cache.put(cb, e);
    return Outcome::kTouching;
}

} // namespace tri_empty
