#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace body_contain {

struct Body {
    const void* p;
    uint32_t id;
    double point[3];
    double centre[3]; // b's sphere centre, as test() computes it
    double reach;     // 2 * b's radius, plus a margin
};

constexpr double kCell = 8.0;  // m
constexpr int kMaxSpan = 4;    // cells per axis; bigger spheres go on the list every query checks

inline int64_t cell_of(double v) { return static_cast<int64_t>(std::floor(v / kCell)); }
inline uint64_t cell_key(int64_t x, int64_t y, int64_t z)
{
    return (static_cast<uint64_t>(x & 0x1fffff) << 42) | (static_cast<uint64_t>(y & 0x1fffff) << 21) | static_cast<uint64_t>(z & 0x1fffff);
}

struct Grid {
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells; // body indices, ascending
    std::vector<uint32_t> big;                                  // ascending
    std::vector<uint32_t> scratch;

    void build(const std::vector<Body>& bodies)
    {
        cells.clear(); // keeps its buckets; cells bodies left would otherwise pile up (and be walked) for the session
        big.clear();
        for (uint32_t i = 0; i < bodies.size(); i++) {
            const Body& b = bodies[i];
            if (!(b.reach >= 0) || !std::isfinite(b.reach) || !std::isfinite(b.centre[0]) || !std::isfinite(b.centre[1]) ||
                !std::isfinite(b.centre[2])) {
                big.push_back(i); // can't place it: always a candidate
                continue;
            }
            int64_t lo[3], hi[3];
            bool too_big = false;
            for (int k = 0; k < 3; k++) {
                lo[k] = cell_of(b.centre[k] - b.reach);
                hi[k] = cell_of(b.centre[k] + b.reach);
                if (hi[k] - lo[k] + 1 > kMaxSpan) too_big = true;
            }
            if (too_big) {
                big.push_back(i);
                continue;
            }
            for (int64_t x = lo[0]; x <= hi[0]; x++)
                for (int64_t y = lo[1]; y <= hi[1]; y++)
                    for (int64_t z = lo[2]; z <= hi[2]; z++) cells[cell_key(x, y, z)].push_back(i);
        }
    }

    // Candidates for a point, ascending (= list order).
    const std::vector<uint32_t>& query(const double p[3])
    {
        scratch.clear();
        const std::vector<uint32_t>* c = nullptr;
        if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
            auto it = cells.find(cell_key(cell_of(p[0]), cell_of(p[1]), cell_of(p[2])));
            if (it != cells.end()) c = &it->second;
        } else {
            // a point the grid can't place: every body is a candidate (as in the full walk)
            return scratch;
        }
        if (!c || c->empty()) {
            scratch = big;
            return scratch;
        }
        scratch.resize(c->size() + big.size());
        std::merge(c->begin(), c->end(), big.begin(), big.end(), scratch.begin());
        return scratch;
    }
};

// One pass. test(b, a, &value, &cell) -> hit; take(a, b, value, cell) records a hit (the last call per a is its
// result); clear(a) runs first for every a. Returns the number of test() calls.
template <typename Test, typename Take, typename Clear>
uint64_t run(const std::vector<Body>& bodies, Grid& grid, Test&& test, Take&& take, Clear&& clear)
{
    grid.build(bodies);
    uint64_t tests = 0;
    const uint32_t n = static_cast<uint32_t>(bodies.size());
    for (uint32_t ai = 0; ai < n; ai++) {
        const Body& a = bodies[ai];
        clear(ai);
        uint32_t best = 0xffffffffu;
        const bool placeable = std::isfinite(a.point[0]) && std::isfinite(a.point[1]) && std::isfinite(a.point[2]);
        auto try_b = [&](uint32_t bi) {
            if (bi == ai) return;
            const Body& b = bodies[bi];
            if (b.id <= a.id || b.id >= best) return;
            float value = 0;
            uint32_t cell = 0;
            tests++;
            if (test(bi, ai, &value, &cell)) {
                take(ai, bi, value, cell);
                best = b.id;
            }
        };
        if (!placeable) {
            for (uint32_t bi = 0; bi < n; bi++) try_b(bi);
            continue;
        }
        for (uint32_t bi : grid.query(a.point)) try_b(bi);
    }
    return tests;
}

} // namespace body_contain
