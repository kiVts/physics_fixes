
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr uintptr_t kTest = 0x14048e810;
using Test_t = uint8_t(__fastcall*)(const uint8_t* shape, const double* point, float* value, uint32_t* cell);
Test_t g_test = nullptr;

enum Kind { kResultRdx, kResultR13, kFoundJump };
struct Site {
    uintptr_t at;    // cmp edi, r15d; je  (41 3B FF 74 xx)
    Kind kind;
    bool ring_rsi;   // ring register: rsi (else rbx)
    int point_base;  // 0 = rbp, 1 = rsp
    int point_off;
    int value_base, value_off, cell_base, cell_off;
    int found_off;   // kFoundJump: where the loop goes on a hit, from `at`
};
const Site kSites[] = {
    {0x1405334a0, kResultRdx, false, 0, -0x79, 1, 0x60, 0, 0x17, 0},
    {0x140533584, kResultRdx, false, 0, -0x79, 1, 0x60, 0, 0x17, 0},
    {0x140533670, kResultRdx, false, 0, -0x79, 1, 0x60, 0, 0x17, 0},
    {0x140533735, kResultR13, false, 0, -0x79, 1, 0x60, 0, 0x17, 0},
    {0x1404f5115, kFoundJump, true, 1, 0x50, 0, 0x78, 0, 0x90, 0x3a},
};
constexpr int kNSites = sizeof kSites / sizeof kSites[0];
constexpr int kCheckEvery = 64;
constexpr double kCell = 16.0, kMargin = 2.0;

std::atomic<bool> g_on{false};
std::atomic<uint32_t> g_tick{1};
std::atomic<uint64_t> g_searches{0}, g_tests{0}, g_bodies_walked{0}, g_checks{0}, g_moved{0}, g_builds{0}, g_cycles{0};

uint8_t* slot(uintptr_t base, int off) { return reinterpret_cast<uint8_t*>(base + off); }
template <typename T> T rd(const void* p, size_t off)
{
    T v;
    memcpy(&v, static_cast<const uint8_t*>(p) + off, sizeof v);
    return v;
}

struct Ring {
    const uint8_t* const* data;
    uint32_t cap, head, count;
};
Ring ring_of(const uint8_t* r) { return {rd<const uint8_t* const*>(r, 0), rd<uint32_t>(r, 8), rd<uint32_t>(r, 0xc), rd<uint32_t>(r, 0x10)}; }
const uint8_t* body_at(const Ring& r, uint32_t i) { return r.data[(r.head + i) % r.cap]; }

void sphere(const uint8_t* q, double c[3], double& reach)
{
    double l0 = rd<double>(q, 0x148), l1 = rd<double>(q, 0x150), l2 = rd<double>(q, 0x158);
    for (int k = 0; k < 3; k++)
        c[k] = l0 * rd<double>(q, 0x508 + 8 * k) + l1 * rd<double>(q, 0x528 + 8 * k) + l2 * rd<double>(q, 0x548 + 8 * k) +
               rd<double>(q, 0x568 + 8 * k);
    reach = 2.0 * std::fabs(double(rd<float>(q, 0x160))) + kMargin;
}

int64_t cell_of(double v) { return static_cast<int64_t>(std::floor(v / kCell)); }
uint64_t key(int64_t x, int64_t y, int64_t z)
{
    return (uint64_t(x & 0x1fffff) << 42) | (uint64_t(y & 0x1fffff) << 21) | uint64_t(z & 0x1fffff);
}

struct Grid {
    const uint8_t* ring = nullptr;
    const void* data = nullptr;
    uint32_t head = 0, count = 0, tick = 0;
    std::vector<std::pair<uint64_t, uint32_t>> cells; // sorted
    std::vector<uint32_t> always; // bodies the grid can't place: candidates for every point
};
constexpr int kGridsPerThread = 4;
struct ThreadGrids {
    Grid g[kGridsPerThread];
    int next = 0;
    std::vector<uint32_t> out;
    uint64_t searches = 0;
};
thread_local ThreadGrids* tl_grids = nullptr;

void build(Grid& g, const uint8_t* ring, const Ring& r, uint32_t tick)
{
    g_builds.fetch_add(1, std::memory_order_relaxed);
    g.ring = ring, g.data = r.data, g.head = r.head, g.count = r.count, g.tick = tick;
    g.cells.clear();
    g.always.clear();
    for (uint32_t i = 0; i < r.count; i++) {
        const uint8_t* q = body_at(r, i);
        double c[3], reach;
        if (!q) { g.always.push_back(i); continue; }
        sphere(q, c, reach);
        if (!std::isfinite(c[0] + c[1] + c[2] + reach)) { g.always.push_back(i); continue; }
        int64_t lo[3], hi[3];
        bool big = false;
        for (int k = 0; k < 3; k++) {
            lo[k] = cell_of(c[k] - reach), hi[k] = cell_of(c[k] + reach);
            if (hi[k] - lo[k] > 6) big = true;
        }
        if (big) { g.always.push_back(i); continue; }
        for (int64_t x = lo[0]; x <= hi[0]; x++)
            for (int64_t y = lo[1]; y <= hi[1]; y++)
                for (int64_t z = lo[2]; z <= hi[2]; z++) g.cells.push_back({key(x, y, z), i});
    }
    std::sort(g.cells.begin(), g.cells.end());
}

// Candidates in [start, end), ascending.
const std::vector<uint32_t>& candidates(const Grid& g, std::vector<uint32_t>& out, const double* p, uint32_t start, uint32_t end)
{
    out.clear();
    if (!std::isfinite(p[0] + p[1] + p[2])) {
        for (uint32_t i = start; i < end; i++) out.push_back(i);
        return out;
    }
    uint64_t k = key(cell_of(p[0]), cell_of(p[1]), cell_of(p[2]));
    for (auto it = std::lower_bound(g.cells.begin(), g.cells.end(), std::make_pair(k, 0u)); it != g.cells.end() && it->first == k; ++it)
        if (it->second >= start && it->second < end) out.push_back(it->second);
    if (!g.always.empty()) {
        for (uint32_t i : g.always) if (i >= start && i < end) out.push_back(i);
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
    }
    return out;
}

uint64_t entry_of(const uint8_t* q, uint32_t cell)
{
    uint32_t idx = (rd<uint32_t>(q, 0x854) + cell) % rd<uint32_t>(q, 0x850);
    return rd<uint64_t>(rd<const uint8_t*>(q, 0x848), size_t(idx) * 8);
}

// The game's walk: first hit in [start, end); -1 if none.
int64_t full_walk(const Ring& r, uint32_t start, uint32_t end, const double* p, float* value, uint32_t* cell)
{
    for (uint32_t i = start; i < end; i++)
        if (g_test(body_at(r, i), p, value, cell)) return i;
    return -1;
}

void disable(const char* why)
{
    if (g_on.exchange(false)) swlog("contain walk: %s; every body is walked again for the session", why);
}

} // namespace

extern "C" uint64_t __fastcall cw_search(int site, const uint8_t* ring, uintptr_t rbp, uintptr_t rsp)
{
    const Site& s = kSites[site];
    uint32_t start = *reinterpret_cast<uint32_t*>(rsp + 0x28), end = *reinterpret_cast<uint32_t*>(rsp + 0x2c);
    const double* p = reinterpret_cast<const double*>(slot(s.point_base ? rsp : rbp, s.point_off));
    float* value = reinterpret_cast<float*>(slot(s.value_base ? rsp : rbp, s.value_off));
    uint32_t* cell = reinterpret_cast<uint32_t*>(slot(s.cell_base ? rsp : rbp, s.cell_off));
    Ring r = ring_of(ring);
    auto set_regs = [&](int64_t hit) {
        if (hit >= 0) *reinterpret_cast<const uint8_t**>(rsp + 0x20) = body_at(r, uint32_t(hit));
        else if (end > start) *reinterpret_cast<const uint8_t**>(rsp + 0x20) = body_at(r, end - 1);
        *reinterpret_cast<uint32_t*>(rsp + 0x28) = hit >= 0 ? uint32_t(hit) : (end > start ? end : start);
        return hit >= 0 ? entry_of(body_at(r, uint32_t(hit)), *cell) : 0ull;
    };
    if (end <= start || !r.data || !r.cap) return set_regs(-1);
    if (!g_on.load(std::memory_order_relaxed) || end > r.count) return set_regs(full_walk(r, start, end, p, value, cell));

    uint64_t t0 = __rdtsc();
    ThreadGrids* tg = tl_grids ? tl_grids : (tl_grids = new ThreadGrids());
    uint32_t tick = g_tick.load(std::memory_order_relaxed);
    Grid* g = nullptr;
    for (Grid& x : tg->g)
        if (x.ring == ring) g = &x;
    if (!g) {
        g = &tg->g[tg->next];
        tg->next = (tg->next + 1) % kGridsPerThread;
    }
    if (g->ring != ring || g->data != r.data || g->head != r.head || g->count != r.count || g->tick != tick) build(*g, ring, r, tick);
    struct Cyc {
        uint64_t t0;
        ~Cyc() { g_cycles.fetch_add(__rdtsc() - t0, std::memory_order_relaxed); }
    } cyc{t0};
    g_searches.fetch_add(1, std::memory_order_relaxed);
    g_bodies_walked.fetch_add(end - start, std::memory_order_relaxed);

    int64_t hit = -1;
    uint64_t tested = 0;
    for (uint32_t i : candidates(*g, tg->out, p, start, end)) {
        tested++;
        if (g_test(body_at(r, i), p, value, cell)) {
            hit = i;
            break;
        }
    }
    g_tests.fetch_add(tested, std::memory_order_relaxed);
    if (++tg->searches % kCheckEvery == 0) {
        g_checks.fetch_add(1, std::memory_order_relaxed);
        float v2;
        uint32_t c2;
        int64_t full = full_walk(r, start, end, p, &v2, &c2);
        if (full != hit) {
            build(*g, ring, r, tick); // bodies read again
            int64_t again = -1;
            for (uint32_t i : candidates(*g, tg->out, p, start, end))
                if (g_test(body_at(r, i), p, &v2, &c2)) { again = i; break; }
            int64_t full2 = full_walk(r, start, end, p, &v2, &c2);
            if (again == full2) g_moved.fetch_add(1, std::memory_order_relaxed);
            else {
                char why[160];
                snprintf(why, sizeof why, "the grid found body #%lld, the full walk #%lld (site %d, %u bodies)",
                         static_cast<long long>(again), static_cast<long long>(full2), site, end - start);
                disable(why);
            }
            // answer as the game would, from the full walk
            if (full2 >= 0) g_test(body_at(r, uint32_t(full2)), p, value, cell);
            return set_regs(full2);
        }
    }
    return set_regs(hit);
}

namespace {

void put64(uint8_t*& p, uint64_t v) { memcpy(p, &v, 8), p += 8; }
void emit(uint8_t*& p, std::initializer_list<uint8_t> b) { for (uint8_t x : b) *p++ = x; }

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

// One stub per site. In: the loop's registers (edi = start index, r15d = end, ring in rbx / rsi, rbp, rsp 16-aligned
// as at the loop's own call). Out: as the loop leaves them.
uint8_t* make_stub(uint8_t* p, int site, uintptr_t at, uintptr_t not_found, uintptr_t found)
{
    const Site& s = kSites[site];
    uint8_t* start = p;
    emit(p, {0x4C, 0x89, 0x74, 0x24, 0x20});
    emit(p, {0x89, 0x7C, 0x24, 0x28});
    emit(p, {0x44, 0x89, 0x7C, 0x24, 0x2C});
    emit(p, {0xB9}), memcpy(p, &site, 4), p += 4;        // mov ecx, site
    if (s.ring_rsi) emit(p, {0x48, 0x89, 0xF2});
    else emit(p, {0x48, 0x89, 0xDA});
    emit(p, {0x49, 0x89, 0xE8});
    emit(p, {0x49, 0x89, 0xE1});                         // mov r9, rsp
    emit(p, {0x48, 0xB8}), put64(p, reinterpret_cast<uint64_t>(&cw_search)); // mov rax, cw_search
    emit(p, {0xFF, 0xD0});                               // call rax
    emit(p, {0x4C, 0x8B, 0x74, 0x24, 0x20});
    emit(p, {0x8B, 0x7C, 0x24, 0x28});
    emit(p, {0x48, 0x85, 0xC0});                         // test rax, rax
    uint8_t* jz = p;
    emit(p, {0x74, 0x00});                               // jz not_found
    if (s.kind == kResultRdx) emit(p, {0x48, 0x89, 0xC2});
    if (s.kind == kResultR13) emit(p, {0x49, 0x89, 0xC5}); // mov r13, rax
    emit(p, {0x49, 0xBB}), put64(p, found);              // mov r11, found
    emit(p, {0x41, 0xFF, 0xE3});                         // jmp r11
    jz[1] = static_cast<uint8_t>(p - (jz + 2));
    emit(p, {0x49, 0xBB}), put64(p, not_found);          // mov r11, not_found (the loop's je target)
    emit(p, {0x41, 0xFF, 0xE3});                         // jmp r11
    (void)start, (void)at;
    return p;
}

DWORD WINAPI log_loop(void*)
{
    uint64_t ls = 0, lt = 0, lw = 0, lc = 0, lm = 0, lb = 0, ly = 0;
    LARGE_INTEGER f, a, b;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&a);
    uint64_t r0 = __rdtsc();
    Sleep(50);
    QueryPerformanceCounter(&b);
    double cyc_per_ms = double(__rdtsc() - r0) / (double(b.QuadPart - a.QuadPart) * 1000.0 / double(f.QuadPart));
    for (;;) {
        Sleep(5000);
        uint64_t s = g_searches, t = g_tests, w = g_bodies_walked, c = g_checks, m = g_moved, bl = g_builds, y = g_cycles;
        if (s == ls) continue;
        swlog("contain walk 5s: %llu searches, %.1f bodies tested each instead of %.1f, %llu grid builds, %.2f ms/s in the "
              "search, %llu self-checks (%llu with a body that moved)%s",
              static_cast<unsigned long long>(s - ls), double(t - lt) / double(s - ls), double(w - lw) / double(s - ls),
              static_cast<unsigned long long>(bl - lb), double(y - ly) / cyc_per_ms / 5.0, static_cast<unsigned long long>(c - lc),
              static_cast<unsigned long long>(m - lm), g_on ? "" : " [off]");
        ls = s, lt = t, lw = w, lc = c, lm = m, lb = bl, ly = y;
    }
}

bool install()
{
    if (!GetPrivateProfileIntW(L"contain", L"walk_grid", 1, settings_ini())) {
        swlog("contain walk: off in settings.ini");
        return true;
    }
    if (!swml()->on_loop(swml_self(), [](void*) { g_tick.fetch_add(1, std::memory_order_relaxed); }, nullptr, nullptr, nullptr)) {
        swlog("contain walk: no game loop events; off");
        return true;
    }
    g_test = game_ptr<Test_t>(kTest);
    // check every site first; patch only if all are as expected
    uintptr_t at[kNSites], nf[kNSites], fd[kNSites];
    for (int i = 0; i < kNSites; i++) {
        at[i] = game_live(kSites[i].at);
        const uint8_t* b = reinterpret_cast<const uint8_t*>(at[i]);
        // cmp edi, r15d; then je rel8 (74 xx) or je rel32 (0F 84 xx xx xx xx)
        bool short_je = at[i] && b[3] == 0x74, near_je = at[i] && b[3] == 0x0F && b[4] == 0x84;
        if (!at[i] || b[0] != 0x41 || b[1] != 0x3B || b[2] != 0xFF || !(short_je || near_je)) {
            swlog("contain walk: loop %d isn't the expected code; off", i);
            return true;
        }
        if (short_je) nf[i] = at[i] + 5 + static_cast<int8_t>(b[4]);
        else {
            int32_t rel;
            memcpy(&rel, b + 5, 4);
            nf[i] = at[i] + 9 + rel;
        }
        const uint8_t* n = reinterpret_cast<const uint8_t*>(nf[i]);
        if (kSites[i].kind == kResultRdx) {
            if (n[0] != 0x49 || n[1] != 0x8B || n[2] != 0xD5) { swlog("contain walk: loop %d's exit isn't mov rdx, r13; off", i); return true; }
            fd[i] = nf[i] + 3;
        } else if (kSites[i].kind == kResultR13) {
            if (n[0] != 0x4D || n[1] != 0x89 || n[2] != 0xAC) { swlog("contain walk: loop %d's exit isn't the r13 store; off", i); return true; }
            fd[i] = nf[i];
        } else {
            fd[i] = at[i] + kSites[i].found_off;
            const uint8_t* f = reinterpret_cast<const uint8_t*>(fd[i]);
            if (f[0] != 0x49 || f[1] != 0x8B || f[2] != 0x86) { swlog("contain walk: loop %d's hit path isn't the expected code; off", i); return true; }
        }
    }
    uint8_t* mem = alloc_near(game_base(), 4096);
    if (!mem) return true;
    uint8_t* p = mem;
    uint8_t* stub[kNSites];
    for (int i = 0; i < kNSites; i++) {
        stub[i] = p;
        p = make_stub(p, i, at[i], nf[i], fd[i]);
        p = reinterpret_cast<uint8_t*>((reinterpret_cast<uintptr_t>(p) + 15) & ~uintptr_t(15));
    }
    FlushInstructionCache(GetCurrentProcess(), mem, p - mem);
    g_on = true;
    for (int i = 0; i < kNSites; i++) {
        int64_t rel = reinterpret_cast<intptr_t>(stub[i]) - static_cast<intptr_t>(at[i] + 5);
        if (rel < INT32_MIN || rel > INT32_MAX) return true;
        uint8_t j[5] = {0xE9};
        int32_t r32 = static_cast<int32_t>(rel);
        memcpy(j + 1, &r32, 4);
        DWORD old;
        VirtualProtect(reinterpret_cast<void*>(at[i]), 5, PAGE_EXECUTE_READWRITE, &old);
        memcpy(reinterpret_cast<void*>(at[i]), j, 5); // mod init: before any world exists
        VirtualProtect(reinterpret_cast<void*>(at[i]), 5, old, &old);
        FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(at[i]), 5);
    }
    CloseHandle(CreateThread(nullptr, 0, log_loop, nullptr, 0, nullptr));
    swlog("contain walk: on (%d object-pass containment walks test only bodies near the point, in list order; "
          "\"contain walk 5s\" lines)", kNSites);
    return true;
}

HookRegistrar reg{{"contain walk", /*needs_known_build=*/true, &install}};

} // namespace
