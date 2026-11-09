
#include "body_contain.h"
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <cstring>

namespace {

constexpr uintptr_t kPatchRun = 0x140549360, kResumeOrig = 0x140549367, kZoneEnd = 0x140549631;
constexpr uintptr_t kPatchExit = 0x140549625, kResumeExit = 0x14054962d;
constexpr uintptr_t kTest = 0x14048e810;
constexpr uint8_t kRunBytes[7] = {0x45, 0x33, 0xD2, 0x44, 0x89, 0x55, 0x90};       // xor r10d,r10d; mov [rbp-0x70],r10d
constexpr uint8_t kExitBytes[8] = {0x48, 0x8B, 0x7D, 0xB8, 0x4C, 0x8B, 0x75, 0x98}; // mov rdi,[rbp-0x48]; mov r14,[rbp-0x68]
constexpr int kCheckEvery = 64;

using Test_t = char(__fastcall*)(const uint8_t* shape, const double* point, float* value, uint32_t* cell);
Test_t g_test = nullptr;
bool g_on = false;

std::vector<body_contain::Body> g_bodies;
body_contain::Grid g_grid;
// self-check: our answer for the tick the game's loop runs
struct Answer {
    bool found;
    uint64_t entry;
    float value, v[3];
    int b; // index of the containing body
};
std::vector<Answer> g_answer;
bool g_armed = false;
uint64_t g_calls = 0;

// stats (update / render thread: one at a time)
uint64_t g_ticks = 0, g_body_sum = 0, g_tests = 0, g_cycles = 0, g_checks_ok = 0, g_value_moved = 0, g_data_moved = 0;
std::vector<body_contain::Body> g_fresh; // self-check: the bodies re-read after the game's loop
body_contain::Grid g_fresh_grid;
DWORD g_last_log = 0;

template <typename T> T rd(const void* p, size_t off) { T v; memcpy(&v, static_cast<const uint8_t*>(p) + off, sizeof v); return v; }
template <typename T> void wr(void* p, size_t off, T v) { memcpy(static_cast<uint8_t*>(p) + off, &v, sizeof v); }

void velocity(const uint8_t* a, const uint8_t* b, float out[3])
{
    const uint8_t* rc = rd<const uint8_t*>(b, 0x718);
    const uint8_t* ra = rd<const uint8_t*>(a, 0x718);
    float dz = static_cast<float>(rd<double>(rc, 0x88) - rd<double>(ra, 0x88));
    float dy = static_cast<float>(rd<double>(rc, 0x80) - rd<double>(ra, 0x80));
    float dx = static_cast<float>(rd<double>(rc, 0x78) - rd<double>(ra, 0x78));
    float wx = rd<float>(rc, 0xa4), wy = rd<float>(rc, 0xa8), wz = rd<float>(rc, 0xac);
    float z = dy * wx;
    z -= dx * wy;
    float y = dx * wz;
    y -= dz * wx;
    z += rd<float>(rc, 0xa0);
    y += rd<float>(rc, 0x9c);
    float x = dz * wy;
    x -= dy * wz;
    x += rd<float>(rc, 0x98);
    out[0] = x, out[1] = y, out[2] = z;
}

uint64_t cell_entry(const uint8_t* b, uint32_t cell)
{
    uint32_t cap = rd<uint32_t>(b, 0xf88);
    uint32_t idx = (rd<uint32_t>(b, 0xf8c) + cell) % cap;
    return rd<uint64_t>(rd<const uint8_t*>(b, 0xf80), size_t(idx) * 8);
}

bool eligible(const uint8_t* entry)
{
    if (!entry || rd<int>(entry, 0x10) != 5 || rd<uint8_t>(entry, 0) != 0) return false;
    const uint8_t* ring = rd<const uint8_t*>(entry, 8);
    return ring && rd<uint32_t>(ring, 0x10) != 0;
}

// The bodies in the game's list order. False: something we can't handle (the game's loop runs).
bool collect(const uint8_t* sim, std::vector<body_contain::Body>& out)
{
    out.clear();
    uint32_t n = rd<uint32_t>(sim, 0x330), cap = rd<uint32_t>(sim, 0x328), head = rd<uint32_t>(sim, 0x32c);
    const uint8_t* const* list = rd<const uint8_t* const*>(sim, 0x320);
    if (n == 0) return true;
    if (!list || cap == 0) return false;
    for (uint32_t i = 0; i < n; i++) {
        const uint8_t* entry = list[(head + i) % cap];
        if (!eligible(entry)) continue;
        const uint8_t* ring = rd<const uint8_t*>(entry, 8);
        const uint8_t* const* data = rd<const uint8_t* const*>(ring, 0);
        uint32_t rcap = rd<uint32_t>(ring, 8), rhead = rd<uint32_t>(ring, 0xc), rn = rd<uint32_t>(ring, 0x10);
        if (!data || rcap == 0) return false;
        for (uint32_t j = 0; j < rn; j++) {
            const uint8_t* b = data[(rhead + j) % rcap];
            if (!b) return false;
            const uint8_t* rigid = rd<const uint8_t*>(b, 0x718);
            if (!rigid) return false; // the game would pass a bad pointer here; leave it to the game
            body_contain::Body x{};
            x.p = b;
            x.id = rd<uint32_t>(b, 0x400);
            for (int k = 0; k < 3; k++) x.point[k] = rd<double>(rigid, 0x78 + 8 * k);
            const uint8_t* q = b + 0x738;
            double l0 = rd<double>(q, 0x148), l1 = rd<double>(q, 0x150), l2 = rd<double>(q, 0x158);
            for (int k = 0; k < 3; k++)
                x.centre[k] = l0 * rd<double>(q, 0x508 + 8 * k) + l1 * rd<double>(q, 0x528 + 8 * k) + l2 * rd<double>(q, 0x548 + 8 * k) +
                              rd<double>(q, 0x568 + 8 * k);
            double r = std::fabs(static_cast<double>(rd<float>(q, 0x160)));
            x.reach = 2.0 * r * 1.001 + 0.05;
            out.push_back(x);
        }
    }
    return true;
}

void maybe_log()
{
    DWORD now = GetTickCount();
    if (!g_last_log) g_last_log = now;
    if (now - g_last_log < 5000 || !g_ticks) return;
    LARGE_INTEGER fr;
    QueryPerformanceFrequency(&fr);
    double tsc_per_ms = double(fr.QuadPart) / 1000.0; // g_cycles counts QPC ticks
    double bodies = double(g_body_sum) / g_ticks;
    swlog("body contain 5s: %llu ticks, %.0f vehicle bodies, %.0f containment tests per tick (the full walk tries ~%.0f pairs), "
          "%.3f ms per tick, %llu self-checks ok (%llu with a value and %llu with a body that moved between the two reads)",
          static_cast<unsigned long long>(g_ticks), bodies, double(g_tests) / g_ticks, bodies * bodies / 2,
          g_cycles / tsc_per_ms / g_ticks, static_cast<unsigned long long>(g_checks_ok),
          static_cast<unsigned long long>(g_value_moved), static_cast<unsigned long long>(g_data_moved));
    g_ticks = g_body_sum = g_tests = g_cycles = g_checks_ok = g_value_moved = g_data_moved = 0;
    g_last_log = now;
}

int run_impl(uint8_t* sim)
{
    if (!collect(sim, g_bodies)) return 0;
    bool check = (++g_calls % kCheckEvery) == 0;
    LARGE_INTEGER t0;
    QueryPerformanceCounter(&t0);
    auto test = [&](uint32_t bi, uint32_t ai, float* value, uint32_t* cell) {
        const uint8_t* b = static_cast<const uint8_t*>(g_bodies[bi].p);
        const uint8_t* a = static_cast<const uint8_t*>(g_bodies[ai].p);
        return g_test(b + 0x738, reinterpret_cast<const double*>(rd<const uint8_t*>(a, 0x718) + 0x78), value, cell) != 0;
    };
    uint64_t tests;
    if (check) {
        g_answer.assign(g_bodies.size(), Answer{});
        tests = body_contain::run(
            g_bodies, g_grid, test,
            [&](uint32_t ai, uint32_t bi, float value, uint32_t cell) {
                const uint8_t* a = static_cast<const uint8_t*>(g_bodies[ai].p);
                const uint8_t* b = static_cast<const uint8_t*>(g_bodies[bi].p);
                Answer& x = g_answer[ai];
                x.found = true;
                x.b = int(bi);
                x.entry = cell_entry(b, cell);
                x.value = value;
                velocity(a, b, x.v);
            },
            [&](uint32_t ai) { g_answer[ai].found = false; });
    } else {
        tests = body_contain::run(
            g_bodies, g_grid, test,
            [&](uint32_t ai, uint32_t bi, float value, uint32_t cell) {
                uint8_t* a = const_cast<uint8_t*>(static_cast<const uint8_t*>(g_bodies[ai].p));
                const uint8_t* b = static_cast<const uint8_t*>(g_bodies[bi].p);
                wr<uint64_t>(a, 0x14b8, cell_entry(b, cell));
                wr<float>(a, 0x14c0, value);
                float v[3];
                velocity(a, b, v);
                wr<float>(a, 0x14c4, v[0]), wr<float>(a, 0x14c8, v[1]), wr<float>(a, 0x14cc, v[2]);
            },
            [&](uint32_t ai) { wr<uint64_t>(const_cast<void*>(g_bodies[ai].p), 0x14b8, 0); });
    }
    LARGE_INTEGER t1;
    QueryPerformanceCounter(&t1);
    g_cycles += t1.QuadPart - t0.QuadPart;
    g_ticks++;
    g_body_sum += g_bodies.size();
    g_tests += tests;
    maybe_log();
    if (check) {
        g_armed = true;
        return 0; // the game's loop runs this tick; after_impl compares
    }
    return 1;
}

// One body's answer from v: through the grid, or (grid null) the game's full walk.
Answer eval_one(const std::vector<body_contain::Body>& v, body_contain::Grid* grid, uint32_t ai)
{
    Answer x{};
    x.b = -1;
    const body_contain::Body& a = v[ai];
    const uint8_t* ap = static_cast<const uint8_t*>(a.p);
    uint32_t best = 0xffffffffu;
    auto try_b = [&](uint32_t bi) {
        if (bi == ai || v[bi].id <= a.id || v[bi].id >= best) return;
        const uint8_t* bp = static_cast<const uint8_t*>(v[bi].p);
        float value = 0;
        uint32_t cell = 0;
        if (!g_test(bp + 0x738, reinterpret_cast<const double*>(rd<const uint8_t*>(ap, 0x718) + 0x78), &value, &cell)) return;
        x.found = true;
        x.b = int(bi);
        x.entry = cell_entry(bp, cell);
        x.value = value;
        velocity(ap, bp, x.v);
        best = v[bi].id;
    };
    bool placeable = std::isfinite(a.point[0]) && std::isfinite(a.point[1]) && std::isfinite(a.point[2]);
    if (grid && placeable)
        for (uint32_t bi : grid->query(a.point)) try_b(bi);
    else
        for (uint32_t bi = 0; bi < v.size(); bi++) try_b(bi);
    return x;
}

bool same_choice(const Answer& p, const Answer& q)
{
    return p.found == q.found && (!p.found || (p.entry == q.entry && memcmp(p.v, q.v, sizeof p.v) == 0));
}

void after_impl(const uint8_t* sim)
{
    g_armed = false;
    bool value_moved = false, fresh = false;
    for (size_t i = 0; i < g_bodies.size(); i++) {
        const uint8_t* a = static_cast<const uint8_t*>(g_bodies[i].p);
        const Answer& x = g_answer[i];
        uint64_t entry = rd<uint64_t>(a, 0x14b8);
        bool same = x.found ? entry == x.entry && memcmp(a + 0x14c4, x.v, sizeof x.v) == 0 : entry == 0;
        if (same) {
            if (x.found && rd<uint32_t>(a, 0x14c0) != *reinterpret_cast<const uint32_t*>(&x.value)) value_moved = true;
            continue;
        }
        // Grid or data? Re-read everything now and ask both the full walk and the grid about this body.
        if (!fresh) {
            if (!collect(sim, g_fresh)) g_fresh.clear();
            g_fresh_grid.build(g_fresh);
            fresh = true;
        }
        uint32_t fi = 0;
        while (fi < g_fresh.size() && g_fresh[fi].p != a) fi++;
        if (fi == g_fresh.size()) { // gone from the list in between: nothing to judge
            g_data_moved++;
            continue;
        }
        Answer full = eval_one(g_fresh, nullptr, fi), grid = eval_one(g_fresh, &g_fresh_grid, fi);
        if (same_choice(full, grid)) { // the grid agrees with the game on the same data: the data moved
            if (g_data_moved++ == 0)
                swlog("body contain: self-check: body %zu (%p, id %u) changed between our read and the game's (entry %p then, %p now); "
                      "the grid agrees with the full walk on the new data, grid stays on", i, a, g_bodies[i].id,
                      reinterpret_cast<void*>(x.entry), reinterpret_cast<void*>(entry));
            continue;
        }
        g_on = false;
        if (full.b >= 0) {
            const body_contain::Body& fb = g_fresh[full.b];
            double d2 = 0;
            for (int k = 0; k < 3; k++) d2 += (g_fresh[fi].point[k] - fb.centre[k]) * (g_fresh[fi].point[k] - fb.centre[k]);
            swlog("body contain: grid missed body %d (%p, id %u) as the container: its centre (%.3f %.3f %.3f), reach %.3f m, "
                  "distance %.3f m from the point (%.3f %.3f %.3f)", full.b, fb.p, fb.id, fb.centre[0], fb.centre[1], fb.centre[2],
                  fb.reach, std::sqrt(d2), g_fresh[fi].point[0], g_fresh[fi].point[1], g_fresh[fi].point[2]);
        }
        swlog("body contain: self-check differs from the game's loop at body %zu of %zu (%p, id %u): game entry %p value %g "
              "velocity (%g %g %g), grid %s entry %p value %g velocity (%g %g %g); grid off for this session",
              i, g_bodies.size(), a, g_bodies[i].id, reinterpret_cast<void*>(entry), rd<float>(a, 0x14c0), rd<float>(a, 0x14c4),
              rd<float>(a, 0x14c8), rd<float>(a, 0x14cc), x.found ? "found" : "none", reinterpret_cast<void*>(x.entry), x.value, x.v[0],
              x.v[1], x.v[2]);
        return;
    }
    g_checks_ok++;
    g_value_moved += value_moved;
}

// Called from the stubs.
int body_contain_run(uint8_t* sim)
{
    if (!g_on) return 0;
    __try {
        return run_impl(sim);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        g_on = false;
        g_armed = false;
        swlog("body contain: fault while reading the vehicle list; the game's loop from now on");
        return 0;
    }
}

void body_contain_after(uint8_t* sim)
{
    if (!g_armed) return;
    __try {
        after_impl(sim);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        g_on = false;
        g_armed = false;
        swlog("body contain: fault in the self-check; the game's loop from now on");
    }
}

uintptr_t live(uintptr_t a) { return game_live(a); }

uint8_t* alloc_near(uintptr_t target, size_t size)
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    uintptr_t gran = si.dwAllocationGranularity;
    for (uintptr_t d = gran; d < 0x70000000; d += gran)
        for (uintptr_t a : {target - d, target + d}) {
            a &= ~(gran - 1);
            if (void* p = VirtualAlloc(reinterpret_cast<void*>(a), size, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE))
                return static_cast<uint8_t*>(p);
        }
    return nullptr;
}

void put64(uint8_t* at, uint64_t v) { memcpy(at, &v, 8); }

size_t make_run_stub(uint8_t* m)
{
    uint8_t c[] = {
        0x4C, 0x89, 0xE9,                   // mov rcx, r13
        0x48, 0x83, 0xEC, 0x20,             // sub rsp, 0x20
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0, // mov rax, body_contain_run
        0xFF, 0xD0,                         // call rax
        0x48, 0x83, 0xC4, 0x20,             // add rsp, 0x20
        0x85, 0xC0,                         // test eax, eax
        0x74, 0x0C,                         // jz original
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0, // mov rax, zone end
        0xFF, 0xE0,                         // jmp rax
        0x45, 0x33, 0xD2,                   // original: xor r10d, r10d
        0x44, 0x89, 0x55, 0x90,             //           mov [rbp-0x70], r10d
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,
        0xFF, 0xE0,                         // jmp rax
    };
    put64(c + 9, reinterpret_cast<uint64_t>(&body_contain_run));
    put64(c + 29, live(kZoneEnd));
    put64(c + 48, live(kResumeOrig));
    memcpy(m, c, sizeof c);
    return sizeof c;
}

size_t make_exit_stub(uint8_t* m)
{
    uint8_t c[] = {
        0x4C, 0x89, 0xE9,                   // mov rcx, r13
        0x48, 0x83, 0xEC, 0x20,             // sub rsp, 0x20
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0, // mov rax, body_contain_after
        0xFF, 0xD0,                         // call rax
        0x48, 0x83, 0xC4, 0x20,             // add rsp, 0x20
        0x48, 0x8B, 0x7D, 0xB8,             // mov rdi, [rbp-0x48]
        0x4C, 0x8B, 0x75, 0x98,             // mov r14, [rbp-0x68]
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,
        0xFF, 0xE0,                         // jmp rax
    };
    put64(c + 9, reinterpret_cast<uint64_t>(&body_contain_after));
    put64(c + 33, live(kResumeExit));
    memcpy(m, c, sizeof c);
    return sizeof c;
}

bool patch_jump(uintptr_t site, const uint8_t* stub, size_t len)
{
    int64_t rel = reinterpret_cast<intptr_t>(stub) - static_cast<intptr_t>(site + 5);
    if (rel < INT32_MIN || rel > INT32_MAX) return false;
    uint8_t code[8];
    memset(code, 0x90, sizeof code);
    code[0] = 0xE9;
    int32_t r = static_cast<int32_t>(rel);
    memcpy(code + 1, &r, 4);
    DWORD old;
    if (!VirtualProtect(reinterpret_cast<void*>(site), len, PAGE_EXECUTE_READWRITE, &old)) return false;
    memcpy(reinterpret_cast<void*>(site), code, len); // mod init runs before the game loop starts
    VirtualProtect(reinterpret_cast<void*>(site), len, old, &old);
    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(site), len);
    return true;
}

bool install()
{
    if (!GetPrivateProfileIntW(L"contain", L"grid", 1, settings_ini())) {
        swlog("body contain: off in settings.ini (zone 0x30 walks every pair of vehicle bodies, as the game does)");
        return true;
    }
    uintptr_t run_site = live(kPatchRun), exit_site = live(kPatchExit);
    if (memcmp(reinterpret_cast<const void*>(run_site), kRunBytes, sizeof kRunBytes) != 0 ||
        memcmp(reinterpret_cast<const void*>(exit_site), kExitBytes, sizeof kExitBytes) != 0) {
        swlog("body contain: game code differs from the expected build; off");
        return true;
    }
    g_test = reinterpret_cast<Test_t>(live(kTest));
    uint8_t* mem = alloc_near(run_site, 4096);
    if (!mem) {
        swlog("body contain: no memory near the game for the stubs; off");
        return true;
    }
    size_t run_len = make_run_stub(mem);
    uint8_t* exit_stub = mem + ((run_len + 15) & ~size_t(15));
    make_exit_stub(exit_stub);
    FlushInstructionCache(GetCurrentProcess(), mem, 4096);
    // exit first: it only acts on armed ticks, which only the run stub creates
    if (!patch_jump(exit_site, exit_stub, sizeof kExitBytes) || !patch_jump(run_site, mem, sizeof kRunBytes)) {
        swlog("body contain: patch FAILED; off");
        return false;
    }
    g_on = true;
    swlog("body contain: zone 0x30 finds containing bodies through a %.0f m grid (self-check against the game's loop every %d "
          "ticks; \"body contain 5s\" lines)", body_contain::kCell, kCheckEvery);
    return true;
}

HookRegistrar reg{{"body contain", /*needs_known_build=*/true, &install}};

} // namespace
