
#include "tri_empty.h"
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <atomic>
#include <cstdint>

namespace {

constexpr uintptr_t kProcessAll = 0x1400f88a0, kMeshDtor = 0x1400f8550, kConvexTriangleCallbackVtbl = 0x140ae1350;
constexpr size_t kCbAabbMin = 0x10, kCbAabbMax = 0x30;
constexpr int kCheckEvery = 64;

using ProcessAll_t = void(__fastcall*)(void* shape, void* cb, const double* mn, const double* mx);
using MeshDtor_t = void*(__fastcall*)(void* shape, unsigned flags);
using ProcessTriangle_t = void(__fastcall*)(void* cb, double* tri, int part, int index);
ProcessAll_t o_process_all = nullptr;
MeshDtor_t o_dtor = nullptr;
uintptr_t g_cb_vtbl = 0;

std::atomic<bool> g_on{false};
double g_margin = 0.25;
std::atomic<uint64_t> g_gen{1}, g_pruned_gen{1};
tri_empty::Cache g_cache;

std::atomic<uint64_t> g_calls{0}, g_other{0}, g_skipped{0}, g_stored{0}, g_walked{0}, g_touching{0}, g_checks{0}, g_skip_seq{0};
std::atomic<uint64_t> g_last_log{0};

struct Counter;
void __fastcall counter_tri(Counter* self, double* tri, int part, int index);
void* __fastcall counter_dtor(Counter* self, unsigned) { return self; }
const void* const kCounterVtbl[2] = {reinterpret_cast<const void*>(&counter_dtor), reinterpret_cast<const void*>(&counter_tri)};
struct Counter {
    const void* const* vtbl = kCounterVtbl;
    void* real;
    uint64_t leaves = 0;
};
void __fastcall counter_tri(Counter* self, double* tri, int part, int index)
{
    self->leaves++;
    auto fn = reinterpret_cast<ProcessTriangle_t>((*reinterpret_cast<void* const* const*>(self->real))[1]);
    fn(self->real, tri, part, index);
}

void maybe_log()
{
    uint64_t now = GetTickCount64(), last = g_last_log.load(std::memory_order_relaxed);
    if (now - last < 5000 || !g_last_log.compare_exchange_strong(last, now)) return;
    if (!last) return;
    uint64_t calls = g_calls.exchange(0), other = g_other.exchange(0), skipped = g_skipped.exchange(0), stored = g_stored.exchange(0),
             walked = g_walked.exchange(0), touching = g_touching.exchange(0), checks = g_checks.exchange(0);
    uint64_t gen = g_gen.load();
    if (g_pruned_gen.exchange(gen) != gen) g_cache.prune(gen);
    if (!calls) return;
    swlog("tri empty 5s: %.0f mesh walks/s from convex pairs, %.0f%% skipped (box inside a known empty one), %.0f/s found an empty "
          "grown box, %.0f/s touching (wait %d calls), %.0f/s walked as usual; %.0f/s other callbacks; %llu self-checks ok; %zu pairs kept",
          calls / 5.0, 100.0 * skipped / calls, stored / 5.0, touching / 5.0, tri_empty::kCooldown, walked / 5.0, other / 5.0,
          static_cast<unsigned long long>(checks), g_cache.size());
}

void __fastcall process_all_detour(void* shape, void* cb, const double* mn, const double* mx)
{
    if (!g_on.load(std::memory_order_relaxed) || !cb) return o_process_all(shape, cb, mn, mx);
    auto cbp = static_cast<const uint8_t*>(cb);
    bool own = *reinterpret_cast<const uintptr_t*>(cbp) == g_cb_vtbl && mn == reinterpret_cast<const double*>(cbp + kCbAabbMin) &&
               mx == reinterpret_cast<const double*>(cbp + kCbAabbMax);
    if (!own) {
        g_other.fetch_add(1, std::memory_order_relaxed);
        return o_process_all(shape, cb, mn, mx);
    }
    g_calls.fetch_add(1, std::memory_order_relaxed);
    bool check = false;
    auto walk = [&](const double* lo, const double* hi) -> uint64_t {
        Counter c;
        c.real = cb;
        o_process_all(shape, &c, lo, hi);
        return c.leaves;
    };
    // a skip is decided inside process(); whether it is due for a self-check has to be known before, so count first
    // only when the box is inside a kept one (cheap: the same lookup process() makes)
    uint64_t gen = g_gen.load(std::memory_order_acquire);
    bool hit = g_cache.with(cb, shape, gen, [&](tri_empty::Entry* e) { return e && e->empty && e->box.contains(mn, mx); });
    if (hit) check = (g_skip_seq.fetch_add(1, std::memory_order_relaxed) % kCheckEvery) == kCheckEvery - 1;
    tri_empty::Outcome o = tri_empty::process(g_cache, cb, shape, gen, mn, mx, true, g_margin, check, walk);
    switch (o) {
    case tri_empty::Outcome::kSkipped: g_skipped.fetch_add(1, std::memory_order_relaxed); break;
    case tri_empty::Outcome::kChecked:
        g_skipped.fetch_add(1, std::memory_order_relaxed);
        g_checks.fetch_add(1, std::memory_order_relaxed);
        break;
    case tri_empty::Outcome::kStored: g_stored.fetch_add(1, std::memory_order_relaxed); break;
    case tri_empty::Outcome::kTouching: g_touching.fetch_add(1, std::memory_order_relaxed); break;
    case tri_empty::Outcome::kMismatch:
        // the walk just ran with the real callback, so this call is right; the cache goes off for the session
        if (g_on.exchange(false))
            swlog("tri empty: self-check found triangles inside a box kept as empty (mesh %p, box %.3f %.3f %.3f .. %.3f %.3f %.3f); off",
                  shape, mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]);
        break;
    default: g_walked.fetch_add(1, std::memory_order_relaxed); break;
    }
    maybe_log();
}

void* __fastcall dtor_detour(void* shape, unsigned flags)
{
    g_gen.fetch_add(1, std::memory_order_acq_rel); // before the memory can be reused
    return o_dtor(shape, flags);
}

bool install()
{
    const wchar_t* ini = settings_ini();
    if (!GetPrivateProfileIntW(L"narrowphase", L"tri_empty", 1, ini)) {
        swlog("tri empty: off in settings.ini");
        return true;
    }
    g_margin = GetPrivateProfileIntW(L"narrowphase", L"tri_empty_margin_cm", 25, ini) / 100.0;
    g_cb_vtbl = reinterpret_cast<uintptr_t>(game_ptr<void*>(kConvexTriangleCallbackVtbl));
    if (!hook_game(kMeshDtor, reinterpret_cast<void*>(&dtor_detour), reinterpret_cast<void**>(&o_dtor))) return false;
    if (!hook_game(kProcessAll, reinterpret_cast<void*>(&process_all_detour), reinterpret_cast<void**>(&o_process_all))) return false;
    g_last_log.store(GetTickCount64());
    g_on = true;
    swlog("tri empty: convex vs mesh walks skipped while the convex's box stays inside a %.2f m grown box that reached no triangle "
          "(self-check every %d skips; \"tri empty 5s\" lines)", g_margin, kCheckEvery);
    return true;
}

HookRegistrar reg{{"tri_empty", /*needs_known_build=*/true, &install}};

} // namespace
