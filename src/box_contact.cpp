
#include "box_contact.h"
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

namespace {

constexpr uintptr_t kRefreshContactPoints = 0x140110d10; // btPersistentManifold::refreshContactPoints(manifold, trA, trB)

constexpr size_t kWorldDispatcher = 0x28;
constexpr size_t kDispatcherMatrix = 0x48; // m_doubleDispatchContactPoints[36][36]
constexpr int kMaxTypes = 36;
constexpr size_t kDispatcherGetNewManifold = 2;
constexpr size_t kWrapShape = 0x8, kWrapObject = 0x10, kWrapTransform = 0x18;
constexpr size_t kObjTransform = 0x10;
constexpr size_t kResManifold = 0x8, kResBody0 = 0x10, kResBody1 = 0x18, kResClosestThreshold = 0x30, kResAddContact = 3;
constexpr size_t kManBody0 = 0x5d0, kManPoints = 0x5e0, kManBreaking = 0x5e8;
constexpr size_t kAlgDispatcher = 0x8, kAlgOwn = 0x58, kAlgManifold = 0x60;
constexpr int kAlgSlots = 4; // ~, processCollision, calculateTimeOfImpact, getAllContactManifolds
constexpr size_t kShapeType = 0x8, kShapeScaling = 0x20, kShapeImplicit = 0x40, kShapeMargin = 0x60, kCapsuleUp = 0x70;
constexpr int kBox = 0, kSphere = 8, kCapsule = 10;
constexpr int kEntries[4][2] = {{kBox, kSphere}, {kSphere, kBox}, {kBox, kCapsule}, {kCapsule, kBox}};

template <typename T> T rd(const void* p, size_t off)
{
    T v;
    memcpy(&v, static_cast<const uint8_t*>(p) + off, sizeof v);
    return v;
}
template <typename T> void wr(void* p, size_t off, T v) { memcpy(static_cast<uint8_t*>(p) + off, &v, sizeof v); }
void** vtable_of(const void* obj) { return *reinterpret_cast<void** const*>(obj); }

const char* rtti_name(const void* obj)
{
    __try {
        auto vt = reinterpret_cast<const uintptr_t*>(vtable_of(obj));
        if (!vt) return nullptr;
        auto col = reinterpret_cast<const uint32_t*>(vt[-1]); // RTTICompleteObjectLocator
        if (!col || col[0] != 1) return nullptr;
        uintptr_t base = reinterpret_cast<uintptr_t>(col) - col[5];
        if (base != game_base()) return nullptr;
        const char* name = reinterpret_cast<const char*>(base + col[3] + 0x10);
        return name[0] == '.' && name[1] == '?' && name[2] == 'A' ? name : nullptr;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;
    }
}
bool rtti_is(const void* obj, const char* tail) // tail after ".?AV" / ".?AU"
{
    const char* n = rtti_name(obj);
    return n && strcmp(n + 4, tail) == 0;
}

using Create_t = void*(__fastcall*)(void* cf, void* ci, const void* w0, const void* w1);
using Process_t = void(__fastcall*)(void* alg, const void* w0, const void* w1, const void* info, void* result);
using GetNewManifold_t = void*(__fastcall*)(void* dispatcher, const void* b0, const void* b1);
using AddContact_t = void(__fastcall*)(void* result, const double* normal_on_b, const double* point, double depth);
using Refresh_t = void(__fastcall*)(void* manifold, const void* tr_a, const void* tr_b);

std::atomic<bool> g_on{false};
Refresh_t g_refresh = nullptr;

SRWLOCK g_vt_lock = SRWLOCK_INIT;
std::atomic<void**> g_cc_vtable{nullptr}; // published after g_vt / g_cc_process are filled
void* g_vt[1 + kAlgSlots];
Process_t g_cc_process = nullptr;

struct Stats {
    uint64_t calls[2] = {}, deep = 0, points = 0, cycles = 0, fallback = 0;
};
thread_local Stats* tl_stats = nullptr;
std::mutex g_stats_mu;
std::vector<Stats*> g_stats;
std::atomic<uint64_t> g_swapped{0}, g_kept{0};
Stats& stats()
{
    if (!tl_stats) {
        tl_stats = new Stats();
        std::lock_guard<std::mutex> lock(g_stats_mu);
        g_stats.push_back(tl_stats);
    }
    return *tl_stats;
}

void disable(const char* why)
{
    if (g_on.exchange(false)) swlog("box contacts: off (%s); new and existing pairs use Bullet's GJK", why);
}

// One side of the pair: the box's or the other shape's parameters
bool box_params(const void* shape, double h[3], double& mb)
{
    for (int k = 0; k < 3; k++) h[k] = rd<double>(shape, kShapeImplicit + 8 * k);
    mb = rd<double>(shape, kShapeMargin);
    return true;
}
bool round_params(const void* shape, int type, const double* tr, double p0[3], double p1[3], double& r)
{
    if (type == kSphere) {
        r = rd<double>(shape, kShapeImplicit) * rd<double>(shape, kShapeScaling); // btSphereShape::getRadius
        for (int k = 0; k < 3; k++) p0[k] = p1[k] = tr[12 + k];
        return true;
    }
    int up = rd<int>(shape, kCapsuleUp);
    if (up < 0 || up > 2) return false;
    double hh = rd<double>(shape, kShapeImplicit + 8 * up);
    r = rd<double>(shape, kShapeMargin); // the capsule's margin is its radius (what GJK uses)
    for (int k = 0; k < 3; k++) {
        double ax = tr[4 * k + up] * hh; // column up of the basis
        p0[k] = tr[12 + k] - ax;
        p1[k] = tr[12 + k] + ax;
    }
    return true;
}

void __fastcall process(void* alg, const void* w0, const void* w1, const void* info, void* result)
{
    if (!g_on.load(std::memory_order_relaxed)) return g_cc_process(alg, w0, w1, info, result);
    const void* s0 = rd<const void*>(w0, kWrapShape);
    const void* s1 = rd<const void*>(w1, kWrapShape);
    int t0 = rd<int>(s0, kShapeType), t1 = rd<int>(s1, kShapeType);
    bool box_first = t0 == kBox;
    int other = box_first ? t1 : t0;
    if ((box_first ? t0 : t1) != kBox || (other != kSphere && other != kCapsule)) {
        stats().fallback++;
        return g_cc_process(alg, w0, w1, info, result);
    }
    uint64_t c0 = __rdtsc();
    const void* bw = box_first ? w0 : w1;
    const void* ow = box_first ? w1 : w0;
    const double* btr = rd<const double*>(bw, kWrapTransform);
    const double* otr = rd<const double*>(ow, kWrapTransform);
    double h[3], mb, p0[3], p1[3], r;
    box_contact::Contact c;
    if (!box_params(box_first ? s0 : s1, h, mb) || !round_params(box_first ? s1 : s0, other, otr, p0, p1, r) ||
        !box_contact::box_segment(btr, h, mb, p0, p1, r, c)) {
        stats().fallback++;
        return g_cc_process(alg, w0, w1, info, result);
    }

    void* man = rd<void*>(alg, kAlgManifold);
    if (!man) {
        void* disp = rd<void*>(alg, kAlgDispatcher);
        auto get = reinterpret_cast<GetNewManifold_t>(vtable_of(disp)[kDispatcherGetNewManifold]);
        man = get(disp, rd<const void*>(w0, kWrapObject), rd<const void*>(w1, kWrapObject));
        wr<void*>(alg, kAlgManifold, man);
        wr<uint8_t>(alg, kAlgOwn, 1);
    }
    wr<void*>(result, kResManifold, man);

    // Bullet's convention: A = body0 of processCollision, normal on B pointing to A, point on B, depth < 0 overlapping
    double depth = -c.pen;
    alignas(16) double n[4], pt[4];
    if (box_first) {
        for (int k = 0; k < 3; k++) n[k] = -c.s[k], pt[k] = c.q[k];
    } else {
        for (int k = 0; k < 3; k++) n[k] = c.s[k], pt[k] = c.q[k] + c.s[k] * c.pen;
    }
    n[3] = pt[3] = 0;
    Stats& st = stats();
    st.calls[other == kCapsule]++;
    st.deep += c.deep;
    if (depth < rd<double>(man, kManBreaking) + rd<double>(result, kResClosestThreshold)) {
        reinterpret_cast<AddContact_t>(vtable_of(result)[kResAddContact])(result, n, pt, depth);
        st.points++;
    }
    if (rd<uint8_t>(alg, kAlgOwn) && rd<int>(man, kManPoints)) {
        const void* o0 = rd<const void*>(rd<const void*>(result, kResBody0), kWrapObject);
        const void* o1 = rd<const void*>(rd<const void*>(result, kResBody1), kWrapObject);
        const uint8_t* tr0 = static_cast<const uint8_t*>(o0) + kObjTransform;
        const uint8_t* tr1 = static_cast<const uint8_t*>(o1) + kObjTransform;
        if (rd<const void*>(man, kManBody0) == o0) g_refresh(man, tr0, tr1);
        else g_refresh(man, tr1, tr0);
    }
    st.cycles += __rdtsc() - c0;
}

bool learn_vtable(void* alg, void* ci)
{
    AcquireSRWLockExclusive(&g_vt_lock);
    bool ok = g_cc_vtable.load() != nullptr;
    if (!ok && g_on.load()) {
        void** vt = vtable_of(alg);
        if (!rtti_is(alg, "btConvexConvexAlgorithm@@")) disable("the CreateFunc made something other than btConvexConvexAlgorithm");
        else if (rd<void*>(alg, kAlgDispatcher) != rd<void*>(ci, 0) || rd<void*>(alg, kAlgManifold) != rd<void*>(ci, 8) ||
                 rd<uint8_t>(alg, kAlgOwn) != 0)
            disable("btConvexConvexAlgorithm layout differs (dispatcher +8, manifold +0x60, own flag +0x58)");
        else {
            g_vt[0] = vt[-1];
            for (int i = 0; i < kAlgSlots; i++) g_vt[1 + i] = vt[i];
            g_cc_process = reinterpret_cast<Process_t>(vt[1]);
            g_vt[1 + 1] = reinterpret_cast<void*>(&process);
            g_cc_vtable.store(vt, std::memory_order_release);
            ok = true;
        }
    }
    ReleaseSRWLockExclusive(&g_vt_lock);
    return ok;
}

struct StandIn {
    virtual ~StandIn() = default; // never deleted by the game: the configuration deletes only its own CreateFuncs
    virtual void* create(void* ci, const void* w0, const void* w1)
    {
        void* alg = reinterpret_cast<Create_t>(vtable_of(orig)[1])(orig, ci, w0, w1);
        if (!alg || !g_on.load(std::memory_order_relaxed)) return alg;
        void** cc = g_cc_vtable.load(std::memory_order_acquire);
        if (!cc && learn_vtable(alg, ci)) cc = g_cc_vtable.load(std::memory_order_acquire);
        if (!cc || vtable_of(alg) != cc) {
            g_kept.fetch_add(1, std::memory_order_relaxed);
            return alg;
        }
        *reinterpret_cast<void***>(alg) = &g_vt[1];
        g_swapped.fetch_add(1, std::memory_order_relaxed);
        return alg;
    }
    bool swapped = false;
    void* orig = nullptr;
};
void** g_standin_vtable = nullptr;
const void* g_rejected[4] = {};

DWORD WINAPI log_loop(void*)
{
    Stats last{};
    uint64_t last_swapped = 0;
    for (;;) {
        Sleep(5000);
        Stats sum{};
        {
            std::lock_guard<std::mutex> lock(g_stats_mu);
            for (Stats* s : g_stats) {
                sum.calls[0] += s->calls[0], sum.calls[1] += s->calls[1], sum.deep += s->deep, sum.points += s->points;
                sum.cycles += s->cycles, sum.fallback += s->fallback;
            }
        }
        uint64_t sp = sum.calls[0] - last.calls[0], cp = sum.calls[1] - last.calls[1], n = sp + cp;
        uint64_t sw = g_swapped.load() - last_swapped;
        if (n || sw)
            swlog("box contacts 5s: %llu box-sphere + %llu box-capsule pairs (%.1f%% cores overlapping), %llu contact points, "
                  "%.0f cycles per pair, %llu handed back to GJK, %llu new algorithms swapped (%llu total, %llu not swappable)",
                  (unsigned long long)sp, (unsigned long long)cp, n ? 100.0 * double(sum.deep - last.deep) / double(n) : 0.0,
                  (unsigned long long)(sum.points - last.points), n ? double(sum.cycles - last.cycles) / double(n) : 0.0,
                  (unsigned long long)(sum.fallback - last.fallback), (unsigned long long)sw, (unsigned long long)g_swapped.load(),
                  (unsigned long long)g_kept.load());
        last = sum;
        last_swapped = g_swapped.load();
    }
}

bool install()
{
    if (!GetPrivateProfileIntW(L"narrowphase", L"box_contacts", 0, settings_ini())) {
        swlog("box contacts: off ([narrowphase] box_contacts=1 to try closed-form box vs capsule / sphere contacts)");
        return true;
    }
    g_refresh = game_ptr<Refresh_t>(kRefreshContactPoints);
    if (!g_refresh) {
        swlog("box contacts: btPersistentManifold::refreshContactPoints not found on this build; off");
        return true;
    }
    StandIn probe;
    g_standin_vtable = vtable_of(&probe);
    g_on = true;
    CloseHandle(CreateThread(nullptr, 0, log_loop, nullptr, 0, nullptr));
    swlog("box contacts: on (box vs capsule / sphere without GJK; installed into the dispatcher at the first physics step)");
    return true;
}

HookRegistrar reg{{"box contacts", /*needs_known_build=*/true, &install}};

} // namespace

// Physics thread, before every stepSimulation (census.cpp): puts the stand-ins into the world's dispatcher. Checked
// every step because a new world (menu -> game, reload) brings a new dispatcher, possibly at the same address.
void box_contacts_step_begin(const void* world)
{
    if (!g_on.load(std::memory_order_relaxed)) return;
    void* disp = rd<void*>(world, kWorldDispatcher);
    if (!disp) return;
    for (int i = 0; i < 4; i++) {
        void** entry = reinterpret_cast<void**>(static_cast<uint8_t*>(disp) + kDispatcherMatrix +
                                                (size_t(kEntries[i][0]) * kMaxTypes + kEntries[i][1]) * sizeof(void*));
        void* cur = *entry;
        if (!cur || vtable_of(cur) == g_standin_vtable || cur == g_rejected[i]) continue;
        if (!rtti_is(disp, "btCollisionDispatcher@@")) {
            disable("the world's dispatcher is not a btCollisionDispatcher");
            return;
        }
        if (!rtti_is(cur, "CreateFunc@btConvexConvexAlgorithm@@")) {
            const char* name = rtti_name(cur);
            swlog("box contacts: dispatcher entry (%d, %d) is %s, not the convex-convex one; left alone", kEntries[i][0],
                  kEntries[i][1], name ? name : "unknown");
            g_rejected[i] = cur;
            continue;
        }
        auto* s = new StandIn(); // one per entry and dispatcher, never freed (a few per world load)
        s->swapped = rd<uint8_t>(cur, 8) != 0;
        s->orig = cur;
        *entry = s;
        swlog("box contacts: dispatcher %p entry (%d, %d) -> closed form", disp, kEntries[i][0], kEntries[i][1]);
    }
}
