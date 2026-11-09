
#include "phys_pool.h"
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "solver_units.h"
#include "world.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

constexpr uintptr_t kSolveConstraints = 0x140134db0, kBuildIslands = 0x1400f4ff0, kProcessIslands = 0x1400f54d0,
                    kProcessIsland = 0x140132b40, kFlushBatch = 0x140132f90, kSolveGroup = 0x14012f1c0,
                    kSetup = 0x14012c5b0, kIterations = 0x14012e370, kFinish = 0x14012e4a0, kSingleIteration = 0x14012db40,
                    kSolverCtor = 0x1401281d0, kAlignedAllocFn = 0x140cbf728;

constexpr size_t kInfoIterations = 0x28, kInfoSplitImpulse = 0x68, kInfoSolverMode = 0x90, kInfoBatch = 0x98,
                 kInfoResidual = 0xb0;
constexpr size_t kSolverSize = 0x180, kSolverOverrideIters = 0x128, kSolverRowFns = 0x150, kSolverResidual = 0x170;
constexpr size_t kPoolSize[5] = {0x0c, 0x2c, 0x4c, 0x6c, 0x8c}; // capacity = size + 4
constexpr size_t kCbInfo = 0x08, kCbSolver = 0x10, kCbConstraints = 0x18, kCbConstraintCount = 0x20, kCbDebug = 0x28,
                 kCbDispatcher = 0x30;
constexpr size_t kManPoints = 0x10, kPointStride = 0x170, kPointDistance = 0xa0, kManBody0 = 0x5d0, kManBody1 = 0x5d8,
                 kManCount = 0x5e0;
constexpr size_t kConType = 0x08, kConA = 0x28, kConB = 0x30;
constexpr size_t kObjOrigin = 0x70, kObjProxy = 0x180, kObjShape = 0x188, kObjFlags = 0x1a0, kObjIslandTag = 0x1a4,
                 kObjInternalType = 0x1f0;
constexpr size_t kProxyGroup = 8, kCompoundTree = 0x80, kDbvtLeaves = 0x14;
constexpr int kShapeCompound = 31, kCharGroup = 0x120, kCfStatic = 1, kCfKinematic = 2;

// --- body classes ---------------------------------------------------------------------------------------------
// V vehicle body (dynamic compound), C character body (capsule / sphere in filter group 0x120), D other dynamic,
// S static, G ghost / other object.
enum Cls { kV, kC, kD, kS, kG, kClsN };

struct BodyInfo {
    Cls cls = kG;
    int children = 0;
    double pos[3] = {};
};

BodyInfo classify(const uint8_t* obj)
{
    BodyInfo b;
    if (!obj) return b;
    __try {
        int type = *reinterpret_cast<const int*>(obj + kObjInternalType);
        int flags = *reinterpret_cast<const int*>(obj + kObjFlags);
        memcpy(b.pos, obj + kObjOrigin, sizeof b.pos);
        auto shape = *reinterpret_cast<const uint8_t* const*>(obj + kObjShape);
        int stype = shape ? *reinterpret_cast<const int*>(shape + 8) : -1;
        if (flags & kCfStatic) b.cls = kS;
        else if (type != 2) b.cls = kG;
        else if (stype == kShapeCompound) {
            b.cls = kV;
            auto tree = *reinterpret_cast<const uint8_t* const*>(shape + kCompoundTree);
            b.children = tree ? *reinterpret_cast<const int*>(tree + kDbvtLeaves) : 0;
        } else {
            auto proxy = *reinterpret_cast<const uint8_t* const*>(obj + kObjProxy);
            int group = proxy ? *reinterpret_cast<const int*>(proxy + kProxyGroup) : 0;
            b.cls = (stype == 10 || stype == 8) && (group & kCharGroup) == kCharGroup ? kC : kD;
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        b.cls = kG;
    }
    return b;
}

// Manifold classes: the two sides' classes, sorted; V-V split into the same vehicle (joined bodies) or another.
enum MCls { kVVsame, kVVother, kVS, kVC, kVD, kCS, kCC, kCD, kDS, kDD, kMOther, kMClsN };
const char* const kMClsName[kMClsN] = {"vehicle-same vehicle", "vehicle-other vehicle", "vehicle-static", "vehicle-character",
                                       "vehicle-other dyn", "character-static", "character-character", "character-other dyn",
                                       "other dyn-static", "other dyn-other dyn", "other"};

MCls manifold_class(Cls a, Cls b, const void* oa, const void* ob)
{
    if (a > b) {
        std::swap(a, b);
        std::swap(oa, ob);
    }
    if (a == kV && b == kV) {
        int ga = body_group(oa), gb = body_group(ob);
        return ga >= 0 && ga == gb ? kVVsame : kVVother;
    }
    if (a == kV) return b == kS ? kVS : b == kC ? kVC : b == kD ? kVD : kMOther;
    if (a == kC) return b == kS ? kCS : b == kC ? kCC : b == kD ? kCD : kMOther;
    if (a == kD) return b == kS ? kDS : b == kD ? kDD : kMOther;
    return kMOther;
}

const char* constraint_name(int t)
{
    switch (t) {
    case 3: return "point2point";
    case 4: return "hinge";
    case 5: return "conetwist";
    case 6: return "6dof";
    case 7: return "slider";
    case 8: return "contact";
    case 9: return "6dof spring";
    case 10: return "gear";
    case 11: return "fixed";
    case 12: return "6dof spring2";
    default: return nullptr;
    }
}
constexpr int kConTypes = 16;

// --- per-unit records (filled by the thread that solves the unit) ----------------------------------------------
struct Unit { // one solveGroup call (a batch of islands, or one island)
    uint64_t total = 0, setup = 0, iter = 0, finish = 0;
    int iters = 0, max_iters = 0;
    int islands = 0, bodies = 0, manifolds = 0, contacts = 0, constraints = 0;
    int cls_bodies[kClsN] = {}, blocks = 0, max_children = 0, vgroups = 0, static_manifolds = 0;
    int rows[5] = {};
    bool rows_ok = false;
    int con_types[kConTypes] = {};
    double pos[3] = {}; // the body with the most compound children (else the first body)
    // contact stats
    int m_count[kMClsN] = {}, m_contacts[kMClsN] = {}, m_empty[kMClsN] = {};
    double m_depth_sum[kMClsN] = {}, m_depth_min[kMClsN] = {};
    int points = 0, points_plausible = 0;
};

// --- window accumulators --------------------------------------------------------------------------------------
constexpr int kSizeBins = 8; // island bodies: 1, 2-3, 4-7, 8-15, 16-31, 32-63, 64-127, 128+
const char* const kSizeBinName[kSizeBins] = {"1", "2-3", "4-7", "8-15", "16-31", "32-63", "64-127", "128+"};
int size_bin(int bodies)
{
    int b = 0;
    while (b < kSizeBins - 1 && bodies >= (2 << b)) b++;
    return b;
}
constexpr int kThreadCounts[4] = {2, 4, 6, 8};
constexpr int kTop = 10;

enum class Mode { kGame, kSplit, kSerial, kParallel };

struct Window {
    Mode mode = Mode::kGame;
    ULONGLONG start_ms = 0;
    uint64_t start_tsc = 0;
    uint64_t substeps = 0;
    uint64_t solve_cyc = 0, build_cyc = 0, process_cyc = 0, flush_cyc = 0, max_solve_cyc = 0, wall_cyc = 0, prep_cyc = 0, disjoint_cyc = 0;
    uint64_t group_cyc = 0, setup_cyc = 0, iter_cyc = 0, finish_cyc = 0;
    uint64_t units = 0, islands = 0, island_bodies = 0, island_manifolds = 0, island_contacts = 0, constraints = 0;
    uint64_t bin_count[kSizeBins] = {}, bin_contacts[kSizeBins] = {}, bin_cyc[kSizeBins] = {}; // bin_cyc: split only
    uint64_t biggest_unit_cyc = 0;                                                         // per substep, summed
    uint64_t lpt_cyc[4] = {};
    uint64_t iters = 0, iter_cap = 0, early_exits = 0;
    uint64_t rows[5] = {}, rows_bad = 0;
    uint64_t m_count[kMClsN] = {}, m_contacts[kMClsN] = {}, m_empty[kMClsN] = {};
    double m_depth_sum[kMClsN] = {}, m_depth_min[kMClsN] = {};
    uint64_t points = 0, points_plausible = 0;
    uint64_t con_types[kConTypes] = {};
    uint64_t cls_bodies[kClsN] = {};
    uint64_t kinematic_islands = 0, fallback_steps = 0;
    std::vector<Unit> top; // the costliest units of the window, most expensive first
    int info_iters = -1, info_batch = -1, info_mode = -1, info_split_impulse = -1, override_iters = -1;
    double info_residual = 0;
};

Window g_win;
bool g_log = true, g_split_alternate = true;
Mode g_mode = Mode::kParallel; // configured mode (kGame = off)
int g_threads = 1;             // solver threads including the physics thread

// --- step state (physics thread, except the per-slot unit lists) ------------------------------------------------
constexpr int kMaxSlots = 16;
bool g_active = false;     // inside a recorded / managed solveConstraints
bool g_in_process = false; // inside processIslands (batch flushes there are part of it)
bool g_split_now = false;  // this step runs with batch size 1 (measurement)
bool g_collect = false;    // processIsland collects islands instead of solving them
bool g_own_islands = true;
uint64_t g_own_steps = 0, g_own_checked = 0, g_own_fallback = 0;
int g_pending_islands = 0; // game path: islands handed to processIsland since the last solveGroup
std::vector<Unit> g_slot_units[kMaxSlots];
thread_local Unit* tl_cur = nullptr; // the solveGroup this thread is running now
uint64_t g_build_cyc = 0, g_process_cyc = 0, g_flush_cyc = 0, g_wall_cyc = 0, g_prep_cyc = 0, g_disjoint_cyc = 0;

using SolveConstraints_t = void(__fastcall*)(void* world, void* info);
using BuildIslands_t = void(__fastcall*)(void* mgr, void* dispatcher, void* world);
using ProcessIslands_t = void(__fastcall*)(void* mgr, void* dispatcher, void* world, void* callback);
using ProcessIsland_t = void(__fastcall*)(void* cb, void** bodies, int nbodies, void** manifolds, int nmanifolds, int island);
using Flush_t = void(__fastcall*)(void* cb);
using SolveGroup_t = double(__fastcall*)(void* solver, void** bodies, int nbodies, void** manifolds, int nmanifolds,
                                         void** constraints, int nconstraints, void* info, void* debug, void* dispatcher);
using Setup_t = double(__fastcall*)(void* solver, void** bodies, int nbodies, void** manifolds, int nmanifolds,
                                    void** constraints, int nconstraints, void* info, void* debug);
using Iterations_t = Setup_t;
using Finish_t = double(__fastcall*)(void* solver, void** bodies, int nbodies, void* info);
using SingleIteration_t = double(__fastcall*)(void* solver, int iteration, void** bodies, int nbodies, void** manifolds,
                                              int nmanifolds, void** constraints, int nconstraints, void* info, void* debug);

SolveConstraints_t o_solve_constraints;
BuildIslands_t o_build_islands;
ProcessIslands_t o_process_islands;
ProcessIsland_t o_process_island;
Flush_t o_flush;
SolveGroup_t o_solve_group;
Setup_t o_setup;
Iterations_t o_iterations;
Finish_t o_finish;
SingleIteration_t o_single_iteration;

bool on_physics_thread() { return GetCurrentThreadId() == world_physics_thread(); }

int read_int(const void* base, size_t off)
{
    __try {
        return *reinterpret_cast<const int*>(static_cast<const uint8_t*>(base) + off);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return -1;
    }
}

double read_double(const void* base, size_t off)
{
    __try {
        return *reinterpret_cast<const double*>(static_cast<const uint8_t*>(base) + off);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
}

const uint8_t* read_ptr(const void* base, size_t off)
{
    __try {
        return *reinterpret_cast<const uint8_t* const*>(static_cast<const uint8_t*>(base) + off);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;
    }
}

int manifold_count(const uint8_t* m) { return m ? read_int(m, kManCount) : 0; }

// --- measured solveGroup (any thread) ---------------------------------------------------------------------------
// Records the unit into this slot's list. islands = how many islands the call holds.
double run_unit(int slot, int islands, void* solver, void** bodies, int nbodies, void** manifolds, int nmanifolds,
                void** constraints, int nconstraints, void* info, void* debug, void* dispatcher)
{
    if (!g_log) return o_solve_group(solver, bodies, nbodies, manifolds, nmanifolds, constraints, nconstraints, info, debug, dispatcher);
    Unit u;
    u.islands = islands;
    u.bodies = nbodies;
    u.manifolds = nmanifolds;
    u.constraints = nconstraints;
    u.max_iters = std::max(read_int(info, kInfoIterations), read_int(solver, kSolverOverrideIters));

    int vg[64], nvg = 0;
    bool have_pos = false;
    for (int i = 0; i < nbodies; i++) {
        BodyInfo b = classify(static_cast<const uint8_t*>(bodies[i]));
        u.cls_bodies[b.cls]++;
        if (b.cls == kV) {
            u.blocks += b.children;
            int g = body_group(bodies[i]);
            int key = g >= 0 ? g : -1 - i;
            bool seen = false;
            for (int k = 0; k < nvg; k++) seen |= vg[k] == key;
            if (!seen && nvg < 64) vg[nvg++] = key;
        }
        if (!have_pos || b.children > u.max_children) {
            memcpy(u.pos, b.pos, sizeof u.pos);
            have_pos = true;
            u.max_children = std::max(u.max_children, b.children);
        }
    }
    u.vgroups = nvg;

    for (int i = 0; i < nmanifolds; i++) {
        auto m = static_cast<const uint8_t*>(manifolds[i]);
        if (!m) continue;
        const uint8_t* o0 = read_ptr(m, kManBody0);
        const uint8_t* o1 = read_ptr(m, kManBody1);
        BodyInfo b0 = classify(o0), b1 = classify(o1);
        MCls c = manifold_class(b0.cls, b1.cls, o0, o1);
        if (b0.cls == kS || b1.cls == kS) u.static_manifolds++;
        int n = manifold_count(m);
        u.m_count[c]++;
        if (n <= 0) {
            u.m_empty[c]++;
            continue;
        }
        n = std::min(n, 4);
        u.contacts += n;
        u.m_contacts[c] += n;
        for (int p = 0; p < n; p++) {
            double d = read_double(m, kManPoints + p * kPointStride + kPointDistance);
            u.points++;
            if (std::fabs(d) <= 2.0) u.points_plausible++;
            u.m_depth_sum[c] += d;
            u.m_depth_min[c] = std::min(u.m_depth_min[c], d);
        }
    }
    for (int i = 0; i < nconstraints; i++) {
        int t = constraints[i] ? read_int(constraints[i], kConType) : -1;
        if (t >= 0 && t < kConTypes) u.con_types[t]++;
    }

    tl_cur = &u;
    uint64_t t0 = __rdtsc();
    double r = o_solve_group(solver, bodies, nbodies, manifolds, nmanifolds, constraints, nconstraints, info, debug, dispatcher);
    u.total = __rdtsc() - t0;
    tl_cur = nullptr;
    if (nbodies > 0 || nmanifolds > 0 || nconstraints > 0) g_slot_units[slot].push_back(u);
    return r;
}

// --- parallel solve ----------------------------------------------------------------------------------------------
struct Island {
    size_t body_off;
    int nbodies;
    size_t man_off; // into g_man_buf: the island's manifolds that hold contact points
    int nmanifolds;
    void** constraints;
    int nconstraints;
    uint64_t cost;
    bool kinematic;
    int id; // processIsland's island id
    int far_class; // far_physics_island: 0 none, 1 every dynamic body held, 2 mixed
};

struct Work {
    std::vector<void*> bodies, manifolds, constraints;
    int islands = 0;
    uint64_t cost = 0;
    bool held = false; // far physics: an island of held bodies (skipped on hold sub-steps, n x dt on the first)
    void clear()
    {
        bodies.clear();
        manifolds.clear();
        constraints.clear();
        islands = 0;
        cost = 0;
        held = false;
    }
    void add(const Island& is, const std::vector<void*>& body_buf, const std::vector<void*>& man_buf)
    {
        bodies.insert(bodies.end(), body_buf.begin() + is.body_off, body_buf.begin() + is.body_off + is.nbodies);
        manifolds.insert(manifolds.end(), man_buf.begin() + is.man_off, man_buf.begin() + is.man_off + is.nmanifolds);
        constraints.insert(constraints.end(), is.constraints, is.constraints + is.nconstraints);
        islands++;
        cost += is.cost;
    }
};

std::vector<Island> g_islands;
std::vector<void*> g_body_buf;
std::vector<void*> g_man_buf;
std::deque<Work> g_work_pool; // deque: next_work() hands out pointers that must survive later growth
std::vector<Work*> g_order;
bool g_collect_failed = false;
void* g_cb = nullptr;
void* g_solvers[kMaxSlots] = {};
bool g_parallel_broken = false;

// Workers.
std::atomic<int> g_next{0};
int g_nwork = 0;
void *g_info = nullptr, *g_debug = nullptr, *g_dispatcher = nullptr;
constexpr size_t kInfoTimeStep = 0x18;
alignas(16) uint8_t g_info_long[0x100];
int g_long_scale = 0;

int island_of(const uint8_t* constraint)
{
    const uint8_t* a = read_ptr(constraint, kConA);
    int tag = a ? read_int(a, kObjIslandTag) : -1;
    if (tag < 0) {
        const uint8_t* b = read_ptr(constraint, kConB);
        tag = b ? read_int(b, kObjIslandTag) : -1;
    }
    return tag;
}

bool is_kinematic(const uint8_t* obj) { return obj && (read_int(obj, kObjFlags) & kCfKinematic) != 0; }
bool is_dynamic(const uint8_t* obj) { return obj && (read_int(obj, kObjFlags) & (kCfStatic | kCfKinematic)) == 0; }

// --- per-step checks (see the header) ---------------------------------------------------------------------------
constexpr size_t kJointRowsCount = 0x4c, kJointRowsData = 0x58, kJointRowStride = 0x120, kJointRowConstraint = 0x108;
std::atomic<uint64_t> g_chk_steps{0}, g_chk_shared_steps{0}, g_chk_shared_bodies{0}, g_chk_null_rows{0}, g_chk_null_cons{0};
std::atomic<uint64_t> g_merge_steps{0}, g_merge_bodies{0}, g_merge_units{0};
int g_merge_logged = 0; // "solver: islands share" descriptions this session (physics thread)
std::atomic<int> g_chk_logged{0};
alignas(16) uint8_t g_standin_constraint[0x400] = {}; // finish reads / writes a few fields of a constraint

// body -> unit for units_disjoint: open addressing, cleared by bumping the generation (it was a std::unordered_map:
// ~9000 inserts / lookups per sub-step on the physics thread).
struct OwnerSlot {
    const void* key;
    uint32_t gen;
    int unit;
};
std::vector<OwnerSlot> g_owner_tab(1 << 14);
uint32_t g_owner_gen = 0;

// Inserts obj -> unit if absent; returns the unit it maps to.
int owner_of(const void* obj, int unit)
{
    size_t mask = g_owner_tab.size() - 1;
    size_t h = ((reinterpret_cast<uintptr_t>(obj) >> 4) * 0x9E3779B97F4A7C15ull) >> 20;
    for (size_t i = 0;; i++) {
        OwnerSlot& o = g_owner_tab[(h + i) & mask];
        if (o.gen != g_owner_gen) {
            o = {obj, g_owner_gen, unit};
            return unit;
        }
        if (o.key == obj) return o.unit;
    }
}

// Every dynamic body any unit touches -> the unit; true = no body is touched by two units.
template <typename WorkT>
// record = false: only the answer, no logging and no counters (null constraints go to *nulls), for the first look
// before a possible merge.
bool units_disjoint(const std::vector<WorkT*>& order, bool record = true, uint64_t* nulls = nullptr)
{
    size_t entries = 0;
    for (const WorkT* w : order) entries += w->bodies.size() + 2 * (w->manifolds.size() + w->constraints.size());
    size_t cap = g_owner_tab.size();
    while (cap < entries * 2) cap *= 2;
    if (cap != g_owner_tab.size()) g_owner_tab.assign(cap, OwnerSlot{});
    if (++g_owner_gen == 0) { // wrapped: clear stale slots once
        for (auto& o : g_owner_tab) o.gen = 0;
        g_owner_gen = 1;
    }
    uint64_t shared = 0;
    auto mark = [&](const uint8_t* obj, int wi, const char* via, int con_type) {
        if (!is_dynamic(obj)) return;
        int prev = owner_of(obj, wi);
        if (prev == wi) return;
        shared++;
        if (record && g_chk_logged.fetch_add(1) < 20) {
            swlog("solver: body shared by two units (units %d and %d, via %s%s%d): island tag %d, flags 0x%x; this step uses "
                  "the game's own solver", prev, wi, via, con_type >= 0 ? " type " : "", con_type,
                  read_int(obj, kObjIslandTag), read_int(obj, kObjFlags));
        }
    };
    for (int wi = 0; wi < static_cast<int>(order.size()); wi++) {
        const WorkT* w = order[wi];
        for (void* b : w->bodies) mark(static_cast<const uint8_t*>(b), wi, "its bodies", -1);
        for (void* m : w->manifolds) {
            auto mb = static_cast<const uint8_t*>(m);
            mark(read_ptr(mb, kManBody0), wi, "a manifold", -1);
            mark(read_ptr(mb, kManBody1), wi, "a manifold", -1);
        }
        for (void* c : w->constraints) {
            auto cb = static_cast<const uint8_t*>(c);
            if (!cb) {
                if (record) g_chk_null_cons.fetch_add(1, std::memory_order_relaxed);
                else if (nulls) ++*nulls;
                continue;
            }
            int type = read_int(cb, kConType);
            mark(read_ptr(cb, kConA), wi, "a constraint", type);
            mark(read_ptr(cb, kConB), wi, "a constraint", type);
        }
    }
    if (!record) return shared == 0;
    g_chk_steps.fetch_add(1, std::memory_order_relaxed);
    if (shared) {
        g_chk_shared_steps.fetch_add(1, std::memory_order_relaxed);
        g_chk_shared_bodies.fetch_add(shared, std::memory_order_relaxed);
    }
    return shared == 0;
}

// Before finish: every joint row must still point at its constraint.
void check_joint_rows(void* solver, int nbodies)
{
    int n = read_int(solver, kJointRowsCount);
    auto rows = const_cast<uint8_t*>(read_ptr(solver, kJointRowsData));
    if (n <= 0 || !rows) return;
    int bad = 0, first = -1;
    __try {
        for (int i = 0; i < n; i++) {
            auto& c = *reinterpret_cast<void**>(rows + size_t(i) * kJointRowStride + kJointRowConstraint);
            if (!c) {
                if (first < 0) first = i;
                bad++;
                c = g_standin_constraint;
            }
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return;
    }
    if (!bad) return;
    g_chk_null_rows.fetch_add(bad, std::memory_order_relaxed);
    if (g_chk_logged.fetch_add(1) < 20)
        swlog("solver: joint row without its constraint before finish (%d of %d rows, first #%d; unit %d bodies, thread %lu): "
              "pointed at a stand-in so finish can't crash", bad, n, first, nbodies, GetCurrentThreadId());
}

void run_work(int slot)
{
    for (;;) {
        int i = g_next.fetch_add(1, std::memory_order_relaxed);
        if (i >= g_nwork) return;
        Work* w = g_order[i];
        run_unit(slot, w->islands, g_solvers[slot], w->bodies.data(), static_cast<int>(w->bodies.size()), w->manifolds.data(),
                 static_cast<int>(w->manifolds.size()), w->constraints.data(), static_cast<int>(w->constraints.size()),
                 w->held && g_long_scale > 0 ? static_cast<void*>(g_info_long) : g_info, g_debug, g_dispatcher);
    }
}

void run_work_job(int slot, void*) { run_work(slot); }

bool make_solvers(void* game_solver)
{
    if (g_solvers[1] || g_threads <= 1) {
        g_solvers[0] = game_solver;
        return true;
    }
    auto alloc = *game_ptr<void*(__cdecl**)(size_t, int)>(kAlignedAllocFn);
    auto ctor = game_ptr<void*(__fastcall*)(void*)>(kSolverCtor);
    if (!alloc) return false;
    for (int s = 1; s < g_threads; s++) {
        void* mem = alloc(kSolverSize, 16);
        if (!mem) return false;
        g_solvers[s] = ctor(mem);
    }
    g_solvers[0] = game_solver;
    swlog("solver: made %d extra solver instances", g_threads - 1);
    return true;
}

// Why islands share a body: which islands list it, whether one island id came twice, the body's own state. Logged for
// the first steps of a session that merged units.
void describe_shared(size_t units_before)
{
    std::unordered_map<const void*, int> first; // body -> index in g_islands
    std::unordered_map<int, int> id_count;
    size_t dup_bodies = 0, same_island = 0, kin_pairs = 0;
    const void* ex_body = nullptr;
    int ex_a = -1, ex_b = -1;
    for (int i = 0; i < static_cast<int>(g_islands.size()); i++) {
        const Island& is = g_islands[i];
        id_count[is.id]++;
        for (int k = 0; k < is.nbodies; k++) {
            const void* b = g_body_buf[is.body_off + k];
            if (!is_dynamic(static_cast<const uint8_t*>(b))) continue;
            auto [it, fresh] = first.emplace(b, i);
            if (fresh) continue;
            dup_bodies++;
            const Island& o = g_islands[it->second];
            if (o.id == is.id) same_island++;
            if (o.kinematic != is.kinematic) kin_pairs++;
            if (!ex_body) ex_body = b, ex_a = it->second, ex_b = i;
        }
    }
    size_t dup_ids = 0;
    for (auto& [id, c] : id_count) dup_ids += c > 1 ? c - 1 : 0;
    char ex[400] = "";
    if (ex_body) {
        const Island &a = g_islands[ex_a], &b = g_islands[ex_b];
        auto o = static_cast<const uint8_t*>(ex_body);
        snprintf(ex, sizeof ex, "; e.g. body %p (island tag %d, flags 0x%x, activation %d) in island %d (%d bodies, %d manifolds%s) "
                 "and island %d (%d bodies, %d manifolds%s)", ex_body, read_int(o, kObjIslandTag), read_int(o, kObjFlags),
                 read_int(o, 0x1b0), a.id, a.nbodies, a.nmanifolds, a.kinematic ? ", kinematic" : "", b.id, b.nbodies,
                 b.nmanifolds, b.kinematic ? ", kinematic" : "");
    }
    swlog("solver: islands share bodies this step: %zu islands (%zu island ids handed over more than once), %zu bodies listed "
          "by two islands (%zu of them by the same island id twice, %zu between a kinematic and a normal island); units %zu -> "
          "%zu after merging%s", g_islands.size(), dup_ids, dup_bodies, same_island, kin_pairs, units_before, g_order.size(), ex);
}

// Solves the collected islands. False = nothing was solved (the caller runs the game's path).
bool solve_collected(void* cb)
{
    void* info = const_cast<uint8_t*>(read_ptr(cb, kCbInfo));
    void* game_solver = const_cast<uint8_t*>(read_ptr(cb, kCbSolver));
    if (!info || !game_solver || !make_solvers(game_solver)) return false;
    g_info = info;
    g_debug = const_cast<uint8_t*>(read_ptr(cb, kCbDebug));
    g_dispatcher = const_cast<uint8_t*>(read_ptr(cb, kCbDispatcher));

    // keep the copies' settings in line with the world's solver
    for (int s = 1; s < g_threads; s++) {
        auto dst = static_cast<uint8_t*>(g_solvers[s]);
        auto src = static_cast<const uint8_t*>(game_solver);
        memcpy(dst + kSolverOverrideIters, src + kSolverOverrideIters, 4);
        memcpy(dst + kSolverRowFns, src + kSolverRowFns, 3 * sizeof(void*));
    }

    uint64_t tp = __rdtsc();
    // units: kinematic islands together; islands at or over the batch size alone; smaller ones batched
    int batch = read_int(info, kInfoBatch);
    if (batch < 1 || batch > 1000000) batch = 128;
    std::sort(g_islands.begin(), g_islands.end(), [](const Island& a, const Island& b) { return a.cost > b.cost; });
    size_t used = 0;
    auto next_work = [&]() -> Work* {
        if (used == g_work_pool.size()) g_work_pool.emplace_back();
        Work* w = &g_work_pool[used++];
        w->clear();
        return w;
    };
    g_order.clear();
    Work* kin = nullptr;
    Work* small = nullptr;
    for (const Island& is : g_islands) {
        if (is.far_class == 1 && !is.kinematic) { // far physics: alone, so it can be skipped or stepped long
            Work* w = next_work();
            w->add(is, g_body_buf, g_man_buf);
            w->held = true;
            g_order.push_back(w);
        } else if (is.kinematic) {
            if (!kin) g_order.push_back(kin = next_work());
            kin->add(is, g_body_buf, g_man_buf);
            g_win.kinematic_islands++;
        } else if (is.nmanifolds + is.nconstraints >= batch) {
            Work* w = next_work();
            w->add(is, g_body_buf, g_man_buf);
            g_order.push_back(w);
        } else {
            if (!small) g_order.push_back(small = next_work());
            small->add(is, g_body_buf, g_man_buf);
            if (static_cast<int>(small->manifolds.size() + small->constraints.size()) >= batch) small = nullptr;
        }
    }
    std::sort(g_order.begin(), g_order.end(), [](const Work* a, const Work* b) { return a->cost > b->cost; });
    g_nwork = static_cast<int>(g_order.size());
    g_next.store(0, std::memory_order_relaxed);

    uint64_t t0 = __rdtsc();
    // A body shared by two units would be solved twice, each time with only its unit's constraints: not what the
    // game computes (2026-10-22 17:48: shared bodies via hinges on almost every step near ships, then a character and a
    // vehicle went to 1e9..1e13 m in one tick, NaN 4 ticks later). Such a step goes back to the game's own path.
    // Units that share a dynamic body (seen on clients: the same body handed over in two islands) are merged into one
    // unit with each body / manifold / constraint once (solver_units.h); the check below then still has to pass.
    static const solver_units::Access access{
        [](const void* o) { return is_dynamic(static_cast<const uint8_t*>(o)); },
        [](const void* m) -> const void* { return read_ptr(static_cast<const uint8_t*>(m), kManBody0); },
        [](const void* m) -> const void* { return read_ptr(static_cast<const uint8_t*>(m), kManBody1); },
        [](const void* c) -> const void* { return read_ptr(static_cast<const uint8_t*>(c), kConA); },
        [](const void* c) -> const void* { return read_ptr(static_cast<const uint8_t*>(c), kConB); }};
    size_t units_before = g_order.size();
    // The merge builds a hash map over every body reference (0.46 ms per sub-step on the host's AI-ship world, where
    // nothing is ever shared): only when the fast check (open addressing, quiet) finds a shared body.
    solver_units::MergeStats ms;
    uint64_t nulls = 0;
    bool clean = units_disjoint(g_order, false, &nulls);
    if (clean) { // the usual case: one pass, recorded here as a checked step
        g_chk_steps.fetch_add(1, std::memory_order_relaxed);
        if (nulls) g_chk_null_cons.fetch_add(nulls, std::memory_order_relaxed);
    } else {
        far_physics_solver_fallback(); // merged units can't keep held islands apart
        ms = solver_units::merge_shared(g_order, access);
    }
    if (ms.shared_bodies) {
        g_merge_steps.fetch_add(1, std::memory_order_relaxed);
        g_merge_bodies.fetch_add(ms.shared_bodies, std::memory_order_relaxed);
        g_merge_units.fetch_add(ms.merged_units, std::memory_order_relaxed);
        if (g_merge_logged < 5) {
            g_merge_logged++;
            describe_shared(units_before);
        }
        std::sort(g_order.begin(), g_order.end(), [](const Work* a, const Work* b) { return a->cost > b->cost; });
        g_nwork = static_cast<int>(g_order.size());
    }
    g_prep_cyc += t0 - tp;
    bool disjoint = clean || units_disjoint(g_order); // after a merge: the full check, recorded
    uint64_t td = __rdtsc();
    g_disjoint_cyc += td - t0;
    if (!disjoint) return false;
    // far physics: held units sit out the hold sub-steps, and take one n x dt step on the first
    int long_scale = far_physics_long_scale();
    if (far_physics_hold_substep()) {
        int skipped = 0;
        g_order.erase(std::remove_if(g_order.begin(), g_order.end(), [&](const Work* w) { return w->held && ++skipped; }), g_order.end());
        g_nwork = static_cast<int>(g_order.size());
        far_physics_note_units(skipped, 0);
    } else if (long_scale > 0) {
        memcpy(g_info_long, info, sizeof g_info_long);
        double dt;
        memcpy(&dt, g_info_long + kInfoTimeStep, sizeof dt);
        dt *= long_scale;
        memcpy(g_info_long + kInfoTimeStep, &dt, sizeof dt);
        int n = 0;
        for (const Work* w : g_order) n += w->held;
        far_physics_note_units(0, n);
    }
    g_long_scale = long_scale;
    t0 = td;
    int helpers = g_mode == Mode::kParallel ? std::min(g_threads - 1, g_nwork - 1) : 0;
    phys_pool_run(helpers, &run_work_job, nullptr);
    g_wall_cyc += __rdtsc() - t0;
    return true;
}

// --- logging --------------------------------------------------------------------------------------------------
std::string unit_line(const Unit& u, double cyc_per_ms)
{
    char buf[1024];
    int n = snprintf(buf, sizeof buf,
                     "%6.3f ms (setup %.3f iter %.3f finish %.3f) %d/%d iters | %d island%s, %d bodies (V%d C%d D%d S%d G%d; %d blocks,"
                     " biggest %d, %d vehicle groups) | %d manifolds (%d with static), %d contacts | %d joints",
                     u.total / cyc_per_ms, u.setup / cyc_per_ms, u.iter / cyc_per_ms, u.finish / cyc_per_ms, u.iters, u.max_iters,
                     u.islands, u.islands == 1 ? "" : "s", u.bodies, u.cls_bodies[kV], u.cls_bodies[kC], u.cls_bodies[kD],
                     u.cls_bodies[kS], u.cls_bodies[kG], u.blocks, u.max_children, u.vgroups, u.manifolds, u.static_manifolds,
                     u.contacts, u.constraints);
    std::string s(buf, std::min<size_t>(n, sizeof buf - 1));
    std::string types;
    for (int t = 0; t < kConTypes; t++)
        if (u.con_types[t]) {
            const char* name = constraint_name(t);
            snprintf(buf, sizeof buf, "%s%d %s", types.empty() ? "" : ", ", u.con_types[t], name ? name : "?");
            types += buf;
        }
    if (!types.empty()) s += " (" + types + ")";
    if (u.rows_ok)
        snprintf(buf, sizeof buf, " | rows: %d contact, %d friction, %d rolling, %d joint; %d solver bodies", u.rows[1], u.rows[3], u.rows[4],
                 u.rows[2], u.rows[0]);
    else
        snprintf(buf, sizeof buf, " | rows: unreadable");
    s += buf;
    snprintf(buf, sizeof buf, " | at (%.0f, %.0f, %.0f)", u.pos[0], u.pos[1], u.pos[2]);
    s += buf;
    return s;
}

const char* mode_title(Mode m)
{
    switch (m) {
    case Mode::kGame: return "batched as the game does";
    case Mode::kSplit: return "split: every island solved alone";
    case Mode::kSerial: return "this mod's units on one thread";
    default: return "parallel";
    }
}

void log_window(ULONGLONG now)
{
    Window& w = g_win;
    double secs = (now - w.start_ms) / 1000.0;
    if (!w.substeps || secs <= 0) return;
    double cyc_per_ms = double(__rdtsc() - w.start_tsc) / (secs * 1000.0);
    double ss = double(w.substeps);
    auto ms = [&](uint64_t c) { return c / cyc_per_ms / ss; }; // per substep
    double rest = ms(w.solve_cyc) - ms(w.build_cyc) - ms(w.process_cyc) - ms(w.flush_cyc);
    bool managed = w.mode == Mode::kSerial || w.mode == Mode::kParallel;

    char title[64];
    if (w.mode == Mode::kParallel) snprintf(title, sizeof title, "parallel on %d threads", g_threads);
    else snprintf(title, sizeof title, "%s", mode_title(w.mode));
    swlog("solver 5s [%s]: %.1f sub-steps/s, solveConstraints %.3f ms/sub-step (worst %.3f) = buildIslands %.3f + processIslands %.3f"
          " + last batch %.3f + constraint sort / other %.3f; solveGroup %.3f thread-ms (setup %.3f, iterations %.3f, finish %.3f)",
          title, ss / secs, ms(w.solve_cyc), w.max_solve_cyc / cyc_per_ms, ms(w.build_cyc), ms(w.process_cyc), ms(w.flush_cyc), rest,
          ms(w.group_cyc), ms(w.setup_cyc), ms(w.iter_cyc), ms(w.finish_cyc));
    if (managed && g_own_steps) {
        swlog("  own island walk: %s, %llu sub-steps, %llu checked against the game's walk, %llu left to the game", g_own_islands ? "on" : "OFF",
              static_cast<unsigned long long>(g_own_steps), static_cast<unsigned long long>(g_own_checked),
              static_cast<unsigned long long>(g_own_fallback));
        g_own_steps = g_own_checked = g_own_fallback = 0;
    }
    if (managed)
        swlog("  processIslands split: collecting %.3f, building units %.3f, shared-body check %.3f, solving %.3f ms/sub-step of wall"
              " time (%.3f thread-ms of solveGroup); %llu kinematic islands kept in one unit; %llu sub-steps fell back to the game's path",
              ms(w.process_cyc) - ms(w.prep_cyc) - ms(w.disjoint_cyc) - ms(w.wall_cyc), ms(w.prep_cyc), ms(w.disjoint_cyc),
              ms(w.wall_cyc), ms(w.group_cyc), static_cast<unsigned long long>(w.kinematic_islands),
              static_cast<unsigned long long>(w.fallback_steps));
    swlog("  settings: %d iterations (solver override %d), min batch size %d, residual threshold %g, solver mode 0x%x, split impulse %d",
          w.info_iters, w.override_iters, w.info_batch, w.info_residual, w.info_mode, w.info_split_impulse);
    swlog("  per sub-step: %.1f awake islands (%.1f bodies, %.1f manifolds, %.1f contacts), %.1f joints, %.1f solveGroup calls;"
          " bodies by class: V %.1f, C %.1f, D %.1f, S %.1f, G %.1f",
          w.islands / ss, w.island_bodies / ss, w.island_manifolds / ss, w.island_contacts / ss, w.constraints / ss, w.units / ss,
          w.cls_bodies[kV] / ss, w.cls_bodies[kC] / ss, w.cls_bodies[kD] / ss, w.cls_bodies[kS] / ss, w.cls_bodies[kG] / ss);
    std::string bins;
    char buf[256];
    for (int b = 0; b < kSizeBins; b++) {
        if (!w.bin_count[b]) continue;
        if (w.mode == Mode::kSplit)
            snprintf(buf, sizeof buf, "  [%s] %.1f islands %.0f contacts %.3f ms", kSizeBinName[b], w.bin_count[b] / ss,
                     w.bin_contacts[b] / ss, ms(w.bin_cyc[b]));
        else
            snprintf(buf, sizeof buf, "  [%s] %.1f islands %.0f contacts", kSizeBinName[b], w.bin_count[b] / ss, w.bin_contacts[b] / ss);
        bins += buf;
    }
    swlog("  islands by body count, per sub-step:%s", bins.c_str());
    double serial = ms(w.group_cyc);
    swlog("  parallel estimate per sub-step (%s as units, LPT schedule, no overhead): serial %.3f ms, biggest unit %.3f ms (%.0f%%);"
          " 2 threads %.3f, 4 threads %.3f, 6 threads %.3f, 8 threads %.3f",
          w.mode == Mode::kSplit ? "islands" : "solveGroup calls", serial, ms(w.biggest_unit_cyc),
          serial > 0 ? 100.0 * ms(w.biggest_unit_cyc) / serial : 0.0, ms(w.lpt_cyc[0]), ms(w.lpt_cyc[1]), ms(w.lpt_cyc[2]),
          ms(w.lpt_cyc[3]));
    swlog("  iterations: %.1f avg per solveGroup of max %.1f, %.0f%% stopped early on the residual",
          w.units ? double(w.iters) / w.units : 0.0, w.units ? double(w.iter_cap) / w.units : 0.0,
          w.units ? 100.0 * w.early_exits / w.units : 0.0);
    swlog("  solver rows per sub-step: %.0f contact, %.0f friction, %.0f rolling friction, %.0f joint; %.0f solver bodies%s",
          w.rows[1] / ss, w.rows[3] / ss, w.rows[4] / ss, w.rows[2] / ss, w.rows[0] / ss,
          w.rows_bad ? " (some pools unreadable)" : "");
    for (int c = 0; c < kMClsN; c++) {
        if (!w.m_count[c]) continue;
        uint64_t with = w.m_count[c] - w.m_empty[c];
        swlog("  contacts %-22s %7.1f manifolds (%4.1f%% empty) %8.1f points/sub-step, %.2f per manifold, depth avg %+.4f m, deepest %+.3f m",
              kMClsName[c], w.m_count[c] / ss, 100.0 * w.m_empty[c] / w.m_count[c], w.m_contacts[c] / ss,
              with ? double(w.m_contacts[c]) / with : 0.0, w.m_contacts[c] ? w.m_depth_sum[c] / w.m_contacts[c] : 0.0, w.m_depth_min[c]);
    }
    std::string types;
    for (int t = 0; t < kConTypes; t++)
        if (w.con_types[t]) {
            const char* name = constraint_name(t);
            snprintf(buf, sizeof buf, "%s%.1f %s", types.empty() ? "" : ", ", w.con_types[t] / ss, name ? name : "?");
            types += buf;
        }
    swlog("  joints per sub-step: %s", types.empty() ? "none" : types.c_str());
    swlog("  contact layout check: %.1f%% of %llu points have a depth within +-2 m (m_distance1 at point+0xa0)",
          w.points ? 100.0 * w.points_plausible / w.points : 0.0, static_cast<unsigned long long>(w.points));
    for (size_t i = 0; i < w.top.size(); i++) swlog("  top %-2zu %s", i + 1, unit_line(w.top[i], cyc_per_ms).c_str());
}

Mode next_window_mode()
{
    if (g_mode != Mode::kGame && !g_parallel_broken) return g_mode;
    if (!g_split_alternate) return Mode::kGame;
    return g_win.mode == Mode::kSplit ? Mode::kGame : Mode::kSplit;
}

void reset_window(ULONGLONG now, Mode mode)
{
    g_win = Window{};
    g_win.mode = mode;
    g_win.start_ms = now;
    g_win.start_tsc = __rdtsc();
}

// End of a recorded step: fold the step's units (all slots) into the window.
void fold_step(uint64_t solve_cyc)
{
    Window& w = g_win;
    w.substeps++;
    w.solve_cyc += solve_cyc;
    w.max_solve_cyc = std::max(w.max_solve_cyc, solve_cyc);
    w.build_cyc += g_build_cyc;
    w.process_cyc += g_process_cyc;
    w.flush_cyc += g_flush_cyc;
    w.wall_cyc += g_wall_cyc;
    w.prep_cyc += g_prep_cyc;
    w.disjoint_cyc += g_disjoint_cyc;
    std::vector<uint64_t> times;
    for (int s = 0; s < kMaxSlots; s++) {
        for (const Unit& u : g_slot_units[s]) {
            times.push_back(u.total);
            w.units++;
            w.group_cyc += u.total;
            w.setup_cyc += u.setup;
            w.iter_cyc += u.iter;
            w.finish_cyc += u.finish;
            w.iters += u.iters;
            w.iter_cap += u.max_iters;
            if (u.iters < u.max_iters) w.early_exits++;
            w.constraints += u.constraints;
            for (int c = 0; c < kClsN; c++) w.cls_bodies[c] += u.cls_bodies[c];
            for (int r = 0; r < 5; r++) w.rows[r] += u.rows[r];
            if (!u.rows_ok) w.rows_bad++;
            for (int t = 0; t < kConTypes; t++) w.con_types[t] += u.con_types[t];
            for (int c = 0; c < kMClsN; c++) {
                w.m_count[c] += u.m_count[c];
                w.m_contacts[c] += u.m_contacts[c];
                w.m_empty[c] += u.m_empty[c];
                w.m_depth_sum[c] += u.m_depth_sum[c];
                w.m_depth_min[c] = std::min(w.m_depth_min[c], u.m_depth_min[c]);
            }
            w.points += u.points;
            w.points_plausible += u.points_plausible;
            if (w.mode == Mode::kSplit && u.islands == 1) w.bin_cyc[size_bin(u.bodies)] += u.total;
            if (w.top.size() < kTop || u.total > w.top.back().total) {
                auto at = std::upper_bound(w.top.begin(), w.top.end(), u, [](const Unit& a, const Unit& b) { return a.total > b.total; });
                w.top.insert(at, u);
                if (w.top.size() > kTop) w.top.pop_back();
            }
        }
        g_slot_units[s].clear();
    }
    std::sort(times.begin(), times.end(), std::greater<uint64_t>());
    if (!times.empty()) w.biggest_unit_cyc += times[0];
    for (int k = 0; k < 4; k++) {
        uint64_t load[8] = {};
        int n = kThreadCounts[k];
        for (uint64_t t : times) *std::min_element(load, load + n) += t;
        w.lpt_cyc[k] += *std::max_element(load, load + n);
    }
}

// --- detours --------------------------------------------------------------------------------------------------
void __fastcall solve_constraints_detour(void* world, void* info)
{
    bool managed = g_mode != Mode::kGame && !g_parallel_broken;
    if ((!g_log && !managed) || !on_physics_thread()) return o_solve_constraints(world, info);
    ULONGLONG now = GetTickCount64();
    if (!g_win.start_ms) reset_window(now, next_window_mode());
    else if (now - g_win.start_ms >= 5000) {
        if (g_log) log_window(now);
        uint64_t steps = g_chk_steps.exchange(0), shared = g_chk_shared_steps.exchange(0);
        uint64_t refs = g_chk_shared_bodies.exchange(0), rows = g_chk_null_rows.exchange(0), cons = g_chk_null_cons.exchange(0);
        uint64_t msteps = g_merge_steps.exchange(0), mbodies = g_merge_bodies.exchange(0), munits = g_merge_units.exchange(0);
        if (msteps)
            swlog("solver merges 5s: %llu steps had islands sharing a body (%llu shared bodies): %llu units merged into others, "
                  "the rest still in parallel", (unsigned long long)msteps, (unsigned long long)mbodies, (unsigned long long)munits);
        if (steps)
            swlog("solver checks 5s: %llu parallel steps, %llu had a body shared by two units (%llu references; those steps "
                  "used the game's own solver), %llu joint rows without a constraint, %llu null constraints in units",
                  (unsigned long long)steps, (unsigned long long)shared, (unsigned long long)refs, (unsigned long long)rows,
                  (unsigned long long)cons);
        if (shared == 0 && rows == 0) g_chk_logged.store(std::min(g_chk_logged.load(), 10)); // keep room for new cases
        reset_window(now, next_window_mode());
    }
    Window& w = g_win;
    w.info_iters = read_int(info, kInfoIterations);
    w.info_batch = read_int(info, kInfoBatch);
    w.info_mode = read_int(info, kInfoSolverMode);
    w.info_split_impulse = read_int(info, kInfoSplitImpulse);
    w.info_residual = read_double(info, kInfoResidual);

    int saved_batch = w.info_batch;
    g_split_now = w.mode == Mode::kSplit && saved_batch >= 2 && saved_batch < 1000000;
    if (g_split_now) *reinterpret_cast<int*>(static_cast<uint8_t*>(info) + kInfoBatch) = 1;
    g_active = true;
    g_build_cyc = g_process_cyc = g_flush_cyc = g_wall_cyc = g_prep_cyc = g_disjoint_cyc = 0;
    g_pending_islands = 0;
    uint64_t t0 = __rdtsc();
    touch_solver_begin(world);
    o_solve_constraints(world, info);
    touch_solver_end();
    uint64_t cyc = __rdtsc() - t0;
    g_active = false;
    if (g_split_now) *reinterpret_cast<int*>(static_cast<uint8_t*>(info) + kInfoBatch) = saved_batch;
    fold_step(cyc);
}

void __fastcall build_islands_detour(void* mgr, void* dispatcher, void* world)
{
    if (!g_active) return o_build_islands(mgr, dispatcher, world);
    uint64_t t0 = __rdtsc();
    o_build_islands(mgr, dispatcher, world);
    g_build_cyc += __rdtsc() - t0;
}

bool constraints_sorted(void* cb)
{
    auto cons = reinterpret_cast<void* const*>(read_ptr(cb, kCbConstraints));
    int n = read_int(cb, kCbConstraintCount);
    if (n < 0 || (n > 0 && !cons)) return false;
    int prev = -0x7fffffff;
    for (int i = 0; i < n; i++) {
        int id = island_of(static_cast<const uint8_t*>(cons[i]));
        if (id < prev) return false;
        prev = id;
    }
    return true;
}

bool own_process_islands(void* mgr, void* dispatcher, void* world, void* cb);

void __fastcall process_islands_detour(void* mgr, void* dispatcher, void* world, void* callback)
{
    if (!g_active) return o_process_islands(mgr, dispatcher, world, callback);
    bool managed = (g_win.mode == Mode::kSerial || g_win.mode == Mode::kParallel) && !g_parallel_broken && !touch_solver_native();
    if (touch_solver_native()) far_physics_solver_fallback();
    g_in_process = true;
    uint64_t t0 = __rdtsc();
    if (managed) {
        g_islands.clear();
        g_body_buf.clear();
        g_man_buf.clear();
        g_collect_failed = false;
        g_cb = callback;
        g_collect = true;
        if (!g_own_islands || !own_process_islands(mgr, dispatcher, world, callback)) {
            if (g_own_islands) g_own_fallback++;
            g_islands.clear(), g_body_buf.clear(), g_man_buf.clear();
            o_process_islands(mgr, dispatcher, world, callback);
        }
        g_collect = false;
        bool sorted = constraints_sorted(callback);
        if (g_collect_failed || !sorted) {
            g_parallel_broken = true;
            g_win.fallback_steps++;
            swlog("solver: island collection didn't check out (%s); back to the game's solver path",
                  g_collect_failed ? "unreadable island" : "constraints not sorted by island");
            far_physics_solver_fallback();
            o_process_islands(mgr, dispatcher, world, callback); // nothing was solved yet: run the game's path
        } else if (!solve_collected(callback)) {
            // Shared bodies this step (or the solver copies couldn't be made): the game solves this step itself.
            if (!g_solvers[0]) {
                g_parallel_broken = true;
                swlog("solver: couldn't set up the solver instances; back to the game's solver path");
            }
            g_win.fallback_steps++;
            far_physics_solver_fallback();
            o_process_islands(mgr, dispatcher, world, callback);
        }
    } else {
        o_process_islands(mgr, dispatcher, world, callback);
    }
    g_process_cyc += __rdtsc() - t0;
    g_in_process = false;
}

void __fastcall flush_detour(void* cb)
{
    if (!g_active || g_in_process) return o_flush(cb);
    uint64_t t0 = __rdtsc();
    o_flush(cb);
    g_flush_cyc += __rdtsc() - t0;
}

// Collects one island (managed modes). The bodies array is the island manager's scratch, reused per island: copied.
bool collect_island(void* cb, void** bodies, int nbodies, void** manifolds, int nmanifolds, int island, int contacts)
{
    Island is{g_body_buf.size(), nbodies, g_man_buf.size(), 0, nullptr, 0, 0, false, island};
    g_body_buf.insert(g_body_buf.end(), bodies, bodies + nbodies);
    // Only manifolds with contact points go to the solver (2026-10-27 host, AI ships: 9140 manifolds per sub-step, 8037
    // of them vehicle-static and 95% of those empty). An empty one makes no rows: solveGroup's setup only gets the solver
    // entries of its two bodies, which an island's dynamic bodies already have (setup initialises every listed body
    // first) and a static body shares as the fixed body, so the solve is the same; the copying, the shared-body check
    // and the setup walk get much shorter. A kinematic body touched only by an empty manifold no longer counts below.
    for (int i = 0; i < nmanifolds; i++)
        if (manifold_count(static_cast<const uint8_t*>(manifolds[i])) > 0) g_man_buf.push_back(manifolds[i]);
    is.nmanifolds = static_cast<int>(g_man_buf.size() - is.man_off);

    // the island's constraints: a contiguous run in the list sorted by island id
    auto cons = reinterpret_cast<void**>(const_cast<uint8_t*>(read_ptr(cb, kCbConstraints)));
    int ncons = read_int(cb, kCbConstraintCount);
    if (ncons < 0 || (ncons > 0 && !cons)) return false;
    int lo = 0, hi = ncons;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (island_of(static_cast<const uint8_t*>(cons[mid])) < island) lo = mid + 1;
        else hi = mid;
    }
    int end = lo;
    while (end < ncons && island_of(static_cast<const uint8_t*>(cons[end])) == island) end++;
    is.constraints = cons ? cons + lo : nullptr;
    is.nconstraints = end - lo;

    for (int i = 0; i < is.nmanifolds && !is.kinematic; i++) {
        auto m = static_cast<const uint8_t*>(g_man_buf[is.man_off + i]);
        is.kinematic = is_kinematic(read_ptr(m, kManBody0)) || is_kinematic(read_ptr(m, kManBody1));
    }
    for (int i = 0; i < is.nconstraints && !is.kinematic; i++) {
        auto c = static_cast<const uint8_t*>(is.constraints[i]);
        is.kinematic = is_kinematic(read_ptr(c, kConA)) || is_kinematic(read_ptr(c, kConB));
    }
    is.cost = uint64_t(nbodies) + 2ull * contacts + 8ull * is.nconstraints + is.nmanifolds;
    is.far_class = far_physics_island(g_body_buf.data() + is.body_off, is.nbodies,
                                [](const void* o) { return is_dynamic(static_cast<const uint8_t*>(o)); });
    g_islands.push_back(is);
    return true;
}

void __fastcall process_island_detour(void* cb, void** bodies, int nbodies, void** manifolds, int nmanifolds, int island)
{
    if (!g_active || island < 0) return o_process_island(cb, bodies, nbodies, manifolds, nmanifolds, island);
    int contacts = 0;
    for (int i = 0; i < nmanifolds; i++) contacts += std::max(0, manifold_count(static_cast<const uint8_t*>(manifolds[i])));
    Window& w = g_win;
    w.islands++;
    w.island_bodies += nbodies;
    w.island_manifolds += nmanifolds;
    w.island_contacts += contacts;
    int b = size_bin(nbodies);
    w.bin_count[b]++;
    w.bin_contacts[b] += contacts;
    if (g_collect) {
        if (!collect_island(cb, bodies, nbodies, manifolds, nmanifolds, island, contacts)) g_collect_failed = true;
        return;
    }
    g_pending_islands++;
    o_process_island(cb, bodies, nbodies, manifolds, nmanifolds, island);
}

constexpr size_t kMgrElemCount = 0x0c, kMgrElems = 0x18, kMgrManCount = 0x2c, kMgrMans = 0x38, kMgrSplit = 0x68;
constexpr size_t kWorldObjsData = 0x18, kObjActivation = 0x1b0;
std::vector<int> g_ow_ids, g_ow_start;
std::vector<void*> g_ow_mans, g_ow_bodies;

bool own_walk(void* mgr, void* world, void* cb)
{
    __try {
        auto m = static_cast<uint8_t*>(mgr);
        if (!m[kMgrSplit]) return false;
        int ne = *reinterpret_cast<int*>(m + kMgrElemCount), nm = *reinterpret_cast<int*>(m + kMgrManCount);
        auto elems = *reinterpret_cast<const int* const*>(m + kMgrElems);
        auto mans = *reinterpret_cast<void* const* const*>(m + kMgrMans);
        auto objs = *reinterpret_cast<uint8_t* const* const*>(static_cast<uint8_t*>(world) + kWorldObjsData);
        if (ne < 0 || nm < 0 || (ne && (!elems || !objs)) || (nm && !mans)) return false;
        // island id range: the elements' ids (union-find roots)
        int maxid = -1;
        for (int i = 0; i < ne; i++) maxid = std::max(maxid, elems[2 * i]);
        g_ow_ids.resize(nm);
        for (int i = 0; i < nm; i++) {
            auto mf = static_cast<const uint8_t*>(mans[i]);
            int id = *reinterpret_cast<const int*>(*reinterpret_cast<const uint8_t* const*>(mf + kManBody0) + kObjIslandTag);
            if (id < 0) id = *reinterpret_cast<const int*>(*reinterpret_cast<const uint8_t* const*>(mf + kManBody1) + kObjIslandTag);
            if (id < 0 || id > maxid) return false; // the game's cursor would stall on it: leave it to the game
            g_ow_ids[i] = id;
        }
        g_ow_start.assign(size_t(maxid) + 2, 0);
        for (int i = 0; i < nm; i++) g_ow_start[g_ow_ids[i] + 1]++;
        for (int k = 1; k < maxid + 2; k++) g_ow_start[k] += g_ow_start[k - 1];
        g_ow_mans.resize(nm);
        {
            std::vector<int>& pos = g_ow_ids; // reuse: write cursor per manifold, computed below
            static std::vector<int> cur;
            cur.assign(g_ow_start.begin(), g_ow_start.end() - 1);
            for (int i = 0; i < nm; i++) g_ow_mans[cur[pos[i]]++] = mans[i];
        }
        for (int i = 0; i < ne;) {
            int id = elems[2 * i];
            g_ow_bodies.clear();
            bool awake = false;
            for (; i < ne && elems[2 * i] == id; i++) {
                uint8_t* o = objs[elems[2 * i + 1]];
                g_ow_bodies.push_back(o);
                int act = *reinterpret_cast<const int*>(o + kObjActivation);
                if (act != 2 && act != 5) awake = true;
            }
            if (!awake) continue;
            int m0 = id >= 0 && id <= maxid ? g_ow_start[id] : 0, m1 = id >= 0 && id <= maxid ? g_ow_start[id + 1] : 0;
            process_island_detour(cb, g_ow_bodies.data(), int(g_ow_bodies.size()), m1 > m0 ? g_ow_mans.data() + m0 : nullptr,
                                  m1 - m0, id);
        }
        return !g_collect_failed;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

struct IslandKey {
    int id;
    std::vector<void*> bodies, mans;
    bool operator==(const IslandKey& o) const { return id == o.id && bodies == o.bodies && mans == o.mans; }
};
std::vector<IslandKey> snapshot_islands()
{
    std::vector<IslandKey> out;
    for (const Island& is : g_islands) {
        IslandKey k{is.id, {g_body_buf.begin() + is.body_off, g_body_buf.begin() + is.body_off + is.nbodies},
                    {g_man_buf.begin() + is.man_off, g_man_buf.begin() + is.man_off + is.nmanifolds}};
        std::sort(k.mans.begin(), k.mans.end());
        out.push_back(std::move(k));
    }
    return out;
}

bool own_process_islands(void* mgr, void* dispatcher, void* world, void* cb)
{
    g_own_steps++;
    if (!own_walk(mgr, world, cb)) return false;
    if (g_own_steps % 64 != 0) return true;
    // check: the game's walk gives the same islands, bodies and manifold sets; this step then uses the game's order
    std::vector<IslandKey> mine = snapshot_islands();
    g_islands.clear(), g_body_buf.clear(), g_man_buf.clear();
    o_process_islands(mgr, dispatcher, world, cb);
    g_own_checked++;
    if (snapshot_islands() != mine) {
        g_own_islands = false;
        swlog("solver: own island walk disagreed with the game's (%zu vs %zu islands); game's walk from now on", mine.size(),
              g_islands.size());
    }
    return true;
}

double __fastcall solve_group_detour(void* solver, void** bodies, int nbodies, void** manifolds, int nmanifolds, void** constraints,
                                     int nconstraints, void* info, void* debug, void* dispatcher)
{
    if (!g_active || tl_cur || !on_physics_thread())
        return o_solve_group(solver, bodies, nbodies, manifolds, nmanifolds, constraints, nconstraints, info, debug, dispatcher);
    g_win.override_iters = read_int(solver, kSolverOverrideIters);
    int islands = g_pending_islands;
    g_pending_islands = 0;
    return run_unit(0, islands, solver, bodies, nbodies, manifolds, nmanifolds, constraints, nconstraints, info, debug, dispatcher);
}

double __fastcall setup_detour(void* solver, void** bodies, int nbodies, void** manifolds, int nmanifolds, void** constraints,
                               int nconstraints, void* info, void* debug)
{
    Unit* u = tl_cur;
    if (!u) {
        double r = o_setup(solver, bodies, nbodies, manifolds, nmanifolds, constraints, nconstraints, info, debug);
        touch_solver_setup(solver,bodies,nbodies,manifolds,nmanifolds);
        return r;
    }
    uint64_t t0 = __rdtsc();
    double r = o_setup(solver, bodies, nbodies, manifolds, nmanifolds, constraints, nconstraints, info, debug);
    u->setup += __rdtsc() - t0;
    touch_solver_setup(solver,bodies,nbodies,manifolds,nmanifolds);
    bool ok = true;
    for (int p = 0; p < 5; p++) {
        int size = read_int(solver, kPoolSize[p]), cap = read_int(solver, kPoolSize[p] + 4);
        if (size < 0 || cap < size) ok = false;
        else u->rows[p] = size;
    }
    u->rows_ok = ok;
    return r;
}

double __fastcall iterations_detour(void* solver, void** bodies, int nbodies, void** manifolds, int nmanifolds, void** constraints,
                                    int nconstraints, void* info, void* debug)
{
    Unit* u = tl_cur;
    if (!u) return o_iterations(solver, bodies, nbodies, manifolds, nmanifolds, constraints, nconstraints, info, debug);
    uint64_t t0 = __rdtsc();
    double r = o_iterations(solver, bodies, nbodies, manifolds, nmanifolds, constraints, nconstraints, info, debug);
    u->iter += __rdtsc() - t0;
    return r;
}

double __fastcall finish_detour(void* solver, void** bodies, int nbodies, void* info)
{
    if (g_active) check_joint_rows(solver, nbodies);
    touch_solver_finish(solver);
    Unit* u = tl_cur;
    if (!u) return o_finish(solver, bodies, nbodies, info);
    uint64_t t0 = __rdtsc();
    double r = o_finish(solver, bodies, nbodies, info);
    u->finish += __rdtsc() - t0;
    return r;
}

double __fastcall single_iteration_detour(void* solver, int iteration, void** bodies, int nbodies, void** manifolds, int nmanifolds,
                                          void** constraints, int nconstraints, void* info, void* debug)
{
    if (Unit* u = tl_cur) u->iters++;
    return o_single_iteration(solver, iteration, bodies, nbodies, manifolds, nmanifolds, constraints, nconstraints, info, debug);
}

bool install()
{
    const wchar_t* ini = settings_ini();
    g_log = GetPrivateProfileIntW(L"solver", L"log", 0, ini) != 0;
    g_split_alternate = GetPrivateProfileIntW(L"solver", L"split_alternate", 1, ini) != 0;
    g_own_islands = GetPrivateProfileIntW(L"solver", L"own_islands", 1, ini) != 0;
    wchar_t mode[32] = L"";
    GetPrivateProfileStringW(L"solver", L"parallel", L"parallel", mode, 32, ini);
    g_mode = !_wcsicmp(mode, L"off") ? Mode::kGame : !_wcsicmp(mode, L"serial") ? Mode::kSerial : Mode::kParallel;
    int threads = static_cast<int>(GetPrivateProfileIntW(L"solver", L"threads", 0, ini));
    int hw = static_cast<int>(std::thread::hardware_concurrency());
    if (threads <= 0) threads = std::min(hw - 2, 6);
    g_threads = std::max(1, std::min({threads, kMaxSlots, phys_pool_threads()}));
    if (g_mode == Mode::kParallel && g_threads == 1) g_mode = Mode::kSerial;
    if (!g_log && g_mode == Mode::kGame) {
        swlog("solver: off in settings.ini");
        return true;
    }
    struct {
        uintptr_t at;
        void* detour;
        void** orig;
    } const hooks[] = {
        {kSolveConstraints, reinterpret_cast<void*>(&solve_constraints_detour), reinterpret_cast<void**>(&o_solve_constraints)},
        {kBuildIslands, reinterpret_cast<void*>(&build_islands_detour), reinterpret_cast<void**>(&o_build_islands)},
        {kProcessIslands, reinterpret_cast<void*>(&process_islands_detour), reinterpret_cast<void**>(&o_process_islands)},
        {kProcessIsland, reinterpret_cast<void*>(&process_island_detour), reinterpret_cast<void**>(&o_process_island)},
        {kFlushBatch, reinterpret_cast<void*>(&flush_detour), reinterpret_cast<void**>(&o_flush)},
        {kSolveGroup, reinterpret_cast<void*>(&solve_group_detour), reinterpret_cast<void**>(&o_solve_group)},
        {kSetup, reinterpret_cast<void*>(&setup_detour), reinterpret_cast<void**>(&o_setup)},
        {kIterations, reinterpret_cast<void*>(&iterations_detour), reinterpret_cast<void**>(&o_iterations)},
        {kFinish, reinterpret_cast<void*>(&finish_detour), reinterpret_cast<void**>(&o_finish)},
        {kSingleIteration, reinterpret_cast<void*>(&single_iteration_detour), reinterpret_cast<void**>(&o_single_iteration)},
    };
    for (const auto& h : hooks)
        if (!hook_game(h.at, h.detour, h.orig)) return false;
    swlog("solver: %s%s, log %s (\"solver 5s\" blocks)",
          g_mode == Mode::kGame ? "game's own path" : g_mode == Mode::kSerial ? "islands collected, solved on the physics thread"
                                                                                 : "islands solved in parallel",
          g_mode == Mode::kParallel ? (" on " + std::to_string(g_threads) + " threads").c_str() : "", g_log ? "on" : "off");
    return true;
}

HookRegistrar reg{{"solver", /*needs_known_build=*/true, &install}};

} // namespace

bool solver_units_mode() { return (g_mode == Mode::kSerial || g_mode == Mode::kParallel) && !g_parallel_broken; }
