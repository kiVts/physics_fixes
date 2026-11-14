
#include "far_logic.h"
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "mt_dispatch.h"
#include "phys_pool.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr uintptr_t kSubStep = 0x1401341b0, kPredict = 0x1401364c0, kIntegrate = 0x140136210, kPredictive = 0x140135d10;
constexpr uintptr_t kApplyDamping = 0x140139de0, kPredictXform = 0x140106ab0, kIntegrateInner = 0x140135d70,
                    kPredictiveInner = 0x140135460;
constexpr size_t kWorldBodiesCount = 0x1c4, kWorldBodiesData = 0x1d0, kWorldLocalTime = 0x200, kWorldSpecRestitution = 0x213;
constexpr size_t kWorldObjCount = 0x0c, kWorldObjData = 0x18;
constexpr size_t kObjOrigin = 0x70, kObjFlags = 0x1a0, kObjInternalType = 0x1f0, kObjUser = 0x1f8, kObjShape = 0x188;
constexpr size_t kBodyXform = 0x10, kBodyInterp = 0x90, kBodyLinVel = 0x390, kBodyAngVel = 0x3b0, kShapeType = 0x08;
constexpr size_t kPairCacheCount = 0xc, kPairCacheData = 0x18, kPairSize = 0x20;
constexpr int kCfStatic = 1, kCfKinematic = 2, kCoRigid = 2, kCoGhost = 4, kShapeCompound = 31;
constexpr double kNearM = 1000.0, kMaxLin = 30.0, kMaxAng = 6.0;
constexpr uint64_t kCooldownTicks = 600, kAbPeriod = 1800, kAbOffset = 900;

using Step_t = void(__fastcall*)(void* world, double dt);
using ApplyDamping_t = void(__fastcall*)(void* body, double dt);
using PredictXform_t = void(__fastcall*)(const void* xform, const void* lin, const void* ang, double dt, void* out);
using Inner_t = void(__fastcall*)(void* world, void** bodies, int count, double dt);
Step_t o_substep = nullptr, o_predict = nullptr, o_integrate = nullptr, o_predictive = nullptr;
ApplyDamping_t g_apply_damping = nullptr;
PredictXform_t g_predict_xform = nullptr;
Inner_t g_integrate_inner = nullptr, g_predictive_inner = nullptr;

int g_setting = 0; // [lod] physics
bool g_broken = false; // a self-check failed: off for the session
std::string g_broken_why;

// candidates from vehicle_mt (update thread) -> physics thread
std::mutex g_cand_mu;
std::vector<far_logic::Cand> g_cand_pub; // sorted by scene object
ULONGLONG g_cand_ms = 0;
std::vector<far_logic::Cand> g_cand;
std::unordered_map<uint32_t, far_logic::PartRecord> g_part_rec; // uncrewed vehicles by id: joined since loading?
std::map<std::string, uint64_t> g_other_hist;                 // what refused groups were joined to (5 s window)

// this tick (physics thread; the narrowphase workers and the solver only read)
uint8_t* g_world = nullptr;
bool g_active = false;
int g_n = 0, g_k = 0;
double g_dt = 0;
std::vector<const void*> g_held;   // sorted Bullet bodies held this tick
std::vector<int> g_held_group;     // group index of g_held[i]
std::vector<void*> g_held_list;    // held bodies in the world's moving-body list (for the inner calls)
std::vector<void*> g_rest;         // the other moving bodies
std::vector<uint8_t> g_release;    // per group: a contact released it (set from any thread: byte stores)
std::vector<std::vector<const void*>> g_group_bodies;
std::unordered_map<const void*, uint64_t> g_cooldown; // body -> until tick
uint64_t g_tick = 0;               // ticks with a step (physics thread)

// stats (5 s windows)
struct Stats {
    uint64_t ticks = 0, active = 0, groups = 0, bodies = 0, refused[far_logic::kRefusalCount] = {}, overlap = 0, released = 0;
    uint64_t mixed = 0, no_cand = 0, n_not3 = 0, ab_off = 0, id_mismatch = 0;
    uint64_t f_cands = 0, f_no_host = 0, f_gate = 0, f_scan = 0, f_objs = 0, f_compounds = 0, f_matched = 0, f_cand_groups = 0,
             f_ok_groups = 0, f_split = 0, f_no_pairs = 0, f_all_dropped = 0, f_held_empty = 0;
    std::atomic<uint64_t> pairs_skipped{0}, release_events{0};
    uint64_t units_skipped = 0, units_long = 0;
    uint64_t chk_near = 0, chk_player = 0;
} g_st;
ULONGLONG g_log_at = 0;
struct Ab {
    uint64_t steps = 0, cycles = 0;
} g_ab[2];
int g_ab_state = -1;
double g_cyc_per_ms = 0;

template <typename T> T rd(const void* p, size_t off) { return *reinterpret_cast<const T*>(static_cast<const uint8_t*>(p) + off); }

bool scene_vehicle_id(const void* scene, uint32_t& id)
{
    __try {
        if (!scene) return false;
        const uint8_t* rec = *reinterpret_cast<const uint8_t* const*>(static_cast<const uint8_t*>(scene) + 0xc0);
        if (!rec || *reinterpret_cast<const uint32_t*>(rec) != 1) return false;
        id = *reinterpret_cast<const uint32_t*>(rec + 4);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

std::unordered_map<uint32_t, bool> g_cand_ids; // candidate vehicle id -> crewed (this tick)

const far_logic::Cand* find_cand(const void* scene)
{
    far_logic::Cand key{reinterpret_cast<uintptr_t>(scene), 0, false};
    auto it = std::lower_bound(g_cand.begin(), g_cand.end(), key);
    return it != g_cand.end() && it->scene == key.scene ? &*it : nullptr;
}

const char* rtti_raw(const void* obj)
{
    static uintptr_t image = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
    __try {
        if (!obj) return nullptr;
        auto vtable = *reinterpret_cast<const uintptr_t* const*>(obj);
        auto col = vtable ? reinterpret_cast<const uint32_t*>(vtable[-1]) : nullptr;
        if (!col || col[0] != 1 || reinterpret_cast<uintptr_t>(col) - col[5] != image) return nullptr;
        const char* n = reinterpret_cast<const char*>(image + col[3] + 0x10);
        return n[0] == '.' && n[1] == '?' ? n + 4 : nullptr;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;
    }
}

std::string rtti_class(const void* obj)
{
    const char* name = rtti_raw(obj);
    if (!name) return "?";
    std::string sname(name);
    size_t at = sname.find('@');
    return at == std::string::npos ? sname : sname.substr(0, at);
}

const char* shape_name(int t)
{
    switch (t) {
    case 0: return "box";
    case 4: return "hull";
    case 8: return "sphere";
    case 10: return "capsule";
    case 13: return "cylinder";
    case 31: return "compound";
    default: return "shape";
    }
}

bool held_index(const void* b, size_t& i)
{
    auto it = std::lower_bound(g_held.begin(), g_held.end(), b);
    if (it == g_held.end() || *it != b) return false;
    i = size_t(it - g_held.begin());
    return true;
}

void broken(const std::string& why)
{
    if (g_broken) return;
    g_broken = true;
    g_broken_why = why;
    swlog("far physics: OFF for this session: %s", why.c_str());
}

// --- choosing the held groups (start of a step) --------------------------------------------------------------------
struct Obj {
    const uint8_t* o;
    int group; // body_group, or -1
    bool rigid, ghost, is_static, kinematic, compound;
};

bool scan_world(const uint8_t* world, std::vector<Obj>& out)
{
    __try {
        out.clear();
        int n = rd<int>(world, kWorldObjCount);
        const uint8_t* const* objs = rd<const uint8_t* const*>(world, kWorldObjData);
        if (n <= 0 || n > 1000000 || !objs) return false;
        for (int i = 0; i < n; i++) {
            const uint8_t* o = objs[i];
            if (!o) continue;
            int type = rd<int>(o, kObjInternalType), flags = rd<int>(o, kObjFlags);
            const uint8_t* shape = rd<const uint8_t*>(o, kObjShape);
            Obj x{o, -1, type == kCoRigid, type == kCoGhost, (flags & kCfStatic) != 0, (flags & kCfKinematic) != 0,
                  shape && rd<int>(shape, kShapeType) == kShapeCompound};
            out.push_back(x);
        }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool read_member(const uint8_t* o, far_logic::HoldMember& m)
{
    __try {
        for (int k = 0; k < 3; k++) m.pos[k] = rd<double>(o, kObjOrigin + 8 * k);
        double lx = rd<double>(o, kBodyLinVel), ly = rd<double>(o, kBodyLinVel + 8), lz = rd<double>(o, kBodyLinVel + 16);
        double ax = rd<double>(o, kBodyAngVel), ay = rd<double>(o, kBodyAngVel + 8), az = rd<double>(o, kBodyAngVel + 16);
        m.lin2 = lx * lx + ly * ly + lz * lz;
        m.ang2 = ax * ax + ay * ay + az * az;
        return std::isfinite(m.lin2) && std::isfinite(m.ang2);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool read_pairs(void* cache, int& n, const uint8_t*& data)
{
    __try {
        n = rd<int>(cache, kPairCacheCount);
        data = rd<const uint8_t*>(cache, kPairCacheData);
        return n >= 0 && n < 4000000 && (n == 0 || data);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

std::vector<Obj> g_objs;
std::vector<const void*> g_crew_parts, g_player_parts;

void choose(const uint8_t* world, const double (*players)[3], int np)
{
    g_held.clear();
    g_held_group.clear();
    g_group_bodies.clear();
    if (!scan_world(world, g_objs)) {
        g_st.f_scan++;
        return;
    }
    g_st.f_objs += g_objs.size();
    lod_parts(g_crew_parts, g_player_parts);
    auto in = [](const std::vector<const void*>& v, const void* p) { return std::binary_search(v.begin(), v.end(), p); };
    // groups: constraint-joined rigid bodies; a body without joints is its own group
    std::unordered_map<int, size_t> gidx;
    std::vector<std::vector<size_t>> groups;
    for (size_t i = 0; i < g_objs.size(); i++) {
        Obj& x = g_objs[i];
        if (!x.rigid || x.is_static) continue;
        x.group = body_group(x.o);
        if (x.group >= 0) {
            auto [it, fresh] = gidx.try_emplace(x.group, groups.size());
            if (fresh) groups.emplace_back();
            groups[it->second].push_back(i);
        }
    }
    for (size_t i = 0; i < g_objs.size(); i++) {
        const Obj& x = g_objs[i];
        if (!x.rigid || x.is_static || x.group >= 0 || !x.compound) continue;
        const far_logic::Cand* c = find_cand(rd<const void*>(x.o, kObjUser));
        if (!c) continue;
        g_st.f_matched++;
        if (c->crewed) groups.push_back({i});
        else far_logic::part_seen(g_part_rec[c->id], g_tick, false); // an uncrewed vehicle on its own
    }
    // the rule, per group
    std::vector<far_logic::HoldMember> members;
    std::vector<std::pair<uint32_t, int>> ids; // per member: vehicle id, kind
    std::vector<int> ok_groups;
    for (size_t g = 0; g < groups.size(); g++) {
        members.clear();
        bool any_candidate = false, readable = true;
        // each body's vehicle: a candidate vehicle's own bodies (compounds published by vehicle_mt) and its component
        // bodies (seat / door boxes) carry its id in their record; a compound's record must agree with what was published
        ids.clear();
        bool mismatch = false;
        for (size_t i : groups[g]) {
            const Obj& x = g_objs[i];
            const void* user = rd<const void*>(x.o, kObjUser);
            uint32_t id = 0;
            bool has_id = scene_vehicle_id(user, id);
            const far_logic::Cand* c = x.compound ? find_cand(user) : nullptr;
            if (c && (!has_id || id != c->id)) {
                mismatch = true;
                has_id = false;
            }
            int kind = 0; // 0 none, 1 crewed candidate, 2 uncrewed candidate
            if (has_id) {
                auto it = g_cand_ids.find(id);
                if (it != g_cand_ids.end()) kind = it->second ? 1 : 2;
            }
            ids.push_back({id, kind});
            any_candidate |= kind == 1;
        }
        if (mismatch) g_st.id_mismatch++;
        for (size_t k = 0; k < groups[g].size(); k++) {
            const Obj& x = g_objs[groups[g][k]];
            far_logic::HoldMember m;
            m.candidate = !mismatch && ids[k].second == 1;
            if (!mismatch && ids[k].second == 2) {
                far_logic::PartRecord& rec = g_part_rec[ids[k].first];
                far_logic::part_seen(rec, g_tick, any_candidate);
                m.part = far_logic::part_ok(rec, g_tick);
            }
            m.crew = in(g_crew_parts, x.o);
            m.player = in(g_player_parts, x.o);
            m.kinematic = x.kinematic;
            auto cd = g_cooldown.find(x.o);
            m.cooldown = cd != g_cooldown.end() && g_tick < cd->second;
            if (!read_member(x.o, m)) readable = false;
            members.push_back(m);
        }
        if (!any_candidate) continue; // not a crewed ship of ours: not counted
        g_st.f_cand_groups++;
        far_logic::HoldRefusal r =
            readable ? far_logic::group_rule(members.data(), members.size(), players, np, kNearM, kMaxLin, kMaxAng) : far_logic::kRefuseOther;
        if (r != far_logic::kHoldOk) {
            g_st.refused[r]++;
            if (r == far_logic::kRefuseOther) // what it was joined to (log)
                for (size_t k = 0; k < groups[g].size(); k++) {
                    const far_logic::HoldMember& m = members[k];
                    if (m.candidate || m.part || m.crew || m.player) continue;
                    const Obj& x = g_objs[groups[g][k]];
                    const void* user = rd<const void*>(x.o, kObjUser);
                    std::string what;
                    if (mismatch) what = "a vehicle body whose record disagrees with its id";
                    else if (ids[k].second == 2)
                        what = g_part_rec[ids[k].first].excluded ? "an uncrewed vehicle seen unjoined since loading" : "an uncrewed vehicle in its first 2 s";
                    else if (x.compound) what = "a vehicle body outside the far set";
                    else {
                        const uint8_t* shape = rd<const uint8_t*>(x.o, kObjShape);
                        what = std::string(shape_name(shape ? rd<int>(shape, kShapeType) : -1)) + " " + rtti_class(user);
                    }
                    g_other_hist[what]++;
                }
            continue;
        }
        ok_groups.push_back(int(g));
    }
    g_st.f_ok_groups += ok_groups.size();
    if (ok_groups.empty()) return;
    // no overlap (last sub-step's broadphase pairs) with any body that isn't held. Repeated until nothing changes: a group
    // dropped here makes its bodies "not held" for the groups that touch it.
    void* cache = mt_last_pair_cache();
    int npairs = 0;
    const uint8_t* pairs = nullptr;
    if (!cache || !read_pairs(cache, npairs, pairs)) { // can't check: hold nothing
        g_st.f_no_pairs++;
        return;
    }
    std::unordered_map<const void*, int> group_of; // tentative body -> index in ok_groups
    for (size_t k = 0; k < ok_groups.size(); k++)
        for (size_t i : groups[ok_groups[k]]) group_of[g_objs[i].o] = int(k);
    std::vector<uint8_t> dropped(ok_groups.size(), 0);
    auto held_now = [&](const void* o) {
        auto it = group_of.find(o);
        return it != group_of.end() && !dropped[it->second];
    };
    for (bool changed = true; changed;) {
        changed = false;
        for (int i = 0; i < npairs; i++) {
            const uint8_t* pr = pairs + size_t(i) * kPairSize;
            const uint8_t* p0 = rd<const uint8_t*>(pr, 0);
            const uint8_t* p1 = rd<const uint8_t*>(pr, 8);
            if (!p0 || !p1) continue;
            const uint8_t* a = rd<const uint8_t*>(p0, 0);
            const uint8_t* b = rd<const uint8_t*>(p1, 0);
            bool ha = held_now(a), hb = held_now(b);
            if (ha == hb) continue;
            const uint8_t* other = ha ? b : a;
            int type = rd<int>(other, kObjInternalType), flags = rd<int>(other, kObjFlags);
            if (type == kCoGhost) continue;
            if (type == kCoRigid && (flags & kCfStatic) && !(flags & kCfKinematic)) continue;
            dropped[group_of[ha ? a : b]] = 1;
            g_st.overlap++;
            changed = true;
        }
    }
    for (size_t k = 0; k < ok_groups.size(); k++) {
        if (dropped[k]) continue;
        g_group_bodies.emplace_back();
        for (size_t i : groups[ok_groups[k]]) {
            g_held.push_back(g_objs[i].o);
            g_group_bodies.back().push_back(g_objs[i].o);
        }
    }
    if (g_held.empty()) g_st.f_all_dropped++;
    std::sort(g_held.begin(), g_held.end());
    g_held_group.assign(g_held.size(), -1);
    for (size_t g = 0; g < g_group_bodies.size(); g++)
        for (const void* b : g_group_bodies[g]) {
            size_t i;
            if (held_index(b, i)) g_held_group[i] = int(g);
        }
    g_release.assign(g_group_bodies.size(), 0);
}

bool split_moving(uint8_t* world)
{
    __try {
        int n = rd<int>(world, kWorldBodiesCount);
        void* const* data = rd<void* const*>(world, kWorldBodiesData);
        if (n < 0 || n > 1000000 || (n && !data)) return false;
        g_held_list.clear();
        g_rest.clear();
        for (int i = 0; i < n; i++) {
            size_t k;
            (held_index(data[i], k) ? g_held_list : g_rest).push_back(data[i]);
        }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

// --- the moving-body list swap -------------------------------------------------------------------------------------
struct Swap {
    uint8_t* world;
    int n;
    void* data;
    Swap(uint8_t* w, std::vector<void*>& with) : world(w)
    {
        n = *reinterpret_cast<int*>(w + kWorldBodiesCount);
        data = *reinterpret_cast<void**>(w + kWorldBodiesData);
        *reinterpret_cast<int*>(w + kWorldBodiesCount) = int(with.size());
        *reinterpret_cast<void**>(w + kWorldBodiesData) = with.empty() ? data : with.data();
    }
    ~Swap()
    {
        *reinterpret_cast<int*>(world + kWorldBodiesCount) = n;
        *reinterpret_cast<void**>(world + kWorldBodiesData) = data;
    }
};

bool mine(void* world) { return g_active && world == g_world; }

constexpr int kParPredictMin = 256, kParPredictChunk = 64;
bool g_par_predict = true;
void* g_pp_world = nullptr;
double g_pp_dt = 0;
int g_pp_n = 0;
std::atomic<int> g_pp_next{0};
uint64_t g_pp_calls = 0, g_pp_par = 0, g_pp_cyc = 0, g_pp_log = 0;

void pp_job(int, void*)
{
    auto bodies = rd<void**>(g_pp_world, kWorldBodiesData);
    for (;;) {
        int i0 = g_pp_next.fetch_add(kParPredictChunk, std::memory_order_relaxed);
        if (i0 >= g_pp_n) return;
        int i1 = std::min(i0 + kParPredictChunk, g_pp_n);
        for (int i = i0; i < i1; i++) {
            auto* p = static_cast<uint8_t*>(bodies[i]);
            if (rd<int>(p, kObjFlags) & (kCfStatic | kCfKinematic)) continue; // as Bullet's loop
            g_apply_damping(p, g_pp_dt);
            g_predict_xform(p + kBodyXform, p + kBodyLinVel, p + kBodyAngVel, g_pp_dt, p + kBodyInterp);
        }
    }
}

void predict_all(void* world, double dt)
{
    int n = rd<int>(world, kWorldBodiesCount);
    g_pp_calls++;
    uint64_t t0 = __rdtsc();
    if (!g_par_predict || n < kParPredictMin || !rd<void**>(world, kWorldBodiesData)) {
        o_predict(world, dt);
    } else {
        g_pp_world = world, g_pp_dt = dt, g_pp_n = n;
        g_pp_next.store(0, std::memory_order_relaxed);
        phys_pool_run(phys_pool_threads() - 1, &pp_job, nullptr);
        g_pp_par++;
    }
    g_pp_cyc += __rdtsc() - t0;
    ULONGLONG now = GetTickCount64();
    if (!g_pp_log) g_pp_log = now;
    if (now - g_pp_log >= 5000 && g_pp_calls) {
        swlog("predict 5s: %llu calls, %.0f%% on the pool, %.1f Mcycles each", static_cast<unsigned long long>(g_pp_calls),
              100.0 * double(g_pp_par) / double(g_pp_calls), double(g_pp_cyc) / double(g_pp_calls) / 1e6);
        g_pp_calls = g_pp_par = g_pp_cyc = 0;
        g_pp_log = now;
    }
}

constexpr size_t kWorldContinuous = 0x48, kBodyCcdThreshold = 0x218, kBodyActivation = 0x1b0;
bool g_par_integrate = true;
Inner_t o_inner = nullptr;
void* g_pi_world = nullptr;
void** g_pi_bodies = nullptr;
double g_pi_dt = 0;
int g_pi_n = 0;
std::atomic<int> g_pi_next{0};
uint64_t g_pi_calls = 0, g_pi_par = 0, g_pi_ccd = 0, g_pi_cyc = 0, g_pi_log = 0;

void pi_job(int, void*)
{
    for (;;) {
        int i0 = g_pi_next.fetch_add(kParPredictChunk, std::memory_order_relaxed);
        if (i0 >= g_pi_n) return;
        o_inner(g_pi_world, g_pi_bodies + i0, std::min(kParPredictChunk, g_pi_n - i0), g_pi_dt);
    }
}

bool ccd_possible(const uint8_t* world, void* const* bodies, int n)
{
    if (!world[kWorldContinuous]) return false;
    for (int i = 0; i < n; i++) {
        auto* p = static_cast<const uint8_t*>(bodies[i]);
        if (rd<double>(p, kBodyCcdThreshold) != 0.0) return true;
    }
    return false;
}

void __fastcall inner_detour(void* world, void** bodies, int n, double dt)
{
    g_pi_calls++;
    uint64_t t0 = __rdtsc();
    if (!g_par_integrate || n < kParPredictMin || !bodies) {
        o_inner(world, bodies, n, dt);
    } else if (ccd_possible(static_cast<const uint8_t*>(world), bodies, n)) {
        g_pi_ccd++;
        o_inner(world, bodies, n, dt);
    } else {
        g_pi_world = world, g_pi_bodies = bodies, g_pi_dt = dt, g_pi_n = n;
        g_pi_next.store(0, std::memory_order_relaxed);
        phys_pool_run(phys_pool_threads() - 1, &pi_job, nullptr);
        g_pi_par++;
    }
    g_pi_cyc += __rdtsc() - t0;
    ULONGLONG now = GetTickCount64();
    if (!g_pi_log) g_pi_log = now;
    if (now - g_pi_log >= 5000 && g_pi_calls) {
        swlog("integrate 5s: %llu calls, %.0f%% on the pool, %.0f%% kept serial for CCD, %.1f Mcycles each",
              static_cast<unsigned long long>(g_pi_calls), 100.0 * double(g_pi_par) / double(g_pi_calls),
              100.0 * double(g_pi_ccd) / double(g_pi_calls), double(g_pi_cyc) / double(g_pi_calls) / 1e6);
        g_pi_calls = g_pi_par = g_pi_ccd = g_pi_cyc = 0;
        g_pi_log = now;
    }
}

void __fastcall substep_detour(void* world, double dt)
{
    if (mine(world)) g_dt = dt;
    o_substep(world, dt);
    if (mine(world)) g_k++;
}

void __fastcall predict_detour(void* world, double dt)
{
    if (!mine(world)) return predict_all(world, dt);
    {
        Swap s(static_cast<uint8_t*>(world), g_rest);
        predict_all(world, dt);
    }
    if (g_k != 0) return;
    double big = dt * g_n;
    for (void* b : g_held_list) {
        if (rd<int>(b, kObjFlags) & (kCfStatic | kCfKinematic)) continue; // as Bullet's loop
        g_apply_damping(b, big);
        auto* p = static_cast<uint8_t*>(b);
        g_predict_xform(p + kBodyXform, p + kBodyLinVel, p + kBodyAngVel, big, p + kBodyInterp);
    }
}

void __fastcall predictive_detour(void* world, double dt)
{
    if (!mine(world)) return o_predictive(world, dt);
    {
        Swap s(static_cast<uint8_t*>(world), g_rest);
        o_predictive(world, dt);
    }
    if (g_k == 0 && !g_held_list.empty())
        g_predictive_inner(world, g_held_list.data(), int(g_held_list.size()), dt * g_n);
}

void __fastcall integrate_detour(void* world, double dt)
{
    if (!mine(world)) return o_integrate(world, dt);
    {
        Swap s(static_cast<uint8_t*>(world), g_rest);
        o_integrate(world, dt);
    }
    if (g_k == 0 && !g_held_list.empty())
        g_integrate_inner(world, g_held_list.data(), int(g_held_list.size()), dt * g_n);
}

void maybe_log()
{
    ULONGLONG now = GetTickCount64();
    if (!g_log_at) g_log_at = now;
    if (now - g_log_at < 5000) return;
    double t = g_st.ticks ? double(g_st.ticks) : 1.0;
    static const char* names[far_logic::kRefusalCount] = {"ok", "no player known", "a player's body", "joined to something else",
                                                          "kinematic", "cooldown after a contact", "near a player", "fast",
                                                          "crew only"};
    std::string refused;
    for (int r = 1; r < far_logic::kRefusalCount; r++)
        if (g_st.refused[r]) {
            char buf[96];
            snprintf(buf, sizeof buf, "%s%s %.1f", refused.empty() ? "" : ", ", names[r], g_st.refused[r] / t);
            refused += buf;
        }
    swlog("far physics 5s: %s; held in %.0f%% of ticks: %.1f groups, %.0f bodies per tick; refused per tick: %s%s, overlapping"
          " something not held %.1f; released by a contact %llu; mixed islands %llu; %.0f pairs/s skipped; units skipped %.1f,"
          " long %.1f per tick; self-checks: held near a player %llu, player body held %llu, vehicle record mismatches %llu%s",
          g_broken ? "OFF" : g_setting == 3 ? "A/B" : "on", 100.0 * g_st.active / t, g_st.groups / t, g_st.bodies / t,
          refused.empty() ? "none" : refused.c_str(), g_st.no_cand ? " (no candidates in some ticks)" : "", g_st.overlap / t,
          static_cast<unsigned long long>(g_st.released), static_cast<unsigned long long>(g_st.mixed),
          g_st.pairs_skipped.exchange(0) / ((now - g_log_at) / 1000.0), g_st.units_skipped / t, g_st.units_long / t,
          static_cast<unsigned long long>(g_st.chk_near), static_cast<unsigned long long>(g_st.chk_player),
          static_cast<unsigned long long>(g_st.id_mismatch), g_broken ? (" [" + g_broken_why + "]").c_str() : "");
    g_st.f_cands = g_st.f_no_host = g_st.f_gate = g_st.f_scan = g_st.f_objs = g_st.f_compounds = g_st.f_matched = 0;
    g_st.f_cand_groups = g_st.f_ok_groups = g_st.f_split = g_st.f_no_pairs = g_st.f_all_dropped = g_st.f_held_empty = 0;
    if (!g_other_hist.empty()) {
        std::vector<std::pair<uint64_t, std::string>> top;
        for (auto& [k, v] : g_other_hist) top.push_back({v, k});
        std::sort(top.begin(), top.end(), [](auto& a, auto& b) { return a.first > b.first; });
        std::string line;
        for (size_t k = 0; k < top.size() && k < 6; k++) line += (k ? ", " : "") + top[k].second + " " + std::to_string(top[k].first);
        swlog("  far physics: groups refused for being joined to: %s (bodies, 5 s)", line.c_str());
        g_other_hist.clear();
    }
    if (g_setting == 3 && (g_ab[0].steps || g_ab[1].steps)) {
        double c = g_cyc_per_ms > 0 ? g_cyc_per_ms : 1.0;
        swlog("far physics A/B since start (by host tick, 30 s each way): ON %llu steps, %.3f ms per physics step | OFF %llu steps,"
              " %.3f ms per physics step",
              static_cast<unsigned long long>(g_ab[1].steps), g_ab[1].steps ? g_ab[1].cycles / c / g_ab[1].steps : 0.0,
              static_cast<unsigned long long>(g_ab[0].steps), g_ab[0].steps ? g_ab[0].cycles / c / g_ab[0].steps : 0.0);
    }
    g_st.ticks = g_st.active = g_st.groups = g_st.bodies = g_st.overlap = g_st.released = g_st.mixed = g_st.no_cand = 0;
    g_st.units_skipped = g_st.units_long = g_st.n_not3 = g_st.ab_off = g_st.id_mismatch = 0;
    for (auto& r : g_st.refused) r = 0;
    g_log_at = now;
}

bool install()
{
    g_setting = std::clamp(int(GetPrivateProfileIntW(L"lod", L"physics", 0, settings_ini())), 0, 3);
    g_par_predict = GetPrivateProfileIntW(L"lod", L"parallel_predict", 1, settings_ini()) != 0;
    g_par_integrate = GetPrivateProfileIntW(L"lod", L"parallel_integrate", 1, settings_ini()) != 0;
    if (g_setting == 2) g_setting = 1;
    if (!g_setting) {
        swlog("far physics: off ([lod] physics=0)");
        return true;
    }
    LARGE_INTEGER f, a, b;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&a);
    uint64_t r0 = __rdtsc();
    Sleep(20);
    QueryPerformanceCounter(&b);
    g_cyc_per_ms = double(__rdtsc() - r0) / (double(b.QuadPart - a.QuadPart) * 1000.0 / double(f.QuadPart));
    g_apply_damping = game_ptr<ApplyDamping_t>(kApplyDamping);
    g_predict_xform = game_ptr<PredictXform_t>(kPredictXform);
    g_integrate_inner = game_ptr<Inner_t>(kIntegrateInner);
    g_predictive_inner = game_ptr<Inner_t>(kPredictiveInner);
    if (!hook_game(kSubStep, reinterpret_cast<void*>(&substep_detour), reinterpret_cast<void**>(&o_substep)) ||
        !hook_game(kPredict, reinterpret_cast<void*>(&predict_detour), reinterpret_cast<void**>(&o_predict)) ||
        !hook_game(kPredictive, reinterpret_cast<void*>(&predictive_detour), reinterpret_cast<void**>(&o_predictive)) ||
        !hook_game(kIntegrate, reinterpret_cast<void*>(&integrate_detour), reinterpret_cast<void**>(&o_integrate))) {
        swlog("far physics: hooks FAILED, off");
        g_setting = 0;
        return false;
    }
    if (g_par_integrate && !hook_game(kIntegrateInner, reinterpret_cast<void*>(&inner_detour), reinterpret_cast<void**>(&o_inner))) {
        swlog("far physics: parallel integrate hook failed; integrate stays serial");
        g_par_integrate = false;
    }
    swlog("far physics: %s (far AI ships one %s step per tick; never within %.0f m of a player; \"far physics 5s\" lines)",
          g_setting == 3 ? "A/B, on 30 s / off 30 s" : "on", "n x dt", kNearM);
    return true;
}

HookRegistrar reg{{"far physics", /*needs_known_build=*/true, &install}};

} // namespace

// --- interface ----------------------------------------------------------------------------------------------------
void far_physics_candidates(std::vector<far_logic::Cand>& cands)
{
    if (!g_setting) return;
    std::sort(cands.begin(), cands.end());
    std::lock_guard<std::mutex> lock(g_cand_mu);
    g_cand_pub.swap(cands);
    g_cand_ms = GetTickCount64();
}

void far_physics_step_begin(void* world_ptr, double time_step, int max_sub, double fixed)
{
    g_active = false;
    g_k = 0;
    if (!g_setting || g_broken) return;
    auto* world = static_cast<uint8_t*>(world_ptr);
    g_tick++;
    g_st.ticks++;
    maybe_log();
    {
        std::lock_guard<std::mutex> lock(g_cand_mu);
        if (!g_cand_ms || GetTickCount64() - g_cand_ms > 200) g_cand.clear();
        else g_cand = g_cand_pub;
    }
    if (g_cand.empty()) {
        g_st.no_cand++;
        return;
    }
    g_cand_ids.clear();
    for (const far_logic::Cand& c : g_cand) g_cand_ids[c.id] = c.crewed;
    uint32_t ht = 0;
    g_ab_state = -1;
    if (g_setting == 3) {
        if (!host_tick(ht)) {
            g_st.f_no_host++;
            return;
        }
        g_ab_state = (((ht + kAbOffset) / kAbPeriod) & 1) == 0 ? 1 : 0;
        if (!g_ab_state) {
            g_st.ab_off++;
            return;
        }
    }
    g_st.f_cands++;
    if (!solver_units_mode() || !mt_last_pair_cache() || *reinterpret_cast<uint8_t*>(world + kWorldSpecRestitution)) {
        g_st.f_gate++;
        return;
    }
    double local = *reinterpret_cast<double*>(world + kWorldLocalTime);
    int n = far_logic::substeps_for(local, time_step, fixed, max_sub);
    if (n < 2 || n > 4) {
        g_st.n_not3++;
        return;
    }
    double players[kMaxPlayers][3];
    int np = players_snapshot(players, kMaxPlayers);
    choose(world, players, np);
    if (g_held.empty()) {
        g_st.f_held_empty++;
        return;
    }
    if (!split_moving(world)) {
        g_st.f_split++;
        return;
    }
    g_world = world;
    g_n = n;
    g_active = true;
    g_st.active++;
    g_st.groups += g_group_bodies.size();
    g_st.bodies += g_held.size();
}

void far_physics_step_end(void* world, int steps, uint64_t cycles)
{
    if (g_setting == 3 && g_ab_state >= 0) { // counted only in ticks with candidates and a known host tick
        g_ab[g_ab_state].steps++;
        g_ab[g_ab_state].cycles += cycles;
    }
    if (!g_active || world != g_world) return;
    g_active = false;
    if (steps != g_n) broken("Bullet ran " + std::to_string(steps) + " sub-steps, " + std::to_string(g_n) + " were expected");
    // self-checks: no held body near a player, no player body held
    double players[kMaxPlayers][3];
    int np = players_snapshot(players, kMaxPlayers);
    lod_parts(g_crew_parts, g_player_parts);
    for (const void* p : g_player_parts) {
        size_t i;
        if (held_index(p, i)) {
            g_st.chk_player++;
            broken("a player's body was held");
        }
    }
    for (const void* b : g_held) {
        far_logic::HoldMember m;
        if (!read_member(static_cast<const uint8_t*>(b), m)) continue;
        for (int p = 0; p < np; p++) {
            double dx = m.pos[0] - players[p][0], dy = m.pos[1] - players[p][1], dz = m.pos[2] - players[p][2];
            if (dx * dx + dy * dy + dz * dz < kNearM * kNearM * 0.81) { // 900 m: the rule allowed only > 1000 m at the start
                g_st.chk_near++;
                char buf[160];
                snprintf(buf, sizeof buf, "a held body ended a step %.0f m from a player", std::sqrt(dx * dx + dy * dy + dz * dz));
                broken(buf);
            }
        }
    }
    // groups released by a contact: cooldown
    for (size_t g = 0; g < g_release.size(); g++)
        if (g_release[g]) {
            g_st.released++;
            for (const void* b : g_group_bodies[g]) g_cooldown[b] = g_tick + kCooldownTicks;
        }
    if (g_tick % 600 == 0)
        for (auto it = g_cooldown.begin(); it != g_cooldown.end();) it = it->second <= g_tick ? g_cooldown.erase(it) : ++it;
}

int far_physics_pair(const void* a, const void* b)
{
    if (!g_active) return 0;
    size_t ia = 0, ib = 0;
    bool ha = held_index(a, ia), hb = held_index(b, ib);
    if (!ha && !hb) return 0;
    const uint8_t* other = static_cast<const uint8_t*>(ha ? b : a);
    bool other_static = false, other_ghost = false;
    if (ha != hb) {
        int type = rd<int>(other, kObjInternalType), flags = rd<int>(other, kObjFlags);
        other_ghost = type == kCoGhost;
        other_static = type == kCoRigid && (flags & kCfStatic) && !(flags & kCfKinematic);
    }
    switch (far_logic::pair_rule(ha, hb, other_static, other_ghost, g_k > 0)) {
    case far_logic::kPairSkip: g_st.pairs_skipped.fetch_add(1, std::memory_order_relaxed); return 1;
    case far_logic::kPairRunRelease: {
        int g = g_held_group[ha ? ia : ib];
        if (g >= 0 && g < int(g_release.size())) g_release[g] = 1;
        return 0;
    }
    default: return 0;
    }
}

// Solver (solver_log.cpp, physics thread): 0 = no held body, 1 = every dynamic body held, 2 = mixed.
int far_physics_island(void* const* bodies, int n, bool (*is_dynamic)(const void*))
{
    if (!g_active) return 0;
    int held = 0, dynamic = 0;
    for (int i = 0; i < n; i++) {
        if (!is_dynamic(bodies[i])) continue;
        dynamic++;
        size_t k;
        if (held_index(bodies[i], k)) held++;
    }
    if (!held) return 0;
    if (held == dynamic) return 1;
    g_st.mixed++;
    for (int i = 0; i < n; i++) {
        size_t k;
        if (held_index(bodies[i], k) && g_held_group[k] >= 0) g_release[g_held_group[k]] = 1;
    }
    return 2;
}

bool far_physics_hold_substep() { return g_active && g_k > 0; }
int far_physics_long_scale() { return g_active && g_k == 0 ? g_n : 0; }
void far_physics_note_units(int skipped, int long_units)
{
    g_st.units_skipped += skipped;
    g_st.units_long += long_units;
}
void far_physics_solver_fallback()
{
    if (g_active) broken("the solver fell back to the game's own path during a held tick");
}
bool far_physics_on() { return g_setting != 0 && !g_broken; }
