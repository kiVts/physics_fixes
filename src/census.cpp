
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <intrin.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace {

constexpr uintptr_t kStepSimulation = 0x140133ea0;
constexpr size_t kWorldObjCount = 0xc, kWorldObjData = 0x18;
constexpr size_t kObjOrigin = 0x70, kObjProxy = 0x180, kObjShape = 0x188, kObjFlags = 0x1a0, kObjActivation = 0x1b0,
                 kObjInternalType = 0x1f0, kObjUser = 0x1f8;
constexpr size_t kProxyMask = 0xc;
constexpr size_t kProxyGroup = 8;
constexpr size_t kCompoundTree = 0x80, kDbvtLeaves = 0x14;

using Step_t = int(__fastcall*)(void* world, double time_step, int max_sub_steps, double fixed_time_step);
Step_t g_orig = nullptr;
bool g_log = true; // [census] enabled: the "step 5s" / "census 5s" lines (the hook itself always runs)

uint64_t g_calls = 0, g_substeps = 0, g_cycles = 0, g_max_cycles = 0;
ULONGLONG g_window_start = 0;
uint64_t g_window_tsc = 0;
DWORD g_thread = 0;

struct Body {
    int children, activation, group;
    double pos[3];
};

const char* shape_name(int t)
{
    switch (t) {
    case 0: return "box";
    case 4: return "hull";
    case 8: return "sphere";
    case 10: return "capsule";
    case 13: return "cylinder";
    case 21: return "trimesh";
    case 28: return "plane";
    case 31: return "compound";
    default: return nullptr;
    }
}

const char* activation_name(int a)
{
    switch (a) {
    case 1: return "active";
    case 2: return "sleeping";
    case 3: return "wants-sleep";
    case 4: return "never-sleeps";
    case 5: return "disabled";
    default: return "?";
    }
}

const char* rtti_name_raw(const void* obj)
{
    static uintptr_t image = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
    __try {
        if (!obj) return nullptr;
        auto vtable = *reinterpret_cast<const uintptr_t* const*>(obj);
        if (!vtable) return nullptr;
        auto col = reinterpret_cast<const uint32_t*>(vtable[-1]);
        if (!col || col[0] != 1) return nullptr;
        uintptr_t base = reinterpret_cast<uintptr_t>(col) - col[5];
        if (base != image) return nullptr;
        auto name = reinterpret_cast<const char*>(base + col[3] + 0x10);
        if (name[0] != '.' || name[1] != '?' || name[2] != 'A') return nullptr;
        return name;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;
    }
}

std::string class_name(const char* raw)
{
    if (!raw) return "-";
    std::string s(raw + 4);
    size_t at = s.find('@');
    return at == std::string::npos ? s : s.substr(0, at);
}

struct Owner {
    int group = 0, mask = 0;
    const void* user = nullptr;
};

bool read_owner(const uint8_t* o, Owner& w)
{
    __try {
        const uint8_t* proxy = *reinterpret_cast<const uint8_t* const*>(o + kObjProxy);
        if (proxy) {
            w.group = *reinterpret_cast<const int*>(proxy + kProxyGroup);
            w.mask = *reinterpret_cast<const int*>(proxy + kProxyMask);
        }
        w.user = *reinterpret_cast<const void* const*>(o + kObjUser);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

struct Census {
    int objects = 0, bad = 0;
    std::map<uint32_t, int> by_class; // kind << 24 | shape << 8 | activation
    std::vector<Body> compounds;
    std::vector<Body> characters; // big sensor ghosts
    std::vector<Body> seated;     // small sensor ghosts of seated characters (two each)
    long long children_total = 0, children_awake = 0;
    int seated_ghosts = 0;
    // kind, shape, group, mask, object class, owner class -> count
    std::map<std::tuple<char, int, int, int, std::string, std::string>, int> kinds;
};

// Reads one object; false if it isn't readable.
bool read_object(const uint8_t* o, uint8_t& kind, int& shape, int& activation, int& children, double pos[3])
{
    __try {
        int type = *reinterpret_cast<const int*>(o + kObjInternalType);
        int flags = *reinterpret_cast<const int*>(o + kObjFlags);
        activation = *reinterpret_cast<const int*>(o + kObjActivation);
        const uint8_t* proxy = *reinterpret_cast<const uint8_t* const*>(o + kObjProxy);
        int group = proxy ? *reinterpret_cast<const int*>(proxy + kProxyGroup) : 0;
        if (type == 4) {
            GhostOwner owner = ghost_owner(o);
            bool seated = owner == kGhostCharacterSeated;
            kind = (group & 0x1000) ? 'B' : owner == kGhostNotCharacter ? 'X' : seated ? 'g' : 'G';
        } else {
            kind = (flags & 1) ? 'S' : type == 2 ? 'D' : 'O';
        }
        const uint8_t* s = *reinterpret_cast<const uint8_t* const*>(o + kObjShape);
        shape = s ? *reinterpret_cast<const int*>(s + 8) : -1;
        children = -1;
        if (s && shape == 31) {
            const uint8_t* tree = *reinterpret_cast<const uint8_t* const*>(s + kCompoundTree);
            children = tree ? *reinterpret_cast<const int*>(tree + kDbvtLeaves) : -1;
        }
        for (int i = 0; i < 3; i++) pos[i] = reinterpret_cast<const double*>(o + kObjOrigin)[i];
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool read_object_ptr(const uint8_t* const* data, int i, const uint8_t*& o)
{
    __try {
        o = data[i];
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool read_array(const uint8_t* world, int& n, const uint8_t* const*& data)
{
    __try {
        n = *reinterpret_cast<const int*>(world + kWorldObjCount);
        data = *reinterpret_cast<const uint8_t* const* const*>(world + kWorldObjData);
        return n >= 0 && n < 4000000 && (n == 0 || data);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

// "census by distance": vehicles, bodies, parts and characters in rings around the nearest player (lod.cpp: you and every
// remote player). A vehicle is the joined bodies of one group (or a lone body); it counts in the ring of its biggest body.
void log_rings(const Census& c, const std::map<int, int>& group_sizes)
{
    double players[kMaxPlayers][3];
    int remote = 0;
    int np = players_snapshot(players, kMaxPlayers, &remote);
    if (np <= 0) {
        swlog("  by distance: no player position known (not spawned, or the player scan isn't finding them)");
        return;
    }
    struct Ring {
        int vehicles = 0, bodies = 0, characters = 0, seated_ghosts = 0;
        long long parts = 0, awake = 0;
    } r[kDistRings];
    std::map<int, std::pair<int, int>> group_ring; // group -> (children of its biggest body, ring)
    for (const Body& b : c.compounds) {
        int k = player_ring(b.pos);
        if (k < 0) continue;
        Ring& x = r[k];
        x.bodies++;
        x.parts += std::max(b.children, 0);
        if (b.activation != 2 && b.activation != 5) x.awake += std::max(b.children, 0);
        auto gs = b.group >= 0 ? group_sizes.find(b.group) : group_sizes.end();
        if (gs == group_sizes.end() || gs->second < 2) {
            x.vehicles++;
            continue;
        }
        auto [it, fresh] = group_ring.try_emplace(b.group, b.children, k);
        if (!fresh && b.children > it->second.first) it->second = {b.children, k};
    }
    for (auto& [g, v] : group_ring) r[v.second].vehicles++;
    for (const Body& ch : c.characters)
        if (int k = player_ring(ch.pos); k >= 0) r[k].characters++;
    for (const Body& g : c.seated)
        if (int k = player_ring(g.pos); k >= 0) r[k].seated_ghosts++;
    std::string line;
    for (int k = 0; k < kDistRings; k++) {
        char buf[200];
        snprintf(buf, sizeof buf, "%s%s: %d vehicles, %d bodies, %lld parts (%lld awake), %d characters (%d seated)", k ? " | " : "",
                 kDistRingNames[k], r[k].vehicles, r[k].bodies, r[k].parts, r[k].awake, r[k].characters, r[k].seated_ghosts / 2);
        line += buf;
    }
    swlog("  by distance from the nearest of %d players (%d remote; first at (%.0f, %.0f, %.0f)): %s", np, remote, players[0][0],
          players[0][1], players[0][2], line.c_str());
}

void take_census(const void* world_ptr)
{
    const uint8_t* world = static_cast<const uint8_t*>(world_ptr);
    int n = 0;
    const uint8_t* const* data = nullptr;
    if (!read_array(world, n, data)) {
        swlog("census: world object list unreadable");
        return;
    }
    Census c;
    c.objects = n;
    for (int i = 0; i < n; i++) {
        const uint8_t* o = nullptr;
        uint8_t kind;
        int shape, activation, children;
        Body b{};
        if (!read_object_ptr(data, i, o) || !o || !read_object(o, kind, shape, activation, children, b.pos)) {
            c.bad++;
            continue;
        }
        c.by_class[uint32_t(kind) << 24 | uint32_t(shape & 0xffff) << 8 | uint32_t(activation & 0xff)]++;
        b.children = children;
        b.activation = activation;
        b.group = body_group(o);
        if (kind == 'B') c.characters.push_back(b); // one big sensor per character
        if (kind == 'g') {                           // two small sensors per seated character
            c.seated_ghosts++;
            c.seated.push_back(b);
        }
        if (shape != 31 || kind != 'D') {
            Owner w;
            if (read_owner(o, w))
                c.kinds[{char(kind), shape, w.group, w.mask, class_name(rtti_name_raw(o)), class_name(rtti_name_raw(w.user))}]++;
        }
        if (shape == 31 && kind == 'D') {
            c.compounds.push_back(b);
            c.children_total += std::max(children, 0);
            if (activation != 2 && activation != 5) c.children_awake += std::max(children, 0);
        }
    }
    std::map<int, int> group_sizes;
    for (const Body& b : c.compounds)
        if (b.group >= 0) group_sizes[b.group]++;
    int multi = 0, in_multi = 0;
    for (auto& [g, size] : group_sizes)
        if (size > 1) {
            multi++;
            in_multi += size;
        }
    swlog("census 5s: %d collision objects%s, %zu characters (%d seated, %zu not seated), %zu dynamic vehicle bodies with %lld"
          " blocks/parts (%lld in awake bodies); %d vehicles made of several joined bodies (%d bodies)%s",
          c.objects, c.bad ? " (some unreadable)" : "", c.characters.size(), c.seated_ghosts / 2,
          c.characters.size() - std::min<size_t>(c.characters.size(), c.seated_ghosts / 2),
          c.compounds.size(), c.children_total, c.children_awake, multi, in_multi,
          body_groups_valid() ? "" : " [vehicle groups unavailable]");
    log_rings(c, group_sizes);
    std::vector<std::pair<int, uint32_t>> classes;
    for (auto& [k, v] : c.by_class) classes.push_back({v, k});
    std::sort(classes.begin(), classes.end(), [](auto& a, auto& b) { return a.first > b.first; });
    for (size_t i = 0; i < classes.size() && i < 20; i++) {
        uint32_t k = classes[i].second;
        int shape = int((k >> 8) & 0xffff);
        const char* sn = shape_name(shape);
        char sbuf[16];
        if (!sn) {
            snprintf(sbuf, sizeof sbuf, "%d", shape);
            sn = sbuf;
        }
        swlog("  %6d  %c:%-9s %s", classes[i].first, char(k >> 24), sn, activation_name(int(k & 0xff)));
    }
    std::vector<std::pair<int, const decltype(c.kinds)::key_type*>> kinds;
    for (auto& [k, v] : c.kinds) kinds.push_back({v, &k});
    std::sort(kinds.begin(), kinds.end(), [](auto& a, auto& b) { return a.first > b.first; });
    for (size_t i = 0; i < kinds.size() && i < 24; i++) {
        auto& [kind, shape, group, mask, oc, uc] = *kinds[i].second;
        const char* sn = shape_name(shape);
        swlog("  what %6d  %c:%-9s group 0x%-5x mask 0x%-8x %s / owner %s", kinds[i].first, kind, sn ? sn : "?", group, mask,
              oc.c_str(), uc.c_str());
    }
    std::sort(c.compounds.begin(), c.compounds.end(), [](const Body& a, const Body& b) { return a.children > b.children; });
    for (size_t i = 0; i < c.compounds.size() && i < 15; i++) {
        const Body& b = c.compounds[i];
        int nearby = 0;
        for (const Body& ch : c.characters) {
            double dx = ch.pos[0] - b.pos[0], dy = ch.pos[1] - b.pos[1], dz = ch.pos[2] - b.pos[2];
            nearby += dx * dx + dy * dy + dz * dz < 40.0 * 40.0;
        }
        int joined = b.group >= 0 ? group_sizes[b.group] : 1;
        swlog("  body #%-2zu %5d parts at (%.0f, %.0f, %.0f) %-12s %3d characters within 40 m, vehicle of %d joined bodies", i + 1,
              b.children, b.pos[0], b.pos[1], b.pos[2], activation_name(b.activation), nearby, joined);
    }
}

std::atomic<int> g_in_step{0}; // stepSimulation calls running (any scene)

int __fastcall step_detour(void* world, double time_step, int max_sub_steps, double fixed_time_step)
{
    // A joining player far behind: most replayed ticks don't need Bullet (catchup_skip.cpp). The physics task ignores
    // the return value.
    if (catchup_skip_step(world)) return 0;
    g_in_step.fetch_add(1, std::memory_order_acq_rel);
    lod_step_begin(world); // players and the far rule for this step (lod.cpp)
    ULONGLONG now = GetTickCount64();
    DWORD tid = GetCurrentThreadId();
    bool log_now = false;
    if (tid != g_thread) {
        // a different scene's world (main menu vs loaded world): start over
        g_thread = tid;
        g_window_start = 0;
    }
    if (!g_window_start) {
        g_window_start = now;
        g_window_tsc = __rdtsc();
        g_calls = g_substeps = g_cycles = g_max_cycles = 0;
    } else if (g_log && now - g_window_start >= 5000) {
        log_now = true;
    }
    if (log_now) {
        double secs = (now - g_window_start) / 1000.0;
        double cyc_per_ms = double(__rdtsc() - g_window_tsc) / (secs * 1000.0);
        swlog("step 5s [tid %lu]: %.1f stepSimulation calls/s, %.2f sub-steps per call (fixed step %.2f ms), %.2f ms/call avg,"
              " %.2f ms worst, %.1f ms/s total",
              tid, g_calls / secs, g_calls ? double(g_substeps) / g_calls : 0.0, fixed_time_step * 1000.0,
              g_calls ? g_cycles / cyc_per_ms / g_calls : 0.0, g_max_cycles / cyc_per_ms, g_cycles / cyc_per_ms / secs);
        take_census(world);
        g_window_start = now;
        g_window_tsc = __rdtsc();
        g_calls = g_substeps = g_cycles = g_max_cycles = 0;
    }
    world_step_begin(world); // vehicle groups for this step (narrowphase "J" pairs)
    puppet_step_begin(world); // client: which characters are puppets this step (puppet_cull.cpp)
    touch_step_begin(world); // client: what your character touches (network_fixes)
    broadphase_step_begin(world);
    box_contacts_step_begin(world); // closed-form box vs capsule / sphere into the dispatcher (box_contact.cpp)
    fixed_time_step = network_fixed_step(time_step, max_sub_steps, fixed_time_step); // client: finer steps while pushing
    far_physics_step_begin(world, time_step, max_sub_steps, fixed_time_step); // after the groups and the players
    uint64_t t0 = __rdtsc();
    character_launch_begin(world,time_step,fixed_time_step);
    int steps = g_orig(world, time_step, max_sub_steps, fixed_time_step);
    character_launch_end(world);
    uint64_t cyc = __rdtsc() - t0;
    network_step_end(world, fixed_time_step, steps);
    far_physics_step_end(world, steps, cyc);
    if (steps >= 0) puppet_step_end(); // where each character was left (puppet_cull.cpp)
    sanity_after_step(world);
    g_in_step.fetch_sub(1, std::memory_order_acq_rel);
    g_calls++;
    g_substeps += steps > 0 ? steps : 0;
    g_cycles += cyc;
    g_max_cycles = std::max(g_max_cycles, cyc);
    return steps;
}

bool install()
{
    // The step hook always runs: it tells the other parts which thread is the physics thread and rebuilds the
    // vehicle groups (parallel solver, broadphase and narrowphase depend on it). enabled=0 only stops the logging.
    g_log = GetPrivateProfileIntW(L"census", L"enabled", 0, settings_ini()) != 0;
    if (!hook_game(kStepSimulation, reinterpret_cast<void*>(&step_detour), reinterpret_cast<void**>(&g_orig))) return false;
    swlog(g_log ? "census: on (\"step 5s\" and \"census 5s\" lines every 5 s)" : "census: step hook on, logging off");
    return true;
}

HookRegistrar reg{{"census", /*needs_known_build=*/true, &install}};

} // namespace

bool physics_step_running() { return g_in_step.load(std::memory_order_acquire) != 0; }
