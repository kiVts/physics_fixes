#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace far_logic {

constexpr uintptr_t kImpulseVtable = 0x140b82d30;
constexpr uintptr_t kImpulseThresholdVtable = 0x140b62280;
constexpr size_t kEventTarget = 8;
enum EventKind {
    kReplay, // a force for this tick only: copied on run ticks, posted again on skipped ones
    kHolds,  // a setting physics keeps until changed (motor power, collision flags, damping): nothing to repeat
    kUnsafe, // anything else (transforms, constraint frames, velocities, creation...): this vehicle is not decimated
};
inline EventKind event_kind(uintptr_t vtable, size_t* size = nullptr)
{
    switch (vtable) {
    case kImpulseVtable:
        if (size) *size = 0x30;
        return kReplay;
    case kImpulseThresholdVtable:
        if (size) *size = 0x38;
        return kReplay;
    case 0x140b52c58: // event_set_constraint_hinge_power
    case 0x140b59488: // event_set_constraint_slider_power
    case 0x140b6ecf0: // event_set_constraint_slider_power_angular
    case 0x140b61af8: // event_set_constraint_cone_twist_power
    case 0x140b6a0c0: // event_set_collision_flags
    case 0x140b82ab0: // event_set_damping
        return kHolds;
    default:
        return kUnsafe;
    }
}

// A stash: copied events, each stored as a u32 size then its bytes.
inline void stash_add(std::vector<uint8_t>& stash, const void* ev, uint32_t size)
{
    size_t at = stash.size();
    stash.resize(at + 4 + size);
    memcpy(stash.data() + at, &size, 4);
    memcpy(stash.data() + at + 4, ev, size);
}
// Calls fn(bytes, size) for each event; false if the stash is malformed.
template <typename Fn> bool stash_each(const std::vector<uint8_t>& stash, Fn fn)
{
    size_t at = 0;
    while (at < stash.size()) {
        uint32_t size;
        if (stash.size() - at < 4) return false;
        memcpy(&size, stash.data() + at, 4);
        if (size < kEventTarget + 8 || size > 0x100 || stash.size() - at - 4 < size) return false;
        fn(stash.data() + at + 4, size);
        at += 4 + size;
    }
    return true;
}

// Far with hysteresis: far beyond far2 (squared metres); once far, stays far until within near2.
inline bool far_rule(bool was_far, double d2, double far2, double near2) { return d2 > far2 || (was_far && d2 > near2); }

// A unit's key, from the smallest game vehicle id in it: the same on every PC (ids are the network's), spread over its bits.
inline uint64_t unit_key(uint32_t min_vehicle_id) { return (uint64_t(min_vehicle_id) + 1) * 0x9E3779B97F4A7C15ull; }

// Compare mode: each unit alternates between decimated and control (full logic) every period ticks, half of the units
// starting each way, so every unit is its own control and the two groups are the same ships.
inline bool control_now(uint64_t tick, uint64_t key, uint64_t period) { return (((tick / period) + ((key >> 40) & 1)) & 1) != 0; }

// Does the unit run its logic this tick? One tick in n, phase spread by the key.
inline bool runs_this_tick(uint64_t tick, uint64_t key, int n) { return n <= 1 || (tick + (key >> 32)) % uint64_t(n) == 0; }

// A stash may be replayed only when every event's target is still one of the vehicle's body scene objects (sorted).
inline bool stash_valid(const std::vector<uint8_t>& stash, const std::vector<uintptr_t>& scenes)
{
    bool ok = true;
    bool well_formed = stash_each(stash, [&](const uint8_t* ev, uint32_t) {
        uintptr_t target;
        memcpy(&target, ev + kEventTarget, sizeof target);
        if (!target || !std::binary_search(scenes.begin(), scenes.end(), target)) ok = false;
    });
    return well_formed && ok;
}

// Is this vehicle AI-crewed? Its components (any order) against the seats of seated crew and of players (both sorted):
// at least one crew seat and no player seat. hits gets the number of crew seats found, player whether a player's seat is
// one of them.
inline bool ai_crewed(const std::vector<uintptr_t>& components, const std::vector<const void*>& crew_seats,
                      const std::vector<const void*>& player_seats, int* hits = nullptr, bool* player = nullptr)
{
    int crew = 0;
    bool pl = false;
    for (uintptr_t c : components) {
        const void* p = reinterpret_cast<const void*>(c);
        if (std::binary_search(player_seats.begin(), player_seats.end(), p)) pl = true;
        else if (std::binary_search(crew_seats.begin(), crew_seats.end(), p)) crew++;
    }
    if (hits) *hits = crew;
    if (player) *player = pl;
    return crew > 0 && !pl;
}

// AI group: a vehicle without crew and without a player's seat counts as AI when it is within group_m of an AI-crewed
// vehicle (anchors = first body positions). Far from every player, that is the rest of an AI group (its other vehicles).
inline bool near_any(const double p[3], const std::vector<std::array<double, 3>>& anchors, double group_m)
{
    for (const auto& a : anchors) {
        double dx = p[0] - a[0], dy = p[1] - a[1], dz = p[2] - a[2];
        if (dx * dx + dy * dy + dz * dz < group_m * group_m) return true;
    }
    return false;
}

// Multiplayer: the host's far set (vehicle ids allowed to run their logic every nth tick), its tick, mode and n. Sent by
// network_fixes on its channel 11 about 10 times a second; clients decimate exactly those, phased by the host tick.
constexpr uint32_t kSetMagic = 0x31444f4c; // "LOD1"
constexpr uint32_t kSetMaxIds = 250;       // keeps a set inside one UDP packet; the host decimates no more than this
struct FarSet {
    uint32_t tick = 0;
    uint8_t mode = 0, n = 0;
    std::vector<uint32_t> ids; // sorted
};
inline uint32_t set_encode(const FarSet& s, uint8_t* out, uint32_t cap)
{
    uint32_t count = uint32_t(std::min<size_t>(s.ids.size(), kSetMaxIds));
    uint32_t need = 12 + 4 * count;
    if (cap < need) return 0;
    uint16_t c16 = uint16_t(count);
    memcpy(out, &kSetMagic, 4);
    memcpy(out + 4, &s.tick, 4);
    out[8] = s.mode;
    out[9] = s.n;
    memcpy(out + 10, &c16, 2);
    if (count) memcpy(out + 12, s.ids.data(), 4 * count);
    return need;
}
inline bool set_decode(const uint8_t* in, uint32_t size, FarSet& s)
{
    uint32_t magic;
    uint16_t c16;
    if (size < 12) return false;
    memcpy(&magic, in, 4);
    memcpy(&c16, in + 10, 2);
    if (magic != kSetMagic || c16 > kSetMaxIds || size != 12u + 4u * c16 || in[9] < 2 || in[9] > 8 || in[8] < 1 || in[8] > 3) return false;
    memcpy(&s.tick, in + 4, 4);
    s.mode = in[8];
    s.n = in[9];
    s.ids.resize(c16);
    if (c16) memcpy(s.ids.data(), in + 12, 4 * c16);
    return std::is_sorted(s.ids.begin(), s.ids.end());
}

// Host: may it decimate? Every connected peer must run it (a fresh network_fixes report); without a report, only while no
// remote player is in the world (single player, or no network_fixes to tell).
inline bool host_may_decimate(bool report_fresh, int connected, int capable, int remote_players)
{
    if (report_fresh) return connected == capable;
    return remote_players == 0;
}

// Client: is the host's set usable for this tick? Received recently (wall clock) and about this tick (the host is ahead of
// a client by its queue: at most max_ticks either way).
inline bool set_fresh(uint32_t now_ms, uint32_t received_ms, uint32_t tick, uint32_t set_tick, uint32_t max_ms, uint32_t max_ticks)
{
    if (!received_ms || now_ms - received_ms > max_ms) return false;
    uint32_t d = tick > set_tick ? tick - set_tick : set_tick - tick;
    return d <= max_ticks;
}


// Bullet's sub-step count for one stepSimulation call (btDiscreteDynamicsWorld::stepSimulation, double precision).
inline int substeps_for(double local_time, double time_step, double fixed, int max_sub)
{
    if (max_sub == 0 || fixed <= 0) return -1; // variable step: far physics doesn't apply
    double t = local_time + time_step;
    int n = t >= fixed ? int(t / fixed) : 0;
    return n < max_sub ? n : max_sub;
}

// What vehicle_mt.cpp hands far physics each tick: every body (compound scene object) of every vehicle in a decimated T2
// unit that has no player seat and is untainted; crewed = an NPC sits in one of its seats.
struct Cand {
    uintptr_t scene;
    uint32_t id; // the game's vehicle id
    bool crewed;
    bool operator<(const Cand& o) const { return scene < o.scene; }
};

// A joined part (an uncrewed vehicle of an AI ship: turrets and the like are separate vehicles joined by physics joints)
// counts only while it has been joined to a crewed candidate every time it was seen loaded, after a grace period for the
// joints to form at load. Seen alone once: never again this session (by vehicle id). A player's drone that docks to an AI
// ship later was seen alone first.
constexpr uint64_t kPartGraceTicks = 120;
struct PartRecord {
    uint64_t first_seen = 0;
    bool excluded = false;
};
inline void part_seen(PartRecord& r, uint64_t tick, bool joined_to_crewed)
{
    if (!r.first_seen) r.first_seen = tick;
    if (!joined_to_crewed && tick - r.first_seen > kPartGraceTicks) r.excluded = true;
}
inline bool part_ok(const PartRecord& r, uint64_t tick) { return !r.excluded && r.first_seen && tick - r.first_seen > kPartGraceTicks; }

// One rigid body of a constraint-joined group, as the hold rule sees it.
struct HoldMember {
    bool candidate = false; // a body of a crewed candidate vehicle (crewed AI, far, untainted, in T2's decimated set)
    bool part = false;      // a body of an uncrewed vehicle joined to it since loading (part_ok)
    bool crew = false;      // a body of a seated non-player character
    bool player = false;    // a body of a player character (yours or a remote player's)
    bool kinematic = false;
    bool cooldown = false;  // released by a contact recently
    double pos[3] = {};
    double lin2 = 0, ang2 = 0; // squared linear (m/s) and angular (rad/s) speed
};
enum HoldRefusal {
    kHoldOk,
    kRefuseNoPlayers, // no player position known: nothing is held
    kRefusePlayer,    // a player character's body is in the group
    kRefuseOther,     // a body that is neither a candidate vehicle's nor seated crew (anything else, a player's vehicle...)
    kRefuseKinematic,
    kRefuseCooldown,
    kRefuseNear,      // a body within near_m of a player
    kRefuseFast,
    kRefuseNoCandidate, // only crew, no candidate vehicle body
    kRefusalCount
};
// The whole group is held, or not at all. Order matters only for which reason is counted.
inline HoldRefusal group_rule(const HoldMember* m, size_t n, const double (*players)[3], int np, double near_m, double max_lin, double max_ang)
{
    if (np <= 0) return kRefuseNoPlayers;
    bool any_candidate = false;
    for (size_t i = 0; i < n; i++)
        if (m[i].player) return kRefusePlayer;
    for (size_t i = 0; i < n; i++) {
        if (!m[i].candidate && !m[i].part && !m[i].crew) return kRefuseOther;
        if (m[i].kinematic) return kRefuseKinematic;
        if (m[i].cooldown) return kRefuseCooldown;
        any_candidate |= m[i].candidate;
    }
    for (size_t i = 0; i < n; i++)
        for (int p = 0; p < np; p++) {
            double dx = m[i].pos[0] - players[p][0], dy = m[i].pos[1] - players[p][1], dz = m[i].pos[2] - players[p][2];
            double d2 = dx * dx + dy * dy + dz * dz;
            if (!(d2 > near_m * near_m)) return kRefuseNear; // NaN positions count as near
        }
    for (size_t i = 0; i < n; i++)
        if (!(m[i].lin2 < max_lin * max_lin) || !(m[i].ang2 < max_ang * max_ang)) return kRefuseFast;
    return any_candidate ? kHoldOk : kRefuseNoCandidate;
}

// What the narrowphase does with a pair during a held tick. other_* describe the pair's other object when exactly one
// side is held.
enum PairAction { kPairRun, kPairSkip, kPairRunRelease };
inline PairAction pair_rule(bool held_a, bool held_b, bool other_static, bool other_ghost, bool hold_substep)
{
    if (!held_a && !held_b) return kPairRun;
    if (held_a && held_b) return hold_substep ? kPairSkip : kPairRun;
    if (other_static) return hold_substep ? kPairSkip : kPairRun;
    if (other_ghost) return kPairRun; // sensors push nothing
    return kPairRunRelease; // a body that isn't held touches a held one: full rate from the next tick
}

} // namespace far_logic
