
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace {

constexpr uintptr_t kIsHost = 0x140ceb6ff; // byte: 1 host / single player, 0 client
constexpr size_t kWorldObjCount = 0x0c, kWorldObjData = 0x18;
constexpr size_t kObjOrigin = 0x70, kObjShape = 0x188, kObjInternalType = 0x1f0, kObjUser = 0x1f8, kShapeType = 0x08;
constexpr int kCoRigid = 2, kCoGhost = 4, kShapeSphere = 8, kShapeCapsule = 10;
constexpr size_t kSceneOwner = 0xc0, kCharOwnerOffset = 0x5b0, kSceneRigid = 0xb8, kSceneGhost = 0x148;
constexpr size_t kCharLocal = 0x288, kCharSeat = 0x2100; // seat: world.cpp's ghost_owner reads the same
constexpr size_t kBodySlots[] = {0x478, 0x480, 0x488}, kGhostSlots[] = {0x490, 0x498, 0x4a0};

bool g_cull = false, g_cull_ghosts = false, g_host_cull = false;
using RemoteChars_t = int(__cdecl*)(const void** out, int max);
RemoteChars_t g_remote_chars = nullptr;
DWORD g_remote_looked = 0;
constexpr size_t kObjFlags = 0x1a0;
const volatile uint8_t* g_is_host = nullptr;

// Built on the physics thread at the start of each step, read by narrowphase workers during it and by the ghost
// pass (update thread) after it. Two buffers: readers keep the one that was current when they looked.
struct Sets {
    std::vector<const void*> puppet, local, puppet_ghost_scene;
    int local_chars = 0, local_walking = 0;
    bool host = false; // host rule: puppet parts skip only dynamic rigid bodies
};
Sets g_sets[2];
std::atomic<int> g_cur{-1};

// Physics thread: each character's capsule origin where the last step left it.
struct Track {
    double end[3];
    uint64_t step; // step number that end belongs to
};
std::unordered_map<const uint8_t*, Track> g_track;
std::vector<std::pair<const uint8_t*, const uint8_t*>> g_seen; // (character, capsule body) this step
uint64_t g_step_no = 0, g_kept = 0;

// stats (physics thread / relaxed atomics)
std::atomic<uint64_t> g_pairs_skipped{0}, g_ghosts_skipped{0}, g_manifolds_cleared{0};
uint64_t g_steps = 0, g_sum_puppets = 0, g_sum_local = 0, g_sum_parts = 0, g_unmatched = 0;
uint64_t g_own_moved = 0, g_no_own = 0; // own character moved by the game (spared); steps skipped: own character unknown
DWORD g_last_log = 0;

template <typename T> T rd(const void* p, size_t off) { return *reinterpret_cast<const T*>(static_cast<const uint8_t*>(p) + off); }

const uint8_t* owner_of(const uint8_t* bt, const uint8_t* scene, bool ghost)
{
    const uint8_t* back = rd<const uint8_t*>(scene, kSceneOwner);
    if (!back) return nullptr;
    const uint8_t* ch = back - kCharOwnerOffset;
    for (size_t slot : ghost ? kGhostSlots : kBodySlots) {
        const uint8_t* s = rd<const uint8_t*>(ch, slot);
        if (s && (s == scene || (ghost && rd<const uint8_t*>(s, kSceneGhost) == bt))) return ch;
    }
    return nullptr;
}

// A character's Bullet objects (bodies, and its small ghosts if with_ghosts), validated against their user pointer.
void add_parts(const uint8_t* ch, std::vector<const void*>& out, std::vector<const void*>* ghost_scenes, bool with_ghosts)
{
    for (size_t slot : kBodySlots) {
        const uint8_t* scene = rd<const uint8_t*>(ch, slot);
        const uint8_t* bt = scene ? rd<const uint8_t*>(scene, kSceneRigid) : nullptr;
        if (bt && rd<const uint8_t*>(bt, kObjUser) == scene) out.push_back(bt);
    }
    for (size_t slot : kGhostSlots) {
        if (!with_ghosts) break;
        const uint8_t* scene = rd<const uint8_t*>(ch, slot);
        const uint8_t* bt = scene ? rd<const uint8_t*>(scene, kSceneGhost) : nullptr;
        if (!bt) continue;
        out.push_back(bt);
        if (ghost_scenes) ghost_scenes->push_back(scene);
    }
}

const uint8_t* owner_safe(const uint8_t* bt, const uint8_t* scene, bool ghost);

// Teleported by the game since the last step ended (see the header)? Not when first seen or not seen last step.
bool moved_by_game(const uint8_t* ch, const uint8_t* bt)
{
    auto it = g_track.find(ch);
    if (it == g_track.end() || it->second.step + 1 != g_step_no) return false;
    return memcmp(it->second.end, bt + kObjOrigin, sizeof it->second.end) != 0;
}

void build(const uint8_t* world, Sets& s, const void* own)
{
    s.puppet.clear();
    s.local.clear();
    s.puppet_ghost_scene.clear();
    s.local_chars = s.local_walking = 0;
    int n = rd<int>(world, kWorldObjCount);
    const uint8_t* const* objs = rd<const uint8_t* const*>(world, kWorldObjData);
    if (n <= 0 || n > 1000000 || !objs) return;
    std::unordered_map<const uint8_t*, bool> chars;
    for (int i = 0; i < n; i++) {
        const uint8_t* o = objs[i];
        if (!o) continue;
        int type = rd<int>(o, kObjInternalType);
        bool ghost = type == kCoGhost;
        if (type == kCoRigid) {
            const uint8_t* shape = rd<const uint8_t*>(o, kObjShape);
            int st = shape ? rd<int>(shape, kShapeType) : -1;
            if (st != kShapeCapsule && st != kShapeSphere) continue;
        } else if (!ghost) {
            continue;
        }
        const uint8_t* scene = rd<const uint8_t*>(o, kObjUser);
        if (!scene) continue;
        const uint8_t* ch = owner_safe(o, scene, ghost);
        if (!ch) {
            if (!ghost) g_unmatched++;
            continue;
        }
        chars.emplace(ch, rd<int>(ch, kCharLocal) == 0);
    }
    g_seen.clear();
    for (auto& [ch, flag_puppet] : chars) {
        const uint8_t* scene = rd<const uint8_t*>(ch, kBodySlots[0]);
        const uint8_t* bt = scene ? rd<const uint8_t*>(scene, kSceneRigid) : nullptr;
        if (bt && rd<const uint8_t*>(bt, kObjUser) != scene) bt = nullptr;
        if (bt) g_seen.emplace_back(ch, bt);
        bool moved = flag_puppet && bt && moved_by_game(ch, bt);
        bool puppet = moved && ch != own;
        if (moved && !puppet) g_own_moved++;
        if (flag_puppet && !puppet) g_kept++;
        // A puppet's ghosts count only with cull_ghosts: then their pairs are skipped and they don't regather contacts.
        // Without it they run as before (the game may read them for the puppet's animation).
        if (puppet) add_parts(ch, s.puppet, &s.puppet_ghost_scene, g_cull_ghosts);
        else {
            add_parts(ch, s.local, nullptr, true);
            s.local_chars++;
            if (!rd<const void*>(ch, kCharSeat)) s.local_walking++;
        }
        (puppet ? g_sum_puppets : g_sum_local)++;
    }
    std::sort(s.puppet.begin(), s.puppet.end());
    std::sort(s.local.begin(), s.local.end());
    std::sort(s.puppet_ghost_scene.begin(), s.puppet_ghost_scene.end());
    g_sum_parts += s.puppet.size();
}

const uint8_t* owner_safe(const uint8_t* bt, const uint8_t* scene, bool ghost)
{
    __try {
        return owner_of(bt, scene, ghost);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;
    }
}

// Host: the remote players' characters' bodies, and the host's own character's.
void build_host(Sets& s, const void* own)
{
    s.puppet.clear();
    s.local.clear();
    s.puppet_ghost_scene.clear();
    s.local_chars = s.local_walking = 0;
    s.host = true;
    const void* chars[64];
    int n = g_remote_chars(chars, 64);
    for (int i = 0; i < n; i++)
        if (chars[i] != own) add_parts(static_cast<const uint8_t*>(chars[i]), s.puppet, nullptr, false);
    if (own) add_parts(static_cast<const uint8_t*>(own), s.local, nullptr, false);
    g_sum_puppets += uint64_t(n);
    std::sort(s.puppet.begin(), s.puppet.end());
    std::sort(s.local.begin(), s.local.end());
    g_sum_parts += s.puppet.size();
}

bool build_host_safe(Sets& s, const void* own)
{
    __try {
        build_host(s, own);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool remote_chars_api()
{
    if (g_remote_chars) return true;
    DWORD now = GetTickCount();
    if (g_remote_looked && now - g_remote_looked < 5000) return false;
    g_remote_looked = now;
    HMODULE m = GetModuleHandleW(L"network_fixes.dll");
    g_remote_chars = m ? reinterpret_cast<RemoteChars_t>(GetProcAddress(m, "swml_remote_characters")) : nullptr;
    if (g_remote_chars) swlog("puppets: network_fixes' remote characters found (host: their bodies skip dynamic bodies)");
    return g_remote_chars != nullptr;
}

bool build_safe(const uint8_t* world, Sets& s, const void* own)
{
    __try {
        build(world, s, own);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

bool has(const std::vector<const void*>& v, const void* p) { return std::binary_search(v.begin(), v.end(), p); }

void record_ends()
{
    for (auto& [ch, bt] : g_seen) {
        Track& t = g_track[ch];
        memcpy(t.end, bt + kObjOrigin, sizeof t.end);
        t.step = g_step_no;
    }
    if ((g_step_no & 1023) == 0) // characters that left
        for (auto it = g_track.begin(); it != g_track.end();) it = g_step_no - it->second.step > 8 ? g_track.erase(it) : ++it;
}

bool record_ends_safe()
{
    __try {
        record_ends();
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void maybe_log()
{
    DWORD now = GetTickCount();
    if (now - g_last_log < 5000) return;
    if (g_last_log && g_steps && (g_sum_puppets || g_pairs_skipped.load(std::memory_order_relaxed))) {
        swlog("puppets 5s: %.1f puppet characters and %.1f local ones per tick, %.0f puppet parts; skipped %.0f pairs/s and %.0f ghost "
              "updates/s, %llu old contact sets cleared, %llu character bodies without an owner, %.1f flagged as puppets but kept (not moved "
              "by the game); your character moved by the game (not culled) in %.0f%% of steps, %llu steps unculled (your character unknown)",
              double(g_sum_puppets) / g_steps, double(g_sum_local) / g_steps, double(g_sum_parts) / g_steps,
              g_pairs_skipped.exchange(0) / 5.0, g_ghosts_skipped.exchange(0) / 5.0,
              static_cast<unsigned long long>(g_manifolds_cleared.exchange(0)), static_cast<unsigned long long>(g_unmatched / g_steps), double(g_kept) / g_steps,
              100.0 * double(g_own_moved) / g_steps, static_cast<unsigned long long>(g_no_own));
    }
    g_last_log = now;
    g_steps = g_sum_puppets = g_sum_local = g_sum_parts = g_unmatched = g_kept = g_own_moved = g_no_own = 0;
}

bool install()
{
    const wchar_t* ini = settings_ini();
    g_cull = GetPrivateProfileIntW(L"puppets", L"cull", 1, ini) != 0;
    g_cull_ghosts = g_cull && GetPrivateProfileIntW(L"puppets", L"cull_ghosts", 1, ini) != 0;
    g_host_cull = g_cull && GetPrivateProfileIntW(L"puppets", L"host_cull", 1, ini) != 0;
    g_is_host = game_ptr<const volatile uint8_t*>(kIsHost);
    swlog("puppets (host): %s", g_host_cull ? "on (remote players' characters skip pairs with dynamic bodies)" : "off");
    swlog("puppets: %s", !g_cull ? "off in settings.ini"
                         : g_cull_ghosts ? "on (client: other players' and NPCs' characters skip collisions except with you, and their "
                                           "sensor ghosts skip contact gathering)"
                                         : "on (client: collisions only; ghosts as before)");
    return true;
}

HookRegistrar reg{{"puppet cull", /*needs_known_build=*/true, &install}};

} // namespace

void puppet_step_begin(const void* world)
{
    if (g_host_cull && g_is_host && *g_is_host && world) {
        int next = g_cur.load(std::memory_order_relaxed) == 0 ? 1 : 0;
        const void* own = nullptr;
        double own_pos[3];
        local_player_pos(own_pos, &own);
        if (!remote_chars_api() || !build_host_safe(g_sets[next], own)) {
            g_cur.store(-1, std::memory_order_release);
            return;
        }
        g_cur.store(next, std::memory_order_release);
        g_steps++;
        maybe_log();
        return;
    }
    if (!g_cull || !g_is_host || *g_is_host || !world) {
        g_cur.store(-1, std::memory_order_release);
        return;
    }
    g_step_no++;
    const void* own = nullptr;
    double own_pos[3];
    if (!local_player_pos(own_pos, &own) || !own) {
        g_no_own++; // the step number moves on: nobody counts as moved across the gap (moved_by_game)
        g_cur.store(-1, std::memory_order_release);
        return;
    }
    int next = g_cur.load(std::memory_order_relaxed) == 0 ? 1 : 0;
    if (!build_safe(static_cast<const uint8_t*>(world), g_sets[next], own)) {
        g_cur.store(-1, std::memory_order_release);
        return;
    }
    g_sets[next].host = false;
    g_cur.store(next, std::memory_order_release);
    g_steps++;
    maybe_log();
}

void puppet_step_end()
{
    int cur = g_cur.load(std::memory_order_relaxed);
    if (cur < 0 || g_sets[cur].host) return;
    if (!record_ends_safe()) g_seen.clear();
}

bool puppet_skip_pair(const void* a, const void* b)
{
    int cur = g_cur.load(std::memory_order_acquire);
    if (cur < 0) return false;
    const Sets& s = g_sets[cur];
    if (s.puppet.empty()) return false;
    bool pa = has(s.puppet, a), pb = has(s.puppet, b);
    if (!pa && !pb) return false;
    if ((pa && has(s.local, b)) || (pb && has(s.local, a))) return false; // you vs another character: keep
    if (s.host) {
        if (pa && pb) return false;
        const void* other = pa ? b : a;
        if (rd<int>(other, kObjInternalType) != kCoRigid || (rd<int>(other, kObjFlags) & 3)) return false; // ghosts, terrain
    }
    g_pairs_skipped.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void puppet_note_cleared() { g_manifolds_cleared.fetch_add(1, std::memory_order_relaxed); }

bool puppet_ghosts_on() { return g_cull_ghosts; }

int puppet_local_state()
{
    int cur = g_cur.load(std::memory_order_acquire);
    if (cur < 0) return -1;
    const Sets& s = g_sets[cur];
    if (s.host) return -1; // the host rule doesn't track it
    return s.local_chars == 0 ? 0 : s.local_walking ? 2 : 1;
}

bool puppet_skip_ghost(const void* scene_ghost)
{
    if (!g_cull_ghosts) return false;
    int cur = g_cur.load(std::memory_order_acquire);
    if (cur < 0 || !has(g_sets[cur].puppet_ghost_scene, scene_ghost)) return false;
    g_ghosts_skipped.fetch_add(1, std::memory_order_relaxed);
    return true;
}
