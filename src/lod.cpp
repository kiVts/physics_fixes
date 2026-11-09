
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr size_t kWorldObjCount = 0x0c, kWorldObjData = 0x18;
constexpr size_t kObjOrigin = 0x70, kObjShape = 0x188, kObjInternalType = 0x1f0, kObjUser = 0x1f8, kShapeType = 0x08;
constexpr int kCoRigid = 2, kShapeSphere = 8, kShapeCapsule = 10;
constexpr size_t kSceneOwner = 0xc0, kCharOwnerOffset = 0x5b0, kSceneRigid = 0xb8;
constexpr size_t kCharSimulated = 0x288, kCharSeat = 0x2100, kCharPos = 0x68;
constexpr size_t kBodySlots[] = {0x478, 0x480, 0x488};

bool g_on = true, g_crew = false;
double g_far_m = 1200, g_near_m = 1000;

// players: written at each step on the physics thread, read anywhere
SRWLOCK g_players_lock = SRWLOCK_INIT;
double g_players[kMaxPlayers][3];
int g_nplayers = -1, g_nremote = 0;
std::vector<const void*> g_crew_seats, g_player_seats;

// far crew parts: built before the step into the buffer nobody reads, then published (readers keep theirs)
std::vector<const void*> g_parts[2];
std::atomic<int> g_cur{-1};
// physics thread: every seated non-player character's bodies and every player character's bodies (far_physics.cpp)
std::vector<const void*> g_all_crew_parts, g_player_parts;

// physics thread only
std::unordered_set<const uint8_t*> g_far_chars; // far at the last step (hysteresis)
struct Char {
    const uint8_t* ch;
    bool simulated, seated;
    const void* seat;
    double pos[3];
};
std::vector<Char> g_chars;

// stats
std::atomic<uint64_t> g_pairs_skipped{0}, g_cleared{0};
uint64_t g_steps = 0, g_sum_players = 0, g_sum_remote = 0, g_sum_chars = 0, g_sum_crew = 0, g_sum_far_crew = 0, g_sum_parts = 0;
uint64_t g_went_far = 0, g_came_near = 0, g_no_player = 0, g_scan_failed = 0;
DWORD g_last_log = 0;

template <typename T> T rd(const void* p, size_t off) { return *reinterpret_cast<const T*>(static_cast<const uint8_t*>(p) + off); }

const uint8_t* body_owner(const uint8_t* scene)
{
    const uint8_t* back = rd<const uint8_t*>(scene, kSceneOwner);
    if (!back) return nullptr;
    const uint8_t* ch = back - kCharOwnerOffset;
    for (size_t slot : kBodySlots)
        if (rd<const uint8_t*>(ch, slot) == scene) return ch;
    return nullptr;
}

const uint8_t* body_owner_safe(const uint8_t* scene)
{
    __try {
        return body_owner(scene);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return nullptr;
    }
}

// The characters in the world, each once, with what the far rule needs.
void scan(const uint8_t* world)
{
    g_chars.clear();
    int n = rd<int>(world, kWorldObjCount);
    const uint8_t* const* objs = rd<const uint8_t* const*>(world, kWorldObjData);
    if (n <= 0 || n > 1000000 || !objs) return;
    std::unordered_set<const uint8_t*> seen;
    for (int i = 0; i < n; i++) {
        const uint8_t* o = objs[i];
        if (!o || rd<int>(o, kObjInternalType) != kCoRigid) continue;
        const uint8_t* shape = rd<const uint8_t*>(o, kObjShape);
        int st = shape ? rd<int>(shape, kShapeType) : -1;
        if (st != kShapeCapsule && st != kShapeSphere) continue;
        const uint8_t* scene = rd<const uint8_t*>(o, kObjUser);
        const uint8_t* ch = scene ? body_owner_safe(scene) : nullptr;
        if (!ch || !seen.insert(ch).second) continue;
        const void* seat = rd<const void*>(ch, kCharSeat);
        Char c{ch, rd<int>(ch, kCharSimulated) != 0, seat != nullptr, seat, {}};
        const uint8_t* s0 = rd<const uint8_t*>(ch, kBodySlots[0]);
        const uint8_t* bt = s0 ? rd<const uint8_t*>(s0, kSceneRigid) : nullptr;
        const double* p = bt && rd<const uint8_t*>(bt, kObjUser) == s0 ? reinterpret_cast<const double*>(bt + kObjOrigin)
                                                                         : reinterpret_cast<const double*>(ch + kCharPos);
        for (int k = 0; k < 3; k++) c.pos[k] = p[k];
        if (std::isfinite(c.pos[0]) && std::isfinite(c.pos[1]) && std::isfinite(c.pos[2])) g_chars.push_back(c);
    }
}

bool scan_safe(const uint8_t* world)
{
    __try {
        scan(world);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        g_chars.clear();
        return false;
    }
}

void add_parts(const uint8_t* ch, std::vector<const void*>& out)
{
    for (size_t slot : kBodySlots) {
        const uint8_t* scene = rd<const uint8_t*>(ch, slot);
        const uint8_t* bt = scene ? rd<const uint8_t*>(scene, kSceneRigid) : nullptr;
        if (bt && rd<const uint8_t*>(bt, kObjUser) == scene) out.push_back(bt);
    }
}

bool add_parts_safe(const uint8_t* ch, std::vector<const void*>& out)
{
    __try {
        add_parts(ch, out);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

double nearest2(const double pos[3], const double (*players)[3], int n)
{
    double best = INFINITY;
    for (int i = 0; i < n; i++) {
        double dx = pos[0] - players[i][0], dy = pos[1] - players[i][1], dz = pos[2] - players[i][2];
        best = std::min(best, dx * dx + dy * dy + dz * dz);
    }
    return best;
}

void maybe_log()
{
    DWORD now = GetTickCount();
    if (now - g_last_log < 5000) return;
    if (g_last_log && g_steps) {
        double s = double(g_steps);
        swlog("lod 5s: %.1f players (%.1f remote) among %.0f characters; %.1f seated crew, %.1f of them far (> %.0f m, back at %.0f m)%s; "
              "%llu went far, %llu came near; crew cull %s: %.0f parts, %.0f pairs/s skipped, %llu old contact sets cleared%s%s",
              g_sum_players / s, g_sum_remote / s, g_sum_chars / s, g_sum_crew / s, g_sum_far_crew / s, g_far_m, g_near_m,
              g_no_player ? " (no player known in some steps: nothing far)" : "", static_cast<unsigned long long>(g_went_far),
              static_cast<unsigned long long>(g_came_near), g_crew ? "on" : "off", g_sum_parts / s,
              g_pairs_skipped.exchange(0) / 5.0, static_cast<unsigned long long>(g_cleared.exchange(0)),
              g_scan_failed ? " [scan failed in some steps]" : "", g_sum_remote > 8 * g_steps ? " [many remote players: check +0x288]" : "");
    }
    g_last_log = now;
    g_steps = g_sum_players = g_sum_remote = g_sum_chars = g_sum_crew = g_sum_far_crew = g_sum_parts = 0;
    g_went_far = g_came_near = g_no_player = g_scan_failed = 0;
}

bool install()
{
    const wchar_t* ini = settings_ini();
    g_on = GetPrivateProfileIntW(L"lod", L"enabled", 1, ini) != 0;
    g_crew = GetPrivateProfileIntW(L"lod", L"crew", 0, ini) != 0;
    g_far_m = std::max(100, int(GetPrivateProfileIntW(L"lod", L"far_m", 1200, ini)));
    g_near_m = std::clamp(int(GetPrivateProfileIntW(L"lod", L"near_m", 1000, ini)), 50, int(g_far_m));
    swlog(g_on ? "lod: players on host and clients (\"lod 5s\" lines), far beyond %.0f m (near again at %.0f m), seated far crew collisions %s"
               : "lod: off in settings.ini",
          g_far_m, g_near_m, g_crew ? "skipped" : "kept (crew=0)");
    return true;
}

HookRegistrar reg{{"lod", /*needs_known_build=*/true, &install}};

} // namespace

void lod_step_begin(const void* world_ptr)
{
    if (!g_on) return;
    const uint8_t* world = static_cast<const uint8_t*>(world_ptr);
    bool ok = scan_safe(world);
    if (!ok) g_scan_failed++;

    double me[3];
    const void* me_char = nullptr;
    bool have_me = local_player_pos(me, &me_char);
    double players[kMaxPlayers][3];
    int np = 0, nremote = 0;
    for (const Char& c : g_chars)
        if (c.ch == me_char) {
            for (int k = 0; k < 3; k++) me[k] = c.pos[k]; // this step's position
            have_me = true;
        }
    if (have_me) {
        for (int k = 0; k < 3; k++) players[np][k] = me[k];
        np++;
    }
    for (const Char& c : g_chars)
        if (!c.simulated && c.ch != me_char && np < kMaxPlayers) {
            for (int k = 0; k < 3; k++) players[np][k] = c.pos[k];
            np++;
            nremote++;
        }
    std::vector<const void*> crew_seats, player_seats;
    for (const Char& c : g_chars) {
        if (!c.seat) continue;
        bool player = c.ch == me_char || !c.simulated;
        (player ? player_seats : crew_seats).push_back(c.seat);
    }
    std::sort(crew_seats.begin(), crew_seats.end());
    std::sort(player_seats.begin(), player_seats.end());
    AcquireSRWLockExclusive(&g_players_lock);
    g_nplayers = np ? np : -1;
    g_nremote = nremote;
    for (int i = 0; i < np; i++)
        for (int k = 0; k < 3; k++) g_players[i][k] = players[i][k];
    g_crew_seats.swap(crew_seats);
    g_player_seats.swap(player_seats);
    ReleaseSRWLockExclusive(&g_players_lock);

    g_all_crew_parts.clear();
    g_player_parts.clear();
    for (const Char& c : g_chars) {
        bool player = c.ch == me_char || !c.simulated;
        if (player) add_parts_safe(c.ch, g_player_parts);
        else if (c.seated) add_parts_safe(c.ch, g_all_crew_parts);
    }
    std::sort(g_all_crew_parts.begin(), g_all_crew_parts.end());
    std::sort(g_player_parts.begin(), g_player_parts.end());

    // far seated crew
    int cur = g_cur.load(std::memory_order_relaxed);
    std::vector<const void*>& out = g_parts[cur == 0 ? 1 : 0];
    out.clear();
    int crew = 0, far_crew = 0;
    std::unordered_set<const uint8_t*> far_now;
    double far2 = g_far_m * g_far_m, near2 = g_near_m * g_near_m;
    for (const Char& c : g_chars) {
        if (!c.simulated || !c.seated || c.ch == me_char) continue; // players and walking characters always collide
        crew++;
        if (!np) continue;
        double d2 = nearest2(c.pos, players, np);
        bool was = g_far_chars.count(c.ch) != 0;
        if (!(d2 > far2 || (was && d2 > near2))) {
            if (was) g_came_near++;
            continue;
        }
        if (!was) g_went_far++;
        far_now.insert(c.ch);
        far_crew++;
        if (g_crew) add_parts_safe(c.ch, out);
    }
    g_far_chars.swap(far_now);
    std::sort(out.begin(), out.end());
    g_cur.store(cur == 0 ? 1 : 0, std::memory_order_release);

    g_steps++;
    g_sum_players += np;
    g_sum_remote += nremote;
    g_sum_chars += g_chars.size();
    g_sum_crew += crew;
    g_sum_far_crew += far_crew;
    g_sum_parts += out.size();
    if (!np) g_no_player++;
    if (!g_chars.empty() || g_steps > 1) maybe_log();
}

bool lod_skip_pair(const void* a, const void* b)
{
    if (!g_crew) return false;
    int cur = g_cur.load(std::memory_order_acquire);
    if (cur < 0) return false;
    const std::vector<const void*>& s = g_parts[cur];
    if (s.empty() || (!std::binary_search(s.begin(), s.end(), a) && !std::binary_search(s.begin(), s.end(), b))) return false;
    g_pairs_skipped.fetch_add(1, std::memory_order_relaxed);
    return true;
}

void lod_note_cleared() { g_cleared.fetch_add(1, std::memory_order_relaxed); }

int players_snapshot(double out[][3], int max, int* remote)
{
    AcquireSRWLockShared(&g_players_lock);
    int n = g_nplayers;
    for (int i = 0; i < n && i < max; i++)
        for (int k = 0; k < 3; k++) out[i][k] = g_players[i][k];
    if (remote) *remote = g_nremote;
    ReleaseSRWLockShared(&g_players_lock);
    return n < 0 ? -1 : std::min(n, max);
}

int player_ring(const double pos[3])
{
    double p[kMaxPlayers][3];
    int n = players_snapshot(p, kMaxPlayers);
    if (n <= 0) return -1;
    double d2 = nearest2(pos, p, n);
    double d = std::sqrt(d2);
    return d < 500.0 ? 0 : d < 1000.0 ? 1 : d < 2000.0 ? 2 : 3;
}

void lod_seats(std::vector<const void*>& crew, std::vector<const void*>& players)
{
    AcquireSRWLockShared(&g_players_lock);
    crew = g_crew_seats;
    players = g_player_seats;
    ReleaseSRWLockShared(&g_players_lock);
}

void lod_parts(std::vector<const void*>& crew, std::vector<const void*>& players)
{
    crew = g_all_crew_parts;
    players = g_player_parts;
}
