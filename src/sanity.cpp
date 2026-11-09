
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>

namespace {

constexpr size_t kWorldObjCount = 0x0c, kWorldObjData = 0x18;
constexpr size_t kObjOrigin = 0x70, kObjShape = 0x188, kObjFlags = 0x1a0, kObjActivation = 0x1b0, kObjInternalType = 0x1f0,
                 kObjUser = 0x1f8, kShapeType = 0x08;
constexpr double kFarM = 1e6, kJumpM = 100.0;
constexpr int kFollowTicks = 8;

bool g_on = false;
uintptr_t g_base = 0, g_size = 0;

struct Track {
    double pos[3];
    uint64_t good_tick = 0; // last tick it was sane
    double good[3] = {};
    bool bad = false;
    int follow = 0; // ticks still to log after going bad
};
std::unordered_map<const void*, Track> g_tracks;
uint64_t g_tick = 0;
int g_described = 0, g_jumps_logged = 0;
uint64_t g_win_objects = 0, g_win_bad = 0, g_win_new_bad = 0, g_win_jumps = 0, g_win_steps = 0;
ULONGLONG g_win_start = 0;

bool far_or_nan(const double* p)
{
    for (int i = 0; i < 3; i++)
        if (!std::isfinite(p[i]) || std::fabs(p[i]) > kFarM) return true;
    return false;
}

void rtti(const uint8_t* obj, char* name, size_t size)
{
    name[0] = 0;
    __try {
        if (!obj) return;
        auto vt = *reinterpret_cast<const uintptr_t*>(obj);
        if (vt < g_base || vt >= g_base + g_size) return;
        auto col = *reinterpret_cast<const uint8_t* const*>(vt - 8);
        if (reinterpret_cast<uintptr_t>(col) < g_base || reinterpret_cast<uintptr_t>(col) >= g_base + g_size) return;
        uint32_t td = *reinterpret_cast<const uint32_t*>(col + 12);
        strncpy_s(name, size, reinterpret_cast<const char*>(g_base + td + 16), _TRUNCATE);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        name[0] = 0;
    }
}

struct Desc {
    int internal = -1, flags = 0, activation = 0, shape = -1;
    const uint8_t* user = nullptr;
};

bool read_obj(const uint8_t* o, double* pos, Desc* d)
{
    __try {
        memcpy(pos, o + kObjOrigin, 3 * sizeof(double));
        if (d) {
            d->internal = *reinterpret_cast<const int*>(o + kObjInternalType);
            d->flags = *reinterpret_cast<const int*>(o + kObjFlags);
            d->activation = *reinterpret_cast<const int*>(o + kObjActivation);
            d->user = *reinterpret_cast<const uint8_t* const*>(o + kObjUser);
            const uint8_t* shape = *reinterpret_cast<const uint8_t* const*>(o + kObjShape);
            d->shape = shape ? *reinterpret_cast<const int*>(shape + kShapeType) : -1;
        }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void describe(const char* what, const uint8_t* o, const double* pos, const Track& t)
{
    double p[3];
    Desc d;
    if (!read_obj(o, p, &d)) return;
    char owner[128], self[128];
    rtti(d.user, owner, sizeof owner);
    rtti(o, self, sizeof self);
    const char* kind = d.internal == 2 ? "rigid body" : d.internal == 4 ? "ghost" : d.internal == 8 ? "soft body" : "other";
    GhostOwner go = d.internal == 4 ? ghost_owner(o) : kGhostNotCharacter;
    swlog("sanity: %s at tick %llu: %s %p (%s) flags 0x%x%s%s, activation %d, shape type %d, vehicle group %d%s, owner %p %s; "
          "now (%.6g %.6g %.6g), last sane (%.6g %.6g %.6g) %llu ticks ago",
          what, static_cast<unsigned long long>(g_tick), kind, o, self[0] ? self : "?", d.flags, (d.flags & 1) ? " static" : "",
          (d.flags & 2) ? " kinematic" : "", d.activation, d.shape, body_group(o),
          go == kGhostCharacterSeated ? ", CHARACTER ghost (seated)" : go == kGhostCharacterWalking ? ", CHARACTER ghost (walking)" : "",
          d.user, owner[0] ? owner : "", pos[0], pos[1], pos[2], t.good[0], t.good[1], t.good[2],
          static_cast<unsigned long long>(g_tick - t.good_tick));
}

void window_log(ULONGLONG now)
{
    if (!g_win_start) g_win_start = now;
    if (now - g_win_start < 5000) return;
    if (g_win_steps)
        swlog("sanity 5s: %llu ticks, %.0f objects per tick, %llu impossible positions per tick on average (%llu objects newly "
              "impossible), %llu jumps > %.0f m in one tick",
              static_cast<unsigned long long>(g_win_steps), double(g_win_objects) / double(g_win_steps), g_win_bad / g_win_steps,
              static_cast<unsigned long long>(g_win_new_bad), static_cast<unsigned long long>(g_win_jumps), kJumpM);
    g_win_start = now;
    g_win_objects = g_win_bad = g_win_new_bad = g_win_jumps = g_win_steps = 0;
    if (g_described > 30) g_described = 30; // room for a few new objects per window
    if (g_jumps_logged > 15) g_jumps_logged = 15;
}

} // namespace

// census.cpp, after every stepSimulation (physics thread).
void sanity_after_step(const void* world_ptr)
{
    if (!g_on || !world_ptr) return;
    auto world = static_cast<const uint8_t*>(world_ptr);
    int n = 0;
    const uint8_t* const* objs = nullptr;
    __try {
        n = *reinterpret_cast<const int*>(world + kWorldObjCount);
        objs = *reinterpret_cast<const uint8_t* const* const*>(world + kWorldObjData);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return;
    }
    if (n <= 0 || n > 1000000 || !objs) return;
    g_tick++;
    g_win_steps++;
    g_win_objects += n;
    if (g_tracks.size() > size_t(n) * 4 + 4096) g_tracks.clear(); // objects come and go: drop stale entries now and then
    for (int i = 0; i < n; i++) {
        const uint8_t* o = nullptr;
        __try {
            o = objs[i];
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            return;
        }
        double p[3];
        if (!o || !read_obj(o, p, nullptr)) continue;
        auto [it, fresh] = g_tracks.try_emplace(o);
        Track& t = it->second;
        bool bad = far_or_nan(p);
        if (bad) g_win_bad++;
        if (fresh) {
            memcpy(t.pos, p, sizeof p);
            if (!bad) {
                memcpy(t.good, p, sizeof p);
                t.good_tick = g_tick;
            }
            t.bad = bad;
            if (bad) {
                g_win_new_bad++;
                if (g_described++ < 40) describe("appeared at an impossible position", o, p, t);
            }
            continue;
        }
        if (bad && !t.bad) {
            g_win_new_bad++;
            if (g_described++ < 40) {
                describe("went to an impossible position", o, p, t);
                t.follow = kFollowTicks;
            }
        } else if (t.follow > 0) {
            t.follow--;
            swlog("sanity:   %p tick %llu: (%.6g %.6g %.6g)", o, static_cast<unsigned long long>(g_tick), p[0], p[1], p[2]);
        }
        if (!bad && !far_or_nan(t.pos)) {
            double dx = p[0] - t.pos[0], dy = p[1] - t.pos[1], dz = p[2] - t.pos[2];
            double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (d > kJumpM) {
                g_win_jumps++;
                if (g_jumps_logged++ < 20) describe("jumped", o, p, t);
            }
        }
        if (!bad) {
            memcpy(t.good, p, sizeof p);
            t.good_tick = g_tick;
        }
        t.bad = bad;
        memcpy(t.pos, p, sizeof p);
    }
    window_log(GetTickCount64());
}

namespace {

bool install()
{
    g_on = GetPrivateProfileIntW(L"sanity", L"enabled", 1, settings_ini()) != 0;
    if (!g_on) {
        swlog("sanity: off in settings.ini");
        return true;
    }
    g_base = game_base();
    auto dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(g_base);
    g_size = reinterpret_cast<const IMAGE_NT_HEADERS64*>(g_base + dos->e_lfanew)->OptionalHeader.SizeOfImage;
    swlog("sanity: on (every physics object checked every tick: impossible positions and > %.0f m jumps are described; "
          "\"sanity\" lines)", kJumpM);
    return true;
}

HookRegistrar reg{{"world sanity watch", /*needs_known_build=*/true, &install}};

} // namespace
