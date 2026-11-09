
#include "hooks.h"
#include "log.h"
#include "world.h"

#include <windows.h>
#include <atomic>
#include <cmath>
#include <cstdint>

const char* const kDistRingNames[kDistRings] = {"<0.5 km", "0.5-1 km", "1-2 km", "2+ km"};

int distance_ring(const double pos[3], const double player[3])
{
    double dx = pos[0] - player[0], dy = pos[1] - player[1], dz = pos[2] - player[2];
    double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    return d < 500.0 ? 0 : d < 1000.0 ? 1 : d < 2000.0 ? 2 : 3;
}

namespace {

constexpr uintptr_t kInputSender = 0x140863ea0;
constexpr size_t kGameControlled = 0xcdab8, kControlledKind = 0, kControlledList = 0x90;
constexpr size_t kCharCapsule = 0x478, kSceneRigid = 0xb8, kObjOrigin = 0x70, kCharPos = 0x68;
constexpr ULONGLONG kStaleMs = 2000;

using Sender_t = void(__fastcall*)(void* game, int64_t a2, void* a3);
Sender_t g_orig = nullptr;

SRWLOCK g_lock = SRWLOCK_INIT;
double g_pos[3];
const void* g_char = nullptr;
ULONGLONG g_seen = 0;
std::atomic<const uint8_t*> g_tick_owner{nullptr};
bool g_logged = false;
std::atomic<const void*> g_game_state{nullptr};

int read_character(const uint8_t* game, double pos[3], double own[3], const uint8_t*& ch)
{
    __try {
        ch = nullptr;
        const uint8_t* c = *reinterpret_cast<const uint8_t* const*>(game + kGameControlled);
        if (!c || *reinterpret_cast<const int*>(c + kControlledKind) != 4) return 0;
        const uint8_t* const* list = *reinterpret_cast<const uint8_t* const* const*>(c + kControlledList);
        ch = list ? list[0] : nullptr;
        if (!ch) return 0;
        for (int i = 0; i < 3; i++) own[i] = reinterpret_cast<const double*>(ch + kCharPos)[i];
        const uint8_t* so = *reinterpret_cast<const uint8_t* const*>(ch + kCharCapsule);
        const uint8_t* bt = so ? *reinterpret_cast<const uint8_t* const*>(so + kSceneRigid) : nullptr;
        if (bt) {
            for (int i = 0; i < 3; i++) pos[i] = reinterpret_cast<const double*>(bt + kObjOrigin)[i];
            if (std::isfinite(pos[0]) && std::isfinite(pos[1]) && std::isfinite(pos[2])) return 1;
        }
        for (int i = 0; i < 3; i++) pos[i] = own[i];
        return std::isfinite(pos[0]) && std::isfinite(pos[1]) && std::isfinite(pos[2]) ? 2 : 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
}

void __fastcall sender_detour(void* game, int64_t a2, void* a3)
{
    if (a2) g_tick_owner.store(reinterpret_cast<const uint8_t*>(a2), std::memory_order_relaxed);
    g_orig(game, a2, a3);
    g_game_state.store(game, std::memory_order_release);
    double pos[3], own[3] = {};
    const uint8_t* ch = nullptr;
    int src = read_character(static_cast<const uint8_t*>(game), pos, own, ch);
    if (!src) return;
    AcquireSRWLockExclusive(&g_lock);
    for (int i = 0; i < 3; i++) g_pos[i] = pos[i];
    g_char = ch;
    g_seen = GetTickCount64();
    ReleaseSRWLockExclusive(&g_lock);
    if (!g_logged) {
        g_logged = true;
        swlog("local player: character at (%.0f, %.0f, %.0f) from its %s (character +0x68: (%.0f, %.0f, %.0f))", pos[0], pos[1], pos[2],
              src == 1 ? "capsule body" : "own position", own[0], own[1], own[2]);
    }
}

bool install()
{
    if (!hook_game(kInputSender, reinterpret_cast<void*>(&sender_detour), reinterpret_cast<void**>(&g_orig))) return false;
    swlog("local player: on (your character's position for the \"by distance\" lines)");
    return true;
}

HookRegistrar reg{{"local player", /*needs_known_build=*/true, &install}};

} // namespace

bool local_player_pos(double out[3], const void** character)
{
    AcquireSRWLockShared(&g_lock);
    bool ok = g_seen && GetTickCount64() - g_seen < kStaleMs;
    for (int i = 0; i < 3; i++) out[i] = g_pos[i];
    if (character) *character = ok ? g_char : nullptr;
    ReleaseSRWLockShared(&g_lock);
    return ok;
}

// network_fixes object_push.cpp (update thread, right after this hook in the same tick): this PC's own character, or null
// when none was seen in the last 2 s.
extern "C" __declspec(dllexport) const void* __cdecl swml_local_character()
{
    const void* ch = nullptr;
    double pos[3];
    return local_player_pos(pos, &ch) ? ch : nullptr;
}

bool host_tick(uint32_t& tick)
{
    const uint8_t* o = g_tick_owner.load(std::memory_order_relaxed);
    if (!o) return false;
    __try {
        tick = *reinterpret_cast<const volatile uint32_t*>(o + 0x33c);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

// object_push.cpp: the same live game state used for local input. Fresh only while this PC controls a character.
extern "C" __declspec(dllexport) const void* __cdecl swml_game_state()
{
    const void* ch = nullptr;
    double pos[3];
    return local_player_pos(pos, &ch) ? g_game_state.load(std::memory_order_acquire) : nullptr;
}
