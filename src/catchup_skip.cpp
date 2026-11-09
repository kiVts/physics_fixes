
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "world.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <cstdint>

namespace {

constexpr uintptr_t kCatchUp = 0x1408686c0, kIsHost = 0x140ceb6ff;
constexpr uint32_t kQueueCount = 0xce300;

using U = uint64_t;
using CatchUp_t = void(__fastcall*)(U, U, U, U, U);
CatchUp_t g_orig = nullptr;

bool g_on = false;
int g_hi = 60, g_lo = 30, g_max_run = 2;
const volatile uint8_t* g_is_host = nullptr;
std::atomic<const uint8_t*> g_game{nullptr};
std::atomic<bool> g_in_catchup{false};

// physics thread (the update thread waits for it while the catch-up runs)
bool g_behind = false;
int g_run = 0;
uint64_t g_skipped = 0, g_stepped = 0, g_walking = 0, g_unknown = 0;
DWORD g_last_log = 0;

void __fastcall catchup_detour(U g, U a2, U a3, U a4, U a5)
{
    g_game.store(reinterpret_cast<const uint8_t*>(g), std::memory_order_relaxed);
    g_in_catchup.store(true, std::memory_order_release);
    g_orig(g, a2, a3, a4, a5);
    g_in_catchup.store(false, std::memory_order_release);
}

void maybe_log()
{
    DWORD now = GetTickCount();
    if (now - g_last_log < 10000) return;
    if (g_skipped || g_walking || g_unknown)
        swlog("catch-up physics 10s: %llu steps skipped, %llu run while behind (every %d), %llu not skipped because your "
              "character was walking, %llu because the character scan wasn't available",
              static_cast<unsigned long long>(g_skipped), static_cast<unsigned long long>(g_stepped), g_max_run + 1,
              static_cast<unsigned long long>(g_walking), static_cast<unsigned long long>(g_unknown));
    g_last_log = now;
    g_skipped = g_stepped = g_walking = g_unknown = 0;
}

bool decide(const void* world)
{
    if (!g_on || !g_is_host || *g_is_host || !g_in_catchup.load(std::memory_order_acquire)) {
        g_run = 0;
        return false;
    }
    const uint8_t* game = g_game.load(std::memory_order_relaxed);
    if (!game) return false;
    uint32_t queued = *reinterpret_cast<const volatile uint32_t*>(game + kQueueCount);
    if (queued > uint32_t(g_hi)) g_behind = true;
    else if (queued < uint32_t(g_lo)) g_behind = false;
    if (!g_behind) {
        g_run = 0;
        return false;
    }
    if (queued == 0 || g_run >= g_max_run) {
        g_stepped++;
        g_run = 0;
        maybe_log();
        return false;
    }
    puppet_step_begin(world); // fresh: is there a local character, and is it seated
    int local = puppet_local_state();
    if (local < 0 || local == 2) {
        (local < 0 ? g_unknown : g_walking)++;
        g_run = 0;
        maybe_log();
        return false;
    }
    g_run++;
    g_skipped++;
    maybe_log();
    return true;
}

bool install()
{
    const wchar_t* ini = settings_ini();
    g_on = GetPrivateProfileIntW(L"catchup", L"skip_physics", 1, ini) != 0;
    g_hi = std::clamp(int(GetPrivateProfileIntW(L"catchup", L"behind_ticks", 60, ini)), 15, 10000);
    g_lo = g_hi / 2;
    g_max_run = std::clamp(int(GetPrivateProfileIntW(L"catchup", L"max_skip_in_row", 2, ini)), 1, 9);
    if (!g_on) {
        swlog("catch-up physics: off in settings.ini");
        return true;
    }
    g_is_host = game_ptr<const volatile uint8_t*>(kIsHost);
    bool ok = hook_game(kCatchUp, reinterpret_cast<void*>(&catchup_detour), reinterpret_cast<void**>(&g_orig));
    swlog("catch-up physics: %s (joining player more than %d ticks behind: skips up to %d of every %d physics steps "
          "while your character is seated or not spawned yet)",
          ok ? "on" : "FAILED", g_hi, g_max_run, g_max_run + 1);
    if (!ok) g_on = false;
    return true;
}

HookRegistrar reg{{"catch-up physics skip", /*needs_known_build=*/true, &install}};

} // namespace

bool catchup_skip_step(const void* world)
{
    __try {
        return decide(world);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        g_on = false; // never again this session
        return false;
    }
}
