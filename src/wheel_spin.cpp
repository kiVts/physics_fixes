
#include "game.h"
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace {

constexpr uintptr_t kWheelTick = 0x1403aa4a0;
constexpr size_t kDefinition = 0x58, kDefKind = 0xda0, kBrake = 0x408, kVariableBrake = 0x428, kSpin = 0x458, kTyre = 0x814;

using Tick_t = void(__fastcall*)(uintptr_t comp, uintptr_t rdx, uintptr_t r8, uintptr_t r9, uintptr_t phys, uintptr_t com, float s30,
                                 uintptr_t s38, uintptr_t s40, uintptr_t s48, uintptr_t s50, uintptr_t s58, uint32_t s60, uint32_t s68,
                                 uintptr_t s70, uintptr_t s78, uintptr_t s80, uintptr_t s88, uintptr_t s90, uintptr_t s98, uint8_t sa0,
                                 uint8_t sa8, uintptr_t sb0, uintptr_t sb8, uintptr_t sc0);
Tick_t g_orig = nullptr;
float g_keep = 0.9999f;

std::atomic<uint64_t> g_ticks{0}, g_fixed{0}, g_braking{0}, g_last_log{0};

template <typename T> T rd(uintptr_t a) { T v; std::memcpy(&v, reinterpret_cast<const void*>(a), sizeof v); return v; }

void __fastcall wheel_tick(uintptr_t comp, uintptr_t rdx, uintptr_t r8, uintptr_t r9, uintptr_t phys, uintptr_t com, float s30,
                           uintptr_t s38, uintptr_t s40, uintptr_t s48, uintptr_t s50, uintptr_t s58, uint32_t s60, uint32_t s68,
                           uintptr_t s70, uintptr_t s78, uintptr_t s80, uintptr_t s88, uintptr_t s90, uintptr_t s98, uint8_t sa0,
                           uint8_t sa8, uintptr_t sb0, uintptr_t sb8, uintptr_t sc0)
{
    g_orig(comp, rdx, r8, r9, phys, com, s30, s38, s40, s48, s50, s58, s60, s68, s70, s78, s80, s88, s90, s98, sa0, sa8, sb0, sb8, sc0);
    g_ticks.fetch_add(1, std::memory_order_relaxed);
    uintptr_t def = comp ? rd<uintptr_t>(comp + kDefinition) : 0;
    if (def) {
        int kind = rd<int>(def + kDefKind), tyre = rd<int>(comp + kTyre);
        float decay = tyre == 0 ? 0.99f : tyre == 1 ? 0.995f : 1.0f;
        if (!(unsigned(kind - 1) < 4 || kind == 8) && decay < 1.0f) {
            float spin = rd<float>(comp + kSpin);
            if (std::isfinite(spin)) {
                spin = spin / decay * g_keep;
                std::memcpy(reinterpret_cast<void*>(comp + kSpin), &spin, sizeof spin);
                g_fixed.fetch_add(1, std::memory_order_relaxed);
                if (rd<uint8_t>(comp + kBrake) || rd<float>(comp + kVariableBrake) > 0.0f) g_braking.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }
    uint64_t now = GetTickCount64(), last = g_last_log.load(std::memory_order_relaxed);
    if (now - last >= 5000 && g_last_log.compare_exchange_strong(last, now, std::memory_order_relaxed) && last) {
        uint64_t t = g_ticks.exchange(0), f = g_fixed.exchange(0), b = g_braking.exchange(0);
        swlog("wheels 5s: %.0f advanced-wheel ticks/s, %.0f%% with the spin loss replaced (%.0f%% of those braking)", t / 5.0,
              t ? 100.0 * f / t : 0.0, f ? 100.0 * b / f : 0.0);
    }
}

bool install()
{
    const wchar_t* ini = settings_ini();
    if (!GetPrivateProfileIntW(L"wheels", L"spin_fix", 1, ini)) {
        swlog("wheels: spin fix off in settings.ini");
        return true;
    }
    wchar_t buf[32];
    GetPrivateProfileStringW(L"wheels", L"keep_per_tick", L"0.9999", buf, 32, ini);
    double keep = wcstod(buf, nullptr);
    g_keep = std::isfinite(keep) ? float(std::clamp(keep, 0.9, 1.0)) : 0.9999f;
    if (!hook_game(kWheelTick, reinterpret_cast<void*>(&wheel_tick), reinterpret_cast<void**>(&g_orig))) {
        swlog("wheels: hooking the advanced wheel tick failed; off");
        return false;
    }
    swlog("wheels: advanced wheels keep %.4f of their spin per tick (game 0.99 / 0.995): speed-proportional rolling drag cut to "
          "%.0f%% of the game's; braking unchanged (\"wheels 5s\" lines)", g_keep, 100.0 * (1.0 - g_keep) / 0.01);
    return true;
}

HookRegistrar reg{{"wheel_spin", /*needs_known_build=*/true, &install}};

} // namespace
