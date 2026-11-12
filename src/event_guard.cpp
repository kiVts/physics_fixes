
#include "hooks.h"
#include "log.h"
#include "sdk.h"

#include <windows.h>
#include <atomic>
#include <cstdint>

namespace {

using Exec_t = void(__fastcall*)(void* ev, void* scene);
struct Kind {
    uintptr_t address;
    const char* name;
    size_t body;
    Exec_t orig;
    std::atomic<uint64_t> skipped;
};
Kind g_kinds[] = {
    {0x1401a0e80, "set_transform", 0xb8, nullptr, {0}},
    {0x1401a1100, "set_transform_kinematic", 0xb8, nullptr, {0}},
    {0x1401a1380, "set_transform_ghost", 0x148, nullptr, {0}},
};
constexpr size_t kEventObject = 0x08, kObjOwner = 0xc0, kEventPos = 0x70;
std::atomic<int> g_logged{0};
std::atomic<uint64_t> g_unreadable{0};
std::atomic<uint64_t> g_last_log{0};

bool missing(const Kind& k, void* ev)
{
    __try {
        auto obj = *reinterpret_cast<uint8_t* const*>(static_cast<uint8_t*>(ev) + kEventObject);
        if (obj && *reinterpret_cast<void* const*>(obj + k.body)) return false;
        if (g_logged.fetch_add(1) < 10) {
            const double* p = reinterpret_cast<const double*>(static_cast<uint8_t*>(ev) + kEventPos);
            void* owner = obj ? *reinterpret_cast<void* const*>(obj + kObjOwner) : nullptr;
            swlog("event guard: %s for scene object %p without a %s (owner %p, at %.1f %.1f %.1f): skipped", k.name, obj,
                  k.body == 0x148 ? "ghost" : "body", owner, k.body == 0x148 ? 0.0 : p[0], k.body == 0x148 ? 0.0 : p[1],
                  k.body == 0x148 ? 0.0 : p[2]);
        }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        if (g_unreadable.fetch_add(1) == 0) {
            swlog("event guard: unreadable %s target; original executor suppressed; object ownership fault", k.name);
            swml_report_fault(swml(), swml_self());
        }
        return true;
    }
}

void maybe_log()
{
    uint64_t now = GetTickCount64(), last = g_last_log.load(std::memory_order_relaxed);
    if (now - last < 30000 || !g_last_log.compare_exchange_strong(last, now)) return;
    swlog("event guard 30s: skipped %llu set_transform, %llu kinematic, %llu ghost (missing or unreadable target); %llu unreadable probes total",
          (unsigned long long)g_kinds[0].skipped.exchange(0), (unsigned long long)g_kinds[1].skipped.exchange(0),
          (unsigned long long)g_kinds[2].skipped.exchange(0), (unsigned long long)g_unreadable.load());
}

template <int I> void __fastcall exec(void* ev, void* scene)
{
    Kind& k = g_kinds[I];
    if (missing(k, ev)) {
        k.skipped.fetch_add(1, std::memory_order_relaxed);
        maybe_log();
        return;
    }
    k.orig(ev, scene);
}

bool install()
{
    if (!GetPrivateProfileIntW(L"parallel", L"event_guard", 1, settings_ini())) {
        swlog("event guard: off in settings.ini");
        return true;
    }
    void* detours[] = {reinterpret_cast<void*>(&exec<0>), reinterpret_cast<void*>(&exec<1>), reinterpret_cast<void*>(&exec<2>)};
    for (int i = 0; i < 3; i++)
        if (!hook_game(g_kinds[i].address, detours[i], reinterpret_cast<void**>(&g_kinds[i].orig))) return false;
    swlog("event guard: transform events skip scene objects without a Bullet object (as event_set_impulse does)");
    return true;
}

HookRegistrar reg{{"event guard", /*needs_known_build=*/true, &install}};

} // namespace
