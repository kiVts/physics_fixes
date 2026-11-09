
#include "parallel.h"

#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "game.h"

#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <cstdint>

namespace {

constexpr uintptr_t kGate = 0x140748860, kPreStep = 0x1405ec4b0, kPostLast = 0x1405ec6b0, kPost1 = 0x140746c60,
                    kLogic = 0x1405e8fb0, kQueueClear = 0x1408990b0, kSubmit = 0x14017d720;
constexpr uintptr_t kPostLastRet13 = 0x140740ad7, kGateRet15 = 0x140743701; // return addresses in simulate
constexpr size_t kSceneQueue = 0x30, kSceneInFlight = 0x3b8, kSceneRate = 0x2a8, kSceneTaskScene = 0x2b0,
                 kSceneTaskFrame = 0x2b8, kSceneTaskOne = 0x2c0, kSceneTaskSlot = 0x2c8, kSceneTask = 0x1f0;
constexpr size_t kGateRetToSceneAdd = 0x1a; // offset of "48 81 C3 imm32" after the gate's return address
size_t g_game_scene = 0;
constexpr size_t kSimRateOverride = 0x90, kSimSamples = 0xc88, kSimSampleCap = 0xc90, kSimSampleHead = 0xc94,
                 kSimSampleCount = 0xc98, kSimRate = 0xca0, kSimRateCountdown = 0xca4, kR13PreStepArg = 0x33c;

struct Queue { uintptr_t* data; uint32_t cap, head, count; };

using Gate_t = bool(__fastcall*)(void* sim);
using PreStep_t = void(__fastcall*)(void* game, uint32_t tick);
using Four_t = uint64_t(__fastcall*)(void*, void*, void*, void*);
using Logic_t = uint64_t(__fastcall*)(void*, void*, void*, void*, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t);
using Clear_t = void(__fastcall*)(void* queue);
using Submit_t = void(__fastcall*)(void* slot, void* task);
using Exec_t = void(__fastcall*)(void* event, void* scene);

Gate_t g_gate = nullptr;
Four_t g_post_last = nullptr, g_post1 = nullptr;
Logic_t g_logic = nullptr;
PreStep_t g_pre_step = nullptr;
Clear_t g_clear = nullptr;
Submit_t g_submit = nullptr;
uintptr_t g_ret13 = 0, g_ret15 = 0;

int g_mode = 0;
DWORD g_update_tid = 0;
uintptr_t g_r13 = 0, g_sim_rate_src = 0, g_frame = 0; // local_8a0, local_8e0 (previous tick's logic call)
bool g_launched = false; // this tick's launch already happened early
uint64_t g_early = 0, g_late = 0, g_skipped = 0, g_stale = 0;
ULONGLONG g_last_log = 0;
bool g_off = false; // switched off at run time (parameters changed)

template <class T> T& at(uintptr_t a, size_t off) { return *reinterpret_cast<T*>(a + off); }

float pick_rate(uintptr_t sim)
{
    float over = at<float>(g_sim_rate_src, kSimRateOverride);
    if (over != 0.0f || over != over) return over; // the game: non-zero (or NaN) override wins
    if (--at<int32_t>(sim, kSimRateCountdown) == 0) {
        uint32_t n = at<uint32_t>(sim, kSimSampleCount), cap = at<uint32_t>(sim, kSimSampleCap),
                 head = at<uint32_t>(sim, kSimSampleHead);
        const uint64_t* s = at<uint64_t*>(sim, kSimSamples);
        double sum = 0.0;
        for (uint32_t i = 0; i < n; i++) sum += static_cast<double>(s[(head + i) % cap]);
        double avg = sum / static_cast<double>(n) / 1000000.0;
        at<int32_t>(sim, kSimRateCountdown) = 900;
        float& rate = at<float>(sim, kSimRate);
        if (rate == 180.0f) {
            if (!(avg < 1.0) && !(avg == 1.0)) rate = 120.0f;
        } else if (rate == 120.0f) {
            if (!(avg <= 1.0)) rate = 60.0f; // NaN included, like the game
            else if (avg < 0.5) rate = 180.0f;
        } else if (rate == 60.0f) {
            if (!(0.5 < avg) && !(avg == 0.5)) rate = 120.0f;
        }
    }
    return at<float>(sim, kSimRate);
}

void launch_early(uintptr_t game, uintptr_t sim)
{
    if (!g_gate(reinterpret_cast<void*>(sim))) return; // the 0x15 gate will say the same: nothing to step
    uintptr_t scene = game + g_game_scene;
    if (at<uint8_t>(scene, kSceneInFlight)) { // can't happen after zone 0x10's wait; leave it to the old path
        g_skipped++;
        return;
    }
    g_pre_step(reinterpret_cast<void*>(game), at<uint32_t>(g_r13, kR13PreStepArg));
    Queue& q = at<Queue>(scene, kSceneQueue);
    for (uint32_t i = 0; i < q.count; i++) {
        uintptr_t ev = q.data[(q.head + i) % q.cap];
        reinterpret_cast<Exec_t>(at<uintptr_t*>(ev, 0)[1])(reinterpret_cast<void*>(ev), reinterpret_cast<void*>(scene));
    }
    g_clear(&q);
    float rate = pick_rate(sim);
    at<uint8_t>(scene, kSceneInFlight) = 1;
    at<float>(scene, kSceneRate) = rate;
    at<uintptr_t>(scene, kSceneTaskScene) = scene;
    at<uintptr_t>(scene, kSceneTaskFrame) = g_frame;
    at<int32_t>(scene, kSceneTaskOne) = 1;
    g_submit(reinterpret_cast<void*>(scene + kSceneTaskSlot), reinterpret_cast<void*>(scene + kSceneTask));
    g_launched = true;
    g_early++;
}

uint64_t __fastcall post1_detour(void* sim, void* r13, void* a3, void* a4)
{
    if (GetCurrentThreadId() == g_update_tid) g_r13 = reinterpret_cast<uintptr_t>(r13);
    return g_post1(sim, r13, a3, a4);
}

uint64_t __fastcall post_last_detour(void* game, void* a2, void* a3, void* sim_plus_8)
{
    uint64_t r = g_post_last(game, a2, a3, sim_plus_8);
    // a new tick: a flag left by a simulate that never reached the 0x15 gate (client catch-up, world leaving) must not
    // skip this tick's launch
    if (reinterpret_cast<uintptr_t>(_ReturnAddress()) == g_ret13 && GetCurrentThreadId() == g_update_tid) {
        if (g_launched) g_stale++;
        g_launched = false;
    }
    if (g_mode == 1 && !g_off && parallel_on() && reinterpret_cast<uintptr_t>(_ReturnAddress()) == g_ret13 &&
        GetCurrentThreadId() == g_update_tid && g_sim_rate_src && g_frame && g_r13)
        launch_early(reinterpret_cast<uintptr_t>(game), reinterpret_cast<uintptr_t>(sim_plus_8) - 8);
    return r;
}

bool __fastcall gate_detour(void* sim)
{
    if (reinterpret_cast<uintptr_t>(_ReturnAddress()) == g_ret15) {
        g_update_tid = GetCurrentThreadId(); // the thread that launches physics
        if (g_launched) {
            g_launched = false;
            return false; // launched at the end of zone 0x13 already: skip the old launch block
        }
        if (g_mode == 1 && !g_off) g_late++;
    }
    return g_gate(sim);
}

uint64_t __fastcall logic_detour(void* game, void* sim_rate_src, void* frame, void* a4, uint64_t a5, uint64_t a6,
                                 uint64_t a7, uint64_t a8, uint64_t a9, uint64_t a10)
{
    uintptr_t s = reinterpret_cast<uintptr_t>(sim_rate_src), f = reinterpret_cast<uintptr_t>(frame);
    if ((g_sim_rate_src && s != g_sim_rate_src) || (g_frame && f != g_frame)) {
        if (g_launched) swlog("early launch: simulate's parameters changed after this tick's early launch");
        if (!g_off) swlog("early launch: simulate's parameters changed (%p %p -> %p %p); following them", (void*)g_sim_rate_src,
                          (void*)g_frame, sim_rate_src, frame);
    }
    g_sim_rate_src = s, g_frame = f;
    uint64_t r = g_logic(game, sim_rate_src, frame, a4, a5, a6, a7, a8, a9, a10);
    ULONGLONG now = GetTickCount64();
    if (!g_last_log) g_last_log = now;
    if (g_mode == 1 && now - g_last_log >= 30000) {
        g_last_log = now;
        swlog("early launch 30s: %llu early, %llu the old way, %llu left to the old way (step still in flight), %llu stale flags",
              (unsigned long long)g_early, (unsigned long long)g_late, (unsigned long long)g_skipped, (unsigned long long)g_stale);
    }
    return r;
}

bool install()
{
    g_mode = static_cast<int>(GetPrivateProfileIntW(L"parallel", L"early_launch", 0, settings_ini()));
    if (g_mode <= 0) return true; // lockstep: nothing hooked
    g_pre_step = game_ptr<PreStep_t>(kPreStep);
    g_clear = game_ptr<Clear_t>(kQueueClear);
    g_submit = game_ptr<Submit_t>(kSubmit);
    g_ret13 = game_live(kPostLastRet13);
    g_ret15 = game_live(kGateRet15);
    if (!g_pre_step || !g_clear || !g_submit || !g_ret13 || !g_ret15) return false;
    const uint8_t* add = reinterpret_cast<const uint8_t*>(g_ret15 + kGateRetToSceneAdd);
    if (add[0] != 0x48 || add[1] != 0x81 || add[2] != 0xc3) {
        swlog("early launch: the launch block doesn't look as expected (no add rbx, imm32 after the gate); off");
        return false;
    }
    g_game_scene = *reinterpret_cast<const uint32_t*>(add + 3);
    if (!hook_game(kGate, reinterpret_cast<void*>(&gate_detour), reinterpret_cast<void**>(&g_gate)) ||
        !hook_game(kPost1, reinterpret_cast<void*>(&post1_detour), reinterpret_cast<void**>(&g_post1)) ||
        !hook_game(kLogic, reinterpret_cast<void*>(&logic_detour), reinterpret_cast<void**>(&g_logic)) ||
        !hook_game(kPostLast, reinterpret_cast<void*>(&post_last_detour), reinterpret_cast<void**>(&g_post_last)))
        return false;
    swlog("early launch: ON: physics starts at the end of zone 0x13, logic runs alongside it (scene at game+%#zx; "
          "\"early launch 30s\")", g_game_scene);
    return true;
}

HookRegistrar reg{{"early launch", /*needs_known_build=*/true, &install}};

} // namespace
