
#include "parallel.h"
#include "early_launch_context.h"
#include "early_launch_contract.h"
#include "early_launch_timing.h"
#include "early_launch_transition.h"
#include "native_finally.h"
#include "stability_policy.h"

#include "hooks.h"
#include "log.h"
#include "sdk.h"
#include "game.h"

#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <cstdint>

namespace {

constexpr uintptr_t kSimulate = 0x14073f890;
constexpr uintptr_t kJoin = 0x14017d2c0;
constexpr uintptr_t kSubmitRet15 = 0x1407439e4;
constexpr uintptr_t kGate = 0x140748860, kPreStep = 0x1405ec4b0, kPostLast = 0x1405ec6b0,
                    kLogic = 0x1405e8fb0, kQueueClear = 0x1408990b0, kSubmit = 0x14017d720;
constexpr uintptr_t kPostLastRet13 = 0x140740ad7, kGateRet15 = 0x140743701; // return addresses in simulate
constexpr size_t kSceneQueue = 0x30, kSceneInFlight = 0x3b8, kSceneRate = 0x2a8, kSceneTaskScene = 0x2b0,
                 kSceneTaskFrame = 0x2b8, kSceneTaskOne = 0x2c0, kSceneTaskSlot = 0x2c8, kSceneTask = 0x1f0;
size_t g_game_scene = 0;
constexpr size_t kSimAsync=0xa68;
constexpr size_t kSimRateOverride = 0x90, kSimSamples = 0xc88, kSimSampleCap = 0xc90, kSimSampleHead = 0xc94,
                 kSimSampleCount = 0xc98, kSimRate = 0xca0, kSimRateCountdown = 0xca4, kR13PreStepArg = 0x33c;

struct Queue { uintptr_t* data; uint32_t cap, head, count; };

using Simulate_t = void(__fastcall*)(uintptr_t, uintptr_t, uintptr_t, uintptr_t, uintptr_t, uintptr_t, uintptr_t, uintptr_t, uintptr_t, uintptr_t, uintptr_t, uintptr_t);
Simulate_t g_simulate = nullptr;
using Gate_t = bool(__fastcall*)(void* sim);
using PreStep_t = void(__fastcall*)(void* game, uint32_t tick);
using Four_t = uint64_t(__fastcall*)(void*, void*, void*, void*);
using Logic_t = uint64_t(__fastcall*)(void*, void*, void*, void*, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t, uint64_t);
using Clear_t = void(__fastcall*)(void* queue);
using Submit_t = void(__fastcall*)(void* slot, void* task);
using Exec_t = void(__fastcall*)(void* event, void* scene);

Gate_t g_gate = nullptr;
Four_t g_post_last = nullptr;
Four_t g_join = nullptr;
Logic_t g_logic = nullptr;
PreStep_t g_pre_step = nullptr;
Clear_t g_clear = nullptr;
Submit_t g_submit = nullptr;
uintptr_t g_ret13 = 0, g_ret15 = 0;

int g_mode = 0;
thread_local EarlyLaunchContext* t_context = nullptr;
thread_local EarlyLaunchTiming t_timing;
thread_local EarlyLaunchTransition t_transition;
std::atomic<bool> g_faulted{false}; // a native exit/mismatch blocks future early submissions for this session
uint64_t g_early = 0, g_late = 0, g_skipped = 0, g_stale = 0;
ULONGLONG g_last_log = 0;

template <class T> T& at(uintptr_t a, size_t off) { return *reinterpret_cast<T*>(a + off); }

uint64_t monotonic_ns()
{
    static const double scale=[] {LARGE_INTEGER f{};QueryPerformanceFrequency(&f);return f.QuadPart>0?1e9/double(f.QuadPart):0.0;}();
    LARGE_INTEGER now{};QueryPerformanceCounter(&now);
    return static_cast<uint64_t>(double(now.QuadPart)*scale);
}

void preserve_native_post(EarlyLaunchContext& context)
{
    if(!context.submitted || at<uint8_t>(context.sim,kSimAsync))return;
    // Do not let a mid-tick mode toggle strand an already completed task's
    // native zone-10/11/12/13 processing. Delay only this toggle by one boundary.
    if(at<uint8_t>(at<uintptr_t>(context.sim,0)+g_game_scene,kSceneInFlight)) {
        t_transition.defer(context.sim,context.frame);
        at<uint8_t>(context.sim,kSimAsync)=1;
        context.disabled=true;
    }
}

void finish(EarlyLaunchContext& context)
{
    context.finish([] { parallel_phase_end(); }, [&](uintptr_t task) {
        const auto start=monotonic_ns();
        g_join(reinterpret_cast<void*>(task), nullptr, nullptr, nullptr);
        const auto end=monotonic_ns();
        if(end>=start)context.paid_wait_ns=EarlyLaunchTiming::add(context.paid_wait_ns,end-start);
    });
}

bool apply_paid_wait(EarlyLaunchContext& context)
{
    const auto paid=t_timing.take(context.sim,context.frame);
    if(!paid)return true;
    const auto count=at<uint32_t>(context.sim,kSimSampleCount);
    const auto cap=at<uint32_t>(context.sim,kSimSampleCap);
    const auto head=at<uint32_t>(context.sim,kSimSampleHead);
    auto* samples=at<uint64_t*>(context.sim,kSimSamples);
    // This runs after native zone-10 insertion and its trim to 120 samples.
    return EarlyLaunchTiming::apply(samples,cap,head,count,paid);
}

bool from_site(uintptr_t direct,uintptr_t expected)
{
    if(direct==expected)return true;
    // A previous SDK detour can replace our direct return address. Look for
    // the native frame in its unwind chain, rather than treating a mod frame
    // as the game's call site.
    void* frames[62];const auto n=RtlCaptureStackBackTrace(0,62,frames,nullptr);
    for(USHORT i=0;i<n;++i)if(reinterpret_cast<uintptr_t>(frames[i])==expected)return true;
    return false;
}

uint64_t __fastcall join_detour(void* task, void* a2, void* a3, void* a4)
{
    // Native joins inside simulate must also release the exclusion before wait.
    // Only the submitting thread has a matching invocation context.
    for (auto* c = t_context; c; c = c->parent)
        if (c->owns_task(reinterpret_cast<uintptr_t>(task)))
            c->release_phase([] { parallel_phase_end(); });
    const auto start=monotonic_ns();
    uint64_t result = g_join(task, a2, a3, a4);
    const auto end=monotonic_ns();
    for (auto* c = t_context; c; c = c->parent)
        if (c->owns_task(reinterpret_cast<uintptr_t>(task))) {
            if(end>=start)c->paid_wait_ns=EarlyLaunchTiming::add(c->paid_wait_ns,end-start);
            c->pending = false;
        }
    return result;
}

float pick_rate(uintptr_t sim, uintptr_t rate_source)
{
    float over = at<float>(rate_source, kSimRateOverride);
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

void launch_early(uintptr_t game, uintptr_t sim, EarlyLaunchContext& context)
{
    if(context.disabled || g_faulted.load(std::memory_order_acquire))return;
    if(!at<uint8_t>(sim,kSimAsync))return;
    if (!g_gate(reinterpret_cast<void*>(sim))) return; // the 0x15 gate will say the same: nothing to step
    uintptr_t scene = game + g_game_scene;
    if (at<uint8_t>(scene, kSceneInFlight)) { // can't happen after zone 0x10's wait; leave it to the old path
        g_skipped++;
        return;
    }
    if (!parallel_phase_begin()) return;
    context.phase_held = true;
    g_pre_step(reinterpret_cast<void*>(game), at<uint32_t>(context.list, kR13PreStepArg));
    Queue& q = at<Queue>(scene, kSceneQueue);
    for (uint32_t i = 0; i < q.count; i++) {
        uintptr_t ev = q.data[(q.head + i) % q.cap];
        reinterpret_cast<Exec_t>(at<uintptr_t*>(ev, 0)[1])(reinterpret_cast<void*>(ev), reinterpret_cast<void*>(scene));
    }
    g_clear(&q);
    float rate = pick_rate(sim, context.rate_source);
    at<uint8_t>(scene, kSceneInFlight) = 1;
    at<float>(scene, kSceneRate) = rate;
    at<uintptr_t>(scene, kSceneTaskScene) = scene;
    at<uintptr_t>(scene, kSceneTaskFrame) = context.frame;
    at<int32_t>(scene, kSceneTaskOne) = 1;
    g_submit(reinterpret_cast<void*>(scene + kSceneTaskSlot), reinterpret_cast<void*>(scene + kSceneTask));
    context.task = scene + kSceneTask;
    context.pending = true;
    context.submitted = true;
    context.launched = true;
    g_early++;
}

void __fastcall simulate_detour(uintptr_t sim, uintptr_t frame, uintptr_t list, uintptr_t a4,
    uintptr_t a5, uintptr_t a6, uintptr_t a7, uintptr_t a8, uintptr_t rate_source,
    uintptr_t a10, uintptr_t a11, uintptr_t a12)
{
    EarlyLaunchContext context{sim, frame, list, rate_source};
    t_transition.discard_other(sim,frame);
    if(t_transition.matches(sim,frame)) {
        const bool native_post_pending=at<uint8_t>(at<uintptr_t>(sim,0)+g_game_scene,kSceneInFlight)!=0;
        at<uint8_t>(sim,kSimAsync)=native_post_pending?1:0;
        if(!native_post_pending)t_transition.complete(sim,frame);
        context.disabled=true;
    }
    auto* previous = t_context;
    // A nested simulation cannot enter a native join while its enclosing
    // invocation still owns the contact exclusion.
    if (previous) {
        finish(*previous);
        context.disabled=true;g_faulted.store(true,std::memory_order_release);
        t_timing={}; // nested invocations cannot exchange one sample ledger
    }
    context.parent = previous;
    bool completed = false;
    t_context = &context;
    native_finally([&] {
        g_simulate(sim, frame, list, a4, a5, a6, a7, a8, rate_source, a10, a11, a12);
        completed = true;
    }, [&] {
        if (!completed) g_faulted.store(true, std::memory_order_release);
        native_finally([&] {
            finish(context);
            preserve_native_post(context);
            if(context.submitted)t_timing.publish(context.sim,context.frame,context.paid_wait_ns);
        }, [&] { t_context = previous; });
    });
}

uint64_t __fastcall post_last_detour(void* game, void* a2, void* a3, void* sim_plus_8)
{
    uint64_t r = g_post_last(game, a2, a3, sim_plus_8);
    auto* context = t_context;
    const auto sim = reinterpret_cast<uintptr_t>(sim_plus_8) - 8;
    if (context && context->matches(sim) && from_site(reinterpret_cast<uintptr_t>(_ReturnAddress()),g_ret13)) {
        if(t_transition.complete(context->sim,context->frame)) {
            at<uint8_t>(context->sim,kSimAsync)=0;
            context->disabled=true;
        }
        if(!apply_paid_wait(*context)) {
            context->disabled=true;g_faulted.store(true,std::memory_order_release);
            swlog("early launch: native wait-sample ring invalid; following native order");
            swml_report_fault(swml(),swml_self());
        }
        if (context->launched) {
            context->disabled = true;
            g_stale++;
        }
        if (g_mode == 1 && !g_faulted.load(std::memory_order_acquire) && !context->disabled && parallel_on() && context->rate_source && context->frame && context->list)
            launch_early(reinterpret_cast<uintptr_t>(game), sim, *context);
    }
    return r;
}

bool __fastcall gate_detour(void* sim)
{
    if (t_context && t_context->matches(reinterpret_cast<uintptr_t>(sim)) &&
        from_site(reinterpret_cast<uintptr_t>(_ReturnAddress()),g_ret15)) {
        // Even a disabled context must suppress a duplicate submission already made.
        const bool submitted = t_context->consume(reinterpret_cast<uintptr_t>(sim));
        finish(*t_context);
        preserve_native_post(*t_context);
        if (submitted) return false;
        if (g_mode == 1 && !t_context->disabled) g_late++;
    }
    if(t_context && t_context->matches(reinterpret_cast<uintptr_t>(sim)) &&
       t_context->launched && t_context->logic_returned) {
        // If another hook hides the native frame, do not authorize a second
        // submit of the same scene task. Preserve suppression until a known
        // native gate or invocation exit; disable future early submissions.
        finish(*t_context);preserve_native_post(*t_context);
        t_context->disabled=true;g_faulted.store(true,std::memory_order_release);
        swlog("early launch: post-logic gate caller could not be verified; duplicate submission suppressed, session disabled");
        swml_report_fault(swml(),swml_self());
        return false;
    }
    return g_gate(sim);
}

uint64_t __fastcall logic_detour(void* game, void* sim_rate_src, void* frame, void* a4, uint64_t a5, uint64_t a6,
                                 uint64_t a7, uint64_t a8, uint64_t a9, uint64_t a10)
{
    // Conservative boundary: all vehicle workers/sensors see one completed
    // physics state. No worker can depend on an unfinished task excluded by
    // its parent thread's contact lock. Retain overlap with zone 14 only.
    if(t_context)finish(*t_context);
    if (t_context && !t_context->accept_logic(reinterpret_cast<uintptr_t>(sim_rate_src), reinterpret_cast<uintptr_t>(frame))) {
        g_faulted.store(true, std::memory_order_release);
        swlog("early launch: current simulation argument contract mismatch; context disabled%s",
              t_context->launched ? " (physics already submitted; re-enable remains blocked)" : "");
        swml_report_fault(swml(), swml_self());
        finish(*t_context);
    }
    uint64_t r = g_logic(game, sim_rate_src, frame, a4, a5, a6, a7, a8, a9, a10);
    if(t_context)t_context->logic_returned=true;
    ULONGLONG now = GetTickCount64();
    if (!g_last_log) g_last_log = now;
    if (g_mode == 1 && now - g_last_log >= 30000) {
        g_last_log = now;
        swlog("early launch 30s: %llu early, %llu native, %llu in-flight skips, %llu duplicate context attempts",
              (unsigned long long)g_early, (unsigned long long)g_late, (unsigned long long)g_skipped, (unsigned long long)g_stale);
    }
    return r;
}

bool install()
{
    if (!kEarlyLaunchAudited) {
        swlog("early launch: disabled by stability audit; native physics/logic order retained (including existing early_launch=1 settings)");
        return true;
    }
    g_mode = static_cast<int>(GetPrivateProfileIntW(L"parallel", L"early_launch", 1, settings_ini()));
    if (g_mode <= 0) return true; // lockstep: nothing hooked
    g_pre_step = game_ptr<PreStep_t>(kPreStep);
    g_clear = game_ptr<Clear_t>(kQueueClear);
    g_submit = game_ptr<Submit_t>(kSubmit);
    g_ret13 = game_live(kPostLastRet13);
    g_ret15 = game_live(kGateRet15);
    if (!g_pre_step || !g_clear || !g_submit || !g_ret13 || !g_ret15) return false;
    const auto submit_end=game_live(kSubmitRet15);
    early_contract::Layout layout;
    if (submit_end<=g_ret15 || !early_contract::decode(reinterpret_cast<const uint8_t*>(g_ret15),submit_end-g_ret15,
            g_ret15,game_live(kSubmit),layout)) {
        swlog("early launch: launch/task/queue/rate field contract failed; native order retained");
        return false;
    }
    if(!early_contract::async_mode(reinterpret_cast<const uint8_t*>(submit_end),8))return false;
    g_game_scene = layout.scene;
    if (!hook_game(kJoin, reinterpret_cast<void*>(&join_detour), reinterpret_cast<void**>(&g_join)) ||
        !hook_game(kGate, reinterpret_cast<void*>(&gate_detour), reinterpret_cast<void**>(&g_gate)) ||
        !hook_game(kSimulate, reinterpret_cast<void*>(&simulate_detour), reinterpret_cast<void**>(&g_simulate)) ||
        !hook_game(kLogic, reinterpret_cast<void*>(&logic_detour), reinterpret_cast<void**>(&g_logic)) ||
        !hook_game(kPostLast, reinterpret_cast<void*>(&post_last_detour), reinterpret_cast<void**>(&g_post_last)))
        return false;
    swlog("early launch: ON: physics starts at the end of zone 0x13, logic runs alongside it (scene at game+%#zx; "
          "\"early launch 30s\")", g_game_scene);
    return true;
}

HookRegistrar reg{{"early launch", /*needs_known_build=*/true, &install}};

} // namespace
