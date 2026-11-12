#pragma once
#include <cstdint>

// Owned by one simulate invocation on its calling thread. No pointer captured
// here survives that invocation, including nested simulation and fault exits.
struct EarlyLaunchContext {
    uintptr_t sim, frame, list, rate_source;
    bool launched = false;
    bool disabled = false;
    EarlyLaunchContext* parent = nullptr;
    uintptr_t task = 0;
    bool phase_held = false;
    bool pending = false;
    uint64_t paid_wait_ns = 0;
    bool submitted = false;
    bool logic_returned = false;
    bool owns_task(uintptr_t value) const { return pending && task == value; }

    template<class Release> void release_phase(Release release)
    {
        if (!phase_held) return;
        phase_held = false;
        release();
    }
    template<class Release, class Join> void finish(Release release, Join join)
    {
        // The physics callback may be waiting for the phase owner. Never join
        // before releasing it, and never forget ownership on a failed join.
        release_phase(release);
        if (pending) {
            join(task);
            pending = false;
        }
    }

    bool matches(uintptr_t simulation) const { return sim == simulation; }
    bool accept_logic(uintptr_t rate, uintptr_t current_frame)
    {
        if (rate != rate_source || current_frame != frame) disabled = true;
        return !disabled;
    }
    bool consume(uintptr_t simulation)
    {
        if (!matches(simulation) || !launched) return false;
        launched = false;
        return true;
    }
};
