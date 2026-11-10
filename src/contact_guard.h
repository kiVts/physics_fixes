#pragma once
// The contact callback's two guards (parallel_guard.cpp), kept apart from the hooks so they can be tested with threads.
//
// Rings lock: the callback and the logic's contact ring drain exclude each other. It is reentrant per thread, because an
// SRW lock taken twice by one thread never returns.
//
// Despawn wait: after a despawn zeroes an owner pointer, wait until no callback that may have loaded it is still running.
// Not on a thread inside a callback, though: the callback itself can destroy objects (a contact that breaks or despawns
// something), and waiting there waited for itself, forever. No other thread's callback can be running then, because each
// one holds the rings lock, which this thread has.
#include <windows.h>
#include <atomic>
#include <cstdint>

struct ContactGuard {
    SRWLOCK rings = SRWLOCK_INIT;
    std::atomic<int> in_callback{0};
    std::atomic<uint64_t> self_destroys{0};
    // Per thread. There is one guard in the game; tests make several and use each from fresh threads.
    static inline thread_local int t_rings = 0, t_in_callback = 0;

    // True when this call took the lock (and may have waited for it); false when this thread already held it.
    bool lock()
    {
        if (t_rings++) return false;
        AcquireSRWLockExclusive(&rings);
        return true;
    }
    void unlock()
    {
        if (--t_rings == 0) ReleaseSRWLockExclusive(&rings);
    }
    void callback_begin()
    {
        in_callback.fetch_add(1, std::memory_order_seq_cst);
        t_in_callback++;
    }
    void callback_end()
    {
        t_in_callback--;
        in_callback.fetch_sub(1, std::memory_order_release);
    }
    bool self_callback() const { return t_in_callback != 0; }

    // Waits while another thread's callback runs. Returns the spins it took (0: none). `stuck(spins)` is called every
    // 65536 spins so a caller can report one that never ends.
    template <class Stuck> uint64_t wait_callbacks(Stuck&& stuck)
    {
        if (t_in_callback) {
            self_destroys.fetch_add(1, std::memory_order_relaxed);
            return 0;
        }
        std::atomic_thread_fence(std::memory_order_seq_cst); // the zeroing store before the check
        uint64_t spins = 0;
        while (in_callback.load(std::memory_order_acquire)) {
            YieldProcessor();
            if (!(++spins & 0xffff)) stuck(spins);
        }
        return spins;
    }
};
