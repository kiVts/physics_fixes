#pragma once
// The world lock between the physics step's writers and the logic's readers (parallel_guard.cpp).
//
// The physics thread takes it in ~450k short pieces per second (one per setAabb, ~0.1 us each); readers (rays, sphere
// queries) hold it ~10 us. With an SRW lock a reader that found it held waited ~45 us on average and up to 2.5 ms
// (2026-10-30 00:49: 56-63 ms/s blocked in total): woken after one release, it lost the lock again to the physics
// thread's next acquire, and again, until the burst of setAabb calls ended.
//
// Here the writer gives way: before each acquire, if a reader is waiting, it lets the waiting readers in first (they
// enter while the writer flag is clear) and only then raises the flag and waits for the readers inside to leave. A
// reader waits for at most the hold in progress plus the readers already inside; the writer waits for those readers
// instead (the physics step has slack while the update thread is the critical path). Readers and the writer spin
// (pause, then SwitchToThread): every wait is short, and a sleeping reader costs ~10-50 us to wake.
//
// Dekker on seq_cst atomics: the writer stores its flag then reads the reader count; a reader increments the count
// then reads the flag. At least one of them sees the other, so they never both proceed.
// Readers may nest on one thread (depth counted). Writers nest through their own counter in parallel_guard.cpp.
#include <windows.h>
#include <intrin.h>
#include <atomic>
#include <cstdint>

class WorldLock {
public:
    // Bounded courtesy: a waiting reader that doesn't get going (descheduled between announcing itself and entering)
    // can't hold the writer up for longer than this many pauses.
    static constexpr unsigned kCourtesySpins = 4096;

    struct Stats {
        std::atomic<uint64_t> read_blocked{0}, read_blocked_cycles{0}, read_blocked_max{0};
        std::atomic<uint64_t> write_waits{0}, write_wait_cycles{0}, write_wait_max{0}, courtesy{0}, courtesy_timeouts{0};
    };

    // ---- readers (any thread) ----
    void read_begin(Stats* st)
    {
        if (depth()++ > 0) return;
        if (try_enter()) return;
        uint64_t t0 = __rdtsc();
        waiting_.fetch_add(1, std::memory_order_seq_cst);
        for (unsigned n = 0; !try_enter(); n++) pause(n);
        waiting_.fetch_sub(1, std::memory_order_seq_cst);
        if (st) {
            uint64_t d = __rdtsc() - t0;
            st->read_blocked.fetch_add(1, std::memory_order_relaxed);
            st->read_blocked_cycles.fetch_add(d, std::memory_order_relaxed);
            note_max(st->read_blocked_max, d);
        }
    }
    void read_end()
    {
        if (--depth() > 0) return;
        readers_.fetch_sub(1, std::memory_order_seq_cst);
    }

    // ---- writers (the physics thread) ----
    void write_begin(Stats* st)
    {
        if (waiting_.load(std::memory_order_seq_cst)) {
            unsigned n = 0;
            while (waiting_.load(std::memory_order_seq_cst) && n < kCourtesySpins) pause(n++);
            if (st) {
                st->courtesy.fetch_add(1, std::memory_order_relaxed);
                if (n >= kCourtesySpins) st->courtesy_timeouts.fetch_add(1, std::memory_order_relaxed);
            }
        }
        // writers exclude each other too (today only the physics thread writes, in order, but nothing relies on it)
        for (unsigned n = 0;; n++) {
            int expected = 0;
            if (writer_.compare_exchange_weak(expected, 1, std::memory_order_seq_cst)) break;
            pause(n);
        }
        if (readers_.load(std::memory_order_seq_cst) == 0) return;
        uint64_t t0 = __rdtsc();
        for (unsigned n = 0; readers_.load(std::memory_order_seq_cst) != 0; n++) pause(n);
        if (st) {
            uint64_t d = __rdtsc() - t0;
            st->write_waits.fetch_add(1, std::memory_order_relaxed);
            st->write_wait_cycles.fetch_add(d, std::memory_order_relaxed);
            note_max(st->write_wait_max, d);
        }
    }
    void write_end() { writer_.store(0, std::memory_order_seq_cst); }

    // for tests
    int readers_inside() const { return readers_.load(); }
    bool writer_inside() const { return writer_.load() != 0; }

private:
    bool try_enter()
    {
        if (writer_.load(std::memory_order_seq_cst)) return false;
        readers_.fetch_add(1, std::memory_order_seq_cst);
        if (!writer_.load(std::memory_order_seq_cst)) return true;
        readers_.fetch_sub(1, std::memory_order_seq_cst);
        return false;
    }
    static void pause(unsigned n)
    {
        if (n < 2048) YieldProcessor();
        else SwitchToThread();
    }
    static int& depth()
    {
        thread_local int d = 0;
        return d;
    }
    static void note_max(std::atomic<uint64_t>& m, uint64_t d)
    {
        uint64_t v = m.load(std::memory_order_relaxed);
        while (d > v && !m.compare_exchange_weak(v, d)) {}
    }

    alignas(64) std::atomic<int> readers_{0};
    alignas(64) std::atomic<int> waiting_{0};
    alignas(64) std::atomic<int> writer_{0};
};
