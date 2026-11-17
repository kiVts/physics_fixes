#pragma once
// Lua VM tokens (vehicle_mt.cpp, [vehicle_logic] lua_unlocked): one owner thread per global_State at a time.
// Every Lua entry while unlocked Lua is on - the onTick caller (unlocked or locked) and the other Lua sinks (create /
// compile / close / save / editor), on any thread - holds its VM's token for the call. That is what serializes them:
// an unlocked onTick doesn't hold the world lock. Lock order: tokens before the world lock. A thread that holds no
// lock and no other token may wait for a busy token (token holders only ever wait for the world lock, so no cycle);
// one that holds the world lock or another token may not (vehicle_mt reports a conflict instead).
// Free of game state.
#include <windows.h>
#include <intrin.h>
#include <cstdint>

namespace lua_tokens {
constexpr int kWays = 16;
struct Stripe {
    SRWLOCK lock = SRWLOCK_INIT;
    uintptr_t g[kWays] = {};
    DWORD tid[kWays] = {};
};
inline Stripe g_stripes[64];
inline thread_local DWORD tl_tid = 0;
inline thread_local int tl_held = 0; // tokens this thread holds

inline DWORD my_tid() { return tl_tid ? tl_tid : (tl_tid = GetCurrentThreadId()); }
inline Stripe& stripe(uintptr_t G) { return g_stripes[((G >> 4) ^ (G >> 12)) & 63]; }

enum State { kTaken, kOwn, kBusy, kFull };

// one attempt: kTaken (now held, tl_held counted), kOwn (this thread holds it already), kBusy (another thread does),
// kFull (no free way in its stripe)
inline State take(uintptr_t G)
{
    Stripe& s = stripe(G);
    const DWORD me = my_tid();
    AcquireSRWLockExclusive(&s.lock);
    int free_i = -1;
    State r = kFull;
    for (int i = 0; i < kWays; i++) {
        if (s.g[i] == G) {
            r = s.tid[i] == me ? kOwn : kBusy;
            free_i = -2;
            break;
        }
        if (!s.g[i] && free_i == -1) free_i = i;
    }
    if (free_i >= 0) {
        s.g[free_i] = G;
        s.tid[free_i] = me;
        r = kTaken;
    }
    ReleaseSRWLockExclusive(&s.lock);
    if (r == kTaken) tl_held++;
    return r;
}

// gives back a token this thread took
inline void drop(uintptr_t G)
{
    Stripe& s = stripe(G);
    const DWORD me = my_tid();
    AcquireSRWLockExclusive(&s.lock);
    for (int i = 0; i < kWays; i++)
        if (s.g[i] == G && s.tid[i] == me) {
            s.g[i] = 0;
            s.tid[i] = 0;
            break;
        }
    ReleaseSRWLockExclusive(&s.lock);
    tl_held--;
}

// take(), waiting while busy / full when `may_wait`; *waited = it had to
inline State take_or_wait(uintptr_t G, bool may_wait, bool* waited)
{
    State t = take(G);
    *waited = false;
    if (!may_wait) return t;
    for (unsigned n = 0; t == kBusy || t == kFull; ++n) {
        *waited = true;
        if (n < 64) _mm_pause();
        else SwitchToThread();
        t = take(G);
    }
    return t;
}
}
