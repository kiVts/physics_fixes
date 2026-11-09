#pragma once

// Parallel narrowphase (mt_dispatch.cpp).
bool mt_dispatch_active();          // collision pairs are currently running on several threads
bool mt_is_worker();                // this thread is running pairs of the current parallel phase
void mt_set_after_parallel(void (*fn)()); // adds a callback run on the physics thread after each parallel phase (max 4)
void* mt_last_pair_cache();         // the overlapping-pair cache of the world's last collision dispatch (physics thread)
