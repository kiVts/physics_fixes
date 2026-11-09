#pragma once

// One worker pool for the physics thread's parallel phases (phys_pool.cpp): broadphase searches, collision pairs and
// the solver used to have a pool each (15 threads waking 9 times per tick).

int phys_pool_threads();
void phys_pool_run(int helpers, void (*fn)(int slot, void* ctx), void* ctx);
