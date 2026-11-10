#pragma once


// [parallel] early_launch (1 by default): true once the guard's hooks are all in place, and the physics step then runs
// alongside the logic.
bool parallel_on();

// Shared side of the world lock (parallel_guard.cpp): hold it while reading Bullet's broadphase trees or object
// transforms outside the physics thread. No-ops when parallel_on() is false or the thread holds the exclusive side.
// Reads may nest on one thread (world_lock.h counts the depth).
void world_read_begin();
void world_read_end();
