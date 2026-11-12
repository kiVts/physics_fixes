#pragma once


// False in release builds while the stability-policy gate is closed. If that
// gate is eventually opened, true only after every companion guard is installed.
bool parallel_on();

// Shared side of the world lock (parallel_guard.cpp): hold it while reading Bullet's broadphase trees or object
// transforms outside the physics thread. No-ops when parallel_on() is false or the thread holds the exclusive side.
// Reads may nest on one thread (world_lock.h counts the depth).
void world_read_begin();
void world_read_end();

// The caller must release on the same thread, on normal and exceptional exits,
// BEFORE waiting for any physics task whose callback may be blocked here.
bool parallel_phase_begin();
void parallel_phase_end();
