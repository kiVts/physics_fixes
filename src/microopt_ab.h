#pragma once
// A/B switch for the update-thread micro-opts (box_aabb.cpp, pose_cache.cpp, render_mt.cpp's update pool).
// [microopt] ab=1: they turn off and on together every 30 s ("micro-opts: ON window" / "OFF window" lines), so the
// diagnostics timeline lines can be split by window in one run. ab=0 (default): always on.
#include <atomic>

extern std::atomic<bool> g_microopt_off; // true during an OFF window
inline bool microopt_on() { return !g_microopt_off.load(std::memory_order_relaxed); }
