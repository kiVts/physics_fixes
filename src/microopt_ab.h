#pragma once
#include <atomic>

extern std::atomic<bool> g_microopt_off; // true during an OFF window
inline bool microopt_on() { return !g_microopt_off.load(std::memory_order_relaxed); }
