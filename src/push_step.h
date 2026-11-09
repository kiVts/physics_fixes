#pragma once
#include <cmath>

namespace push_step {
// Preserve the game's elapsed time and substep cap. Only increase resolution;
// never slow a faster native world or turn a variable step into a fixed one.
inline double fixed(double native_fixed, double elapsed, int max_substeps, int hz, bool ready, bool walking)
{
    if (!ready || !walking || max_substeps <= 0 || hz < 60 || hz > 360 ||
        !std::isfinite(native_fixed) || native_fixed <= 0 || !std::isfinite(elapsed) || elapsed <= 0)
        return native_fixed;
    double requested = static_cast<double>(1.0f / static_cast<float>(hz));
    return requested < native_fixed ? requested : native_fixed;
}
}
