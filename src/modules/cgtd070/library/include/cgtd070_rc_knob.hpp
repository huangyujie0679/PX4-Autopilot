#pragma once

#include <stdint.h>

namespace cgtd070 {

// Endpoint-only three-state interpretation with hysteresis. A centered knob
// must cross 0.85 to move; an active endpoint must return inside 0.65 to stop.
inline int8_t rc_knob_position(float value, int8_t previous) noexcept
{
    constexpr float enter_threshold = 0.85F;
    constexpr float exit_threshold = 0.65F;

    if (previous > 0) {
        return value < exit_threshold ? 0 : 1;
    }
    if (previous < 0) {
        return value > -exit_threshold ? 0 : -1;
    }
    if (value >= enter_threshold) {
        return 1;
    }
    if (value <= -enter_threshold) {
        return -1;
    }
    return 0;
}

} // namespace cgtd070
