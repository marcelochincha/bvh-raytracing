#pragma once

#include "sr_constants.hpp"
#include "sr_vec.hpp"

// ANGLE HELPERS
inline float to_radians(float degrees)
{
    return degrees * (SR_PI / 180.0f);
}

inline float to_degrees(float radians)
{
    return radians * (180.0f / SR_PI);
}

// Additional math helper functions can be added here