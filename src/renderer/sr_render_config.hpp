#pragma once

#include <cstdint>
#include <math/sr_math.hpp>
#include <renderer/sr_texture.hpp>

// Define here the configuration for drawing mesh
struct renderConfig
{
    uint32_t baseColor = 0xFFFFFFFF; // Base color to use if no texture is set
    texture* tex = nullptr; // Pointer to texture to use
    float lightInfluence = 1.0f; // How much the light affects the base color [0..1]
    bool ignoreDepth = false; // Whether to  NOT update and check depth buffer.
    bool ignoreLight = false; // Whether to ignore lighting calculations
};

