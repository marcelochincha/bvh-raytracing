#pragma once
#include <cstdint>

// Framebuffer structure
struct framebuffer
{
    uint32_t *colorBuffer;
    float *depthBuffer;
    int width;
    int height;

    framebuffer(int w, int h)
        : width(w), height(h)
    {
        colorBuffer = new uint32_t[width * height];
        depthBuffer = new float[width * height];
    }

    ~framebuffer()
    {
        delete[] colorBuffer;
        delete[] depthBuffer;
    }

    inline void clear(uint32_t clearColor)
    {
        for (int i = 0; i < width * height; ++i)
        {
            colorBuffer[i] = clearColor;
            depthBuffer[i] = 1.0f; // Clear depth to far plane
        }
    }
};