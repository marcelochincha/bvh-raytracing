#pragma once

#include <io/stb_image.hpp>
#include <cstdint>
#include <iostream>

struct texture
{
    int width = 0;
    int height = 0;
    uint32_t *data = nullptr; // Pointer to texture data in ARGB format

    texture(int w = 0, int h = 0)
        : width(w), height(h) {}

    ~texture()
    {
        delete[] data;
    }
};

bool load_png_texture(const std::string &filename, texture &m, int max_size = 256);