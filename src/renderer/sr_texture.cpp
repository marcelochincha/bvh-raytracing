#include <renderer/sr_texture.hpp>

bool load_png_texture(const std::string &filename, texture &m)
{
    int channels;
    int tex_width, tex_height;
    if (stbi_info(filename.c_str(), &tex_width, &tex_height, &channels))
    {
        printf("Found texture: %s with dimensions => (%dx%d)\n", filename.c_str(), tex_width, tex_height);
    }
    else
    {
        printf("Failed to get info for texture: %s\n", filename.c_str());
        m = texture(0, 0);
        return false;
    }

    uint8_t *raw_data = stbi_load(filename.c_str(), &m.width, &m.height, &channels, 4); // fuerza RGB
    if (!raw_data)
    {
        printf("Failed to load texture: %s\n", filename.c_str());
        m = texture(0, 0);
        return false;
    }

    // Invertir y convertir a ARGB
    m.data = new uint32_t[m.width * m.height];
    for (int y = 0; y < m.height; ++y)
    {
        for (int x = 0; x < m.width; ++x)
        {
            int src_index = ((m.height - 1 - y) * m.width + x) * 4; // Invertir Y
            int dst_index = y * m.width + x;
            uint8_t r = raw_data[src_index + 0];
            uint8_t g = raw_data[src_index + 1];
            uint8_t b = raw_data[src_index + 2];
            uint8_t a = raw_data[src_index + 3];
            m.data[dst_index] = (a << 24) | (r << 16) | (g << 8) | b; // ARGB
        }
    }

    stbi_image_free(raw_data);
    // Reservamos espacio
    return true;
}