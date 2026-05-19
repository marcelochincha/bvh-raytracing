#include <renderer/sr_texture.hpp>


bool load_png_texture(const std::string &filename, texture &m, int max_size)
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

    uint8_t *raw_data = stbi_load(filename.c_str(), &tex_width, &tex_height, &channels, 4);
    if (!raw_data)
    {
        printf("Failed to load texture: %s\n", filename.c_str());
        m = texture(0, 0);
        return false;
    }

    int dst_w = tex_width;
    int dst_h = tex_height;
    if (max_size > 0 && (dst_w > max_size || dst_h > max_size))
    {
        float scale = (float)max_size / (float)(dst_w > dst_h ? dst_w : dst_h);
        dst_w = (int)(dst_w * scale);
        dst_h = (int)(dst_h * scale);
    }

    m.width  = dst_w;
    m.height = dst_h;
    m.data   = new uint32_t[dst_w * dst_h];

    float sx = (float)tex_width  / dst_w;
    float sy = (float)tex_height / dst_h;

    for (int y = 0; y < dst_h; ++y)
    {
        int src_y = tex_height - 1 - (int)(y * sy); // flip Y
        for (int x = 0; x < dst_w; ++x)
        {
            int src_x   = (int)(x * sx);
            int src_idx = (src_y * tex_width + src_x) * 4;
            uint8_t r = raw_data[src_idx + 0];
            uint8_t g = raw_data[src_idx + 1];
            uint8_t b = raw_data[src_idx + 2];
            uint8_t a = raw_data[src_idx + 3];
            m.data[y * dst_w + x] = (a << 24) | (r << 16) | (g << 8) | b;
        }
    }

    stbi_image_free(raw_data);

    if (dst_w != tex_width || dst_h != tex_height)
        printf("  resized to %dx%d\n", dst_w, dst_h);

    return true;
}
