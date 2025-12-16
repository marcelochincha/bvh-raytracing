#pragma once

#include <cmath>
#include <vector>
#include <renderer/sr_camera.hpp>
#include <renderer/sr_primitives.hpp>
#include <renderer/sr_texture.hpp>
#include <SDL2/SDL.h>

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

    inline void clear(uint32_t clearColor, float clearDepth)
    {
        for (int i = 0; i < width * height; ++i)
        {
            colorBuffer[i] = clearColor;
            depthBuffer[i] = clearDepth;
        }
    }
};

struct render_coord
{
    vec3 p; // screen position
    vec3 t; // texture coord
};

inline void sort_vertices_by_y(vec3 &v0, vec3 &v1, vec3 &v2)
{
    if (v1.y < v0.y)
        std::swap(v0, v1);
    if (v2.y < v0.y)
        std::swap(v0, v2);
    if (v2.y < v1.y)
        std::swap(v1, v2);
}

inline void sort_render_coord_by_y(render_coord &v0, render_coord &v1, render_coord &v2)
{
    if (v1.p.y < v0.p.y)
        std::swap(v0, v1);
    if (v2.p.y < v0.p.y)
        std::swap(v0, v2);
    if (v2.p.y < v1.p.y)
        std::swap(v1, v2);
}

inline float interpolate_by_y(vec3 a, vec3 b, float y, vec_component c)
{
    if (a.y == b.y)
        return a.v[c];
    return a.v[c] + (b.v[c] - a.v[c]) * ((y - a.y) / (b.y - a.y));
}

inline vec3 convert_to_fb(const framebuffer &fb, const vec4 &v)
{
    return vec3(
        (v.x + 1.0f) * 0.5f * (fb.width - 1),
        (-v.y + 1.0f) * 0.5f * (fb.height - 1),
        v.z);
}

// Now add color scale?
inline uint32_t brightness_color(uint32_t color, float factor)
{
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    r = static_cast<uint8_t>(std::min(255.0f, r * factor));
    g = static_cast<uint8_t>(std::min(255.0f, g * factor));
    b = static_cast<uint8_t>(std::min(255.0f, b * factor));

    return (r << 16) | (g << 8) | b;
}

vec3 get_barycentric_coords(float x, float y, const vec3 &v0, const vec3 &v1, const vec3 &v2)
{
    // Calculate area of the full triangle
    float x1 = v0.x, y1 = v0.y;
    float x2 = v1.x, y2 = v1.y;
    float x3 = v2.x, y3 = v2.y;
    float denom = ((y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3));

    if (std::abs(denom) < 1e-6)
    {
        // Triángulo degenerado: devolver un “flag” inválido
        return vec3(-1, -1, -1);
    }

    vec3 r;
    r.x = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / denom;
    r.y = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / denom;
    r.z = 1.0 - r.x - r.y;
    return r;
}

// Simple triangle rasterization function
void render_flat_triangle(framebuffer &fb, vec3 v0, vec3 v1, vec3 v2, uint32_t color)
{
    sort_vertices_by_y(v0, v1, v2);
    int y_start = std::ceil(v0.y);
    int y_end = std::floor(v2.y);

    if (y_start < 0)
        y_start = 0;
    if (y_end >= fb.height)
        y_end = fb.height - 1;

    float x1, x2;
    float z1, z2;
    for (int y = y_start; y <= y_end; ++y)
    {
        x1 = interpolate_by_y(v0, v2, y, DIM_X);
        z1 = interpolate_by_y(v0, v2, y, DIM_Z);
        if (y < v1.y)
        {
            x2 = interpolate_by_y(v0, v1, y, DIM_X);
            z2 = interpolate_by_y(v0, v1, y, DIM_Z);
        }
        else
        {
            x2 = interpolate_by_y(v1, v2, y, DIM_X);
            z2 = interpolate_by_y(v1, v2, y, DIM_Z);
        }

        if (x1 > x2)
        {
            std::swap(x1, x2);
            std::swap(z1, z2);
        }

        int x_start = std::ceil(x1);
        int x_end = std::floor(x2);
        if (x_start < 0)
            x_start = 0;
        if (x_end >= fb.width)
            x_end = fb.width - 1;

        for (int x = x_start; x <= x_end; ++x)
        {
            // Perform depth test
            float denom = (x2 - x1);
            if (std::abs(denom) < 1e-6f)
            {
                continue; // degenerate horizontal span
            }
            float z = z1 + (z2 - z1) * ((x - x1) / denom);
            if (x >= 0 && x < fb.width && y >= 0 && y < fb.height && z >= -1 && z <= 1 && z < fb.depthBuffer[y * fb.width + x])
            {

                // Brightness adjustment
                fb.colorBuffer[y * fb.width + x] = color; // Simple brightness based on depth
                fb.depthBuffer[y * fb.width + x] = z;
            }
        }
    }
}

void draw_line(framebuffer &fb, int x0, int y0, int x1, int y1, uint32_t color)
{
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        if (x0 >= 0 && x0 < fb.width && y0 >= 0 && y0 < fb.height)
        {
            fb.colorBuffer[y0 * fb.width + x0] = color;
        }
        if (x0 == x1 && y0 == y1)
            break;
        int err2 = 2 * err;
        if (err2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (err2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

void render_triangle_lines(framebuffer &fb, vec3 v0, vec3 v1, vec3 v2, uint32_t color)
{
    sort_vertices_by_y(v0, v1, v2);
    draw_line(fb, static_cast<int>(std::round(v0.x)), static_cast<int>(std::round(v0.y)),
              static_cast<int>(std::round(v1.x)), static_cast<int>(std::round(v1.y)), color);
    draw_line(fb, static_cast<int>(std::round(v1.x)), static_cast<int>(std::round(v1.y)),
              static_cast<int>(std::round(v2.x)), static_cast<int>(std::round(v2.y)), color);
    draw_line(fb, static_cast<int>(std::round(v2.x)), static_cast<int>(std::round(v2.y)),
              static_cast<int>(std::round(v0.x)), static_cast<int>(std::round(v0.y)), color);
}

void render_texturized_triangle(framebuffer &fb, render_coord cv0, render_coord cv1, render_coord cv2, texture &tex)
{
    sort_render_coord_by_y(cv0, cv1, cv2);
    int y_start = std::ceil(cv0.p.y);
    int y_end = std::floor(cv2.p.y);

    // GET THE DIFERENCE OF MIN AND MAX Y

    int height = abs(cv0.p.y - cv2.p.y);
    int width = abs(std::max(std::max(cv0.p.x, cv1.p.x), cv2.p.x) - std::min(std::min(cv0.p.x, cv1.p.x), cv2.p.x));
    unsigned int area = width * height;
    // NOTE: previously triangles were skipped when their projected bounding-box
    // area was >= framebuffer area. That is too aggressive (especially when
    // the camera is inside a large cube) and causes faces to disappear.
    // We keep the area value for diagnostics but do not skip here.
    // printf("Triangle buffer area %d\n", int(width * height));

    vec3 &v0 = cv0.p;
    vec3 &v1 = cv1.p;
    vec3 &v2 = cv2.p;

    vec3 &v0_uv = cv0.t;
    vec3 &v1_uv = cv1.t;
    vec3 &v2_uv = cv2.t;

    float x1, x2;
    float z1, z2;
    for (int y = y_start; y <= y_end; ++y)
    {
        x1 = interpolate_by_y(v0, v2, y, DIM_X);
        z1 = interpolate_by_y(v0, v2, y, DIM_Z);
        if (y < v1.y)
        {
            x2 = interpolate_by_y(v0, v1, y, DIM_X);
            z2 = interpolate_by_y(v0, v1, y, DIM_Z);
        }
        else
        {
            x2 = interpolate_by_y(v1, v2, y, DIM_X);
            z2 = interpolate_by_y(v1, v2, y, DIM_Z);
        }

        if (x1 > x2)
        {
            std::swap(x1, x2);
            std::swap(z1, z2);
        }

        int x_start = std::ceil(x1);
        int x_end = std::floor(x2);

        for (int x = x_start; x <= x_end; ++x)
        {
            // Perform depth test
            float denom = (x2 - x1);
            if (std::abs(denom) < 1e-6f)
            {
                continue; // avoid divide by zero for narrow spans
            }
            float z = z1 + (z2 - z1) * ((x - x1) / denom);
            if (x >= 0 && x < fb.width && y >= 0 && y < fb.height && z >= -1 && z <= 1 && z < fb.depthBuffer[y * fb.width + x])
            {
                uint32_t color = 0xFFFFFFFF; // White for now
                vec3 bar = get_barycentric_coords(x, y, v0, v1, v2);
                float uz = bar.x * v0_uv.x + bar.y * v1_uv.x + bar.z * v2_uv.x;
                float vz = bar.x * v0_uv.y + bar.y * v1_uv.y + bar.z * v2_uv.y;
                float z_inv = bar.x * v0_uv.z + bar.y * v1_uv.z + bar.z * v2_uv.z;

                float s = uz / z_inv;
                float t = vz / z_inv;

                int t_x = int(s * (tex.width - 1));
                int t_y = int(t * (tex.height - 1));
                int target = t_y * tex.width + t_x;
                if (target < 0 || target >= tex.width * tex.height)
                    continue;
                color = tex.data[target];
                // Brightness adjustment
                fb.colorBuffer[y * fb.width + x] = color; // Simple brightness based on depth
                fb.depthBuffer[y * fb.width + x] = z;
            }
        }
    }
}

// draw call, not optimized but functional
void render_mesh(framebuffer &fb, const mesh &m, const camera &cam, uint32_t color)
{
    // Matrix for MODEL -> VIEW -> PROJECTION
    mat4 mvp = cam.projection() * cam.view() * m.modelMatrix;
    for (const auto &tri : m.faces)
    {
        vec4 clip_v0 = mvp * m.vertices[tri.v0].p; // vec4(m.vertices[tri.v0].p.x, m.vertices[tri.v0].p.y, m.vertices[tri.v0].p.z, 1.0f);
        vec4 clip_v1 = mvp * m.vertices[tri.v1].p; // vec4(m.vertices[tri.v1].p.x, m.vertices[tri.v1].p.y, m.vertices[tri.v1].p.z, 1.0f);
        vec4 clip_v2 = mvp * m.vertices[tri.v2].p; // vec4(m.vertices[tri.v2].p.x, m.vertices[tri.v2].p.y, m.vertices[tri.v2].p.z, 1.0f);

        // If all clip.w <= 0 then the whole triangle is behind the camera -> skip.
        // Use && so only triangles with ALL vertices behind the camera are skipped.
        if (clip_v0.w <= 0 && clip_v1.w <= 0 && clip_v2.w <= 0)
        {
            continue; // Entire triangle is behind the camera
        }
        // Perform perspective divide to get NDC
        vec4 ndc_v0 = clip_v0 / clip_v0.w;
        vec4 ndc_v1 = clip_v1 / clip_v1.w;
        vec4 ndc_v2 = clip_v2 / clip_v2.w;

        // Transform NDC to screen space
        vec3 screen_v0 = convert_to_fb(fb, ndc_v0);
        vec3 screen_v1 = convert_to_fb(fb, ndc_v1);
        vec3 screen_v2 = convert_to_fb(fb, ndc_v2);

        // Rasterize the triangle
        render_flat_triangle(fb, screen_v0, screen_v1, screen_v2, color);
    }
}

void render_wireframe(framebuffer &fb, const mesh &m, const camera &cam, uint32_t color)
{
    // Matrix for MODEL -> VIEW -> PROJECTION
    mat4 mvp = cam.projection() * cam.view() * m.modelMatrix;
    int cnt = 1;
    for (const auto &tri : m.faces)
    {
        vec4 clip_v0 = mvp * m.vertices[tri.v0].p;
        vec4 clip_v1 = mvp * m.vertices[tri.v1].p;
        vec4 clip_v2 = mvp * m.vertices[tri.v2].p;

        if (clip_v0.w <= 0.01 || clip_v1.w <= 0.01 || clip_v2.w <= 0.01)
        {
            continue; // Entire triangle is behind the camera
        }
        //if (cnt == 1)
        //{
        //    std::cout << "Rendering face " << cnt << "\n";
        //    std::cout << "Clip V0: (" << clip_v0.x << ", " << clip_v0.y << ", " << clip_v0.z << ", " << clip_v0.w << ")\n";
        //    std::cout << "Clip V1: (" << clip_v1.x << ", " << clip_v1.y << ", " << clip_v1.z << ", " << clip_v1.w << ")\n";
        //    std::cout << "Clip V2: (" << clip_v2.x << ", " << clip_v2.y << ", " << clip_v2.z << ", " << clip_v2.w << ")\n";
        //}
        // Perform perspective divide to get NDC
        vec4 ndc_v0 = clip_v0 / clip_v0.w;
        vec4 ndc_v1 = clip_v1 / clip_v1.w;
        vec4 ndc_v2 = clip_v2 / clip_v2.w;

        // Transform NDC to screen space
        vec3 screen_v0 = convert_to_fb(fb, ndc_v0);
        vec3 screen_v1 = convert_to_fb(fb, ndc_v1);
        vec3 screen_v2 = convert_to_fb(fb, ndc_v2);

        // Draw edges
        render_triangle_lines(fb, screen_v0, screen_v1, screen_v2, color * (color + 7 * cnt++));
    }
}

// Render textured mesh
void render_textured_mesh(framebuffer &fb, const mesh &m, const camera &cam, texture &tex)
{
    // Matrix for MODEL -> VIEW -> PROJECTION
    mat4 mvp = cam.projection() * cam.view() * m.modelMatrix;
    for (int index = 0; index < m.faces.size(); ++index)
    {
        const triangle &tri = m.faces[index];
        vec4 clip_v0 = mvp * m.vertices[tri.v0].p;
        vec4 clip_v1 = mvp * m.vertices[tri.v1].p;
        vec4 clip_v2 = mvp * m.vertices[tri.v2].p;

        // if (clip_v0.w <= 0.1 || clip_v1.w <= 0.1 || clip_v2.w <= 0.1)
        //{
        //     continue; // Entire triangle is behind the camera
        // }

        // Perform perspective divide to get NDC
        vec4 ndc_v0 = clip_v0 / clip_v0.w;
        vec4 ndc_v1 = clip_v1 / clip_v1.w;
        vec4 ndc_v2 = clip_v2 / clip_v2.w;

        // Transform NDC to screen space
        vec3 screen_v0 = convert_to_fb(fb, ndc_v0);
        vec3 screen_v1 = convert_to_fb(fb, ndc_v1);
        vec3 screen_v2 = convert_to_fb(fb, ndc_v2);

        vec3 uv_v0 = vec3(m.vertices[tri.v0].t.x, m.vertices[tri.v0].t.y, 1.0f) / clip_v0.w;
        vec3 uv_v1 = vec3(m.vertices[tri.v1].t.x, m.vertices[tri.v1].t.y, 1.0f) / clip_v1.w;
        vec3 uv_v2 = vec3(m.vertices[tri.v2].t.x, m.vertices[tri.v2].t.y, 1.0f) / clip_v2.w;

        // Prepare vertices with screen positions and texture coordinates
        render_coord cv0 = {screen_v0, uv_v0};
        render_coord cv1 = {screen_v1, uv_v1};
        render_coord cv2 = {screen_v2, uv_v2};
        render_texturized_triangle(fb, cv0, cv1, cv2, tex);
    }
}