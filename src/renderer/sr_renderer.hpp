#pragma once

#include <cmath>
#include <vector>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#include <renderer/sr_camera.hpp>
#include <renderer/sr_primitives.hpp>
#include <renderer/sr_texture.hpp>
#include <renderer/sr_text.h>
#include <SDL2/SDL.h>

struct rasterCoord
{
    vec3 p; // screen position
    vec3 t; // texture coord
};

struct clipVertex
{
    vec4 pos; // clip-space position
    vec2 uv;  // raw uv (no division by w). Perspective correction is applied later.
};

struct pixelCoord
{
    vec2 p;
    vec2 t;
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

inline void sort_raster_coord_by_y(rasterCoord &v0, rasterCoord &v1, rasterCoord &v2)
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

inline vec3 convert_to_fb(const framebuffer &fb, const vec3 &v)
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

// Clipping algorithms
inline vec3 get_normal(const vec3 &v0, const vec3 &v1, const vec3 &v2)
{
    vec3 vA = v1 - v0;
    vec3 vB = v2 - v0;
    return normalize(cross(vA, vB));
}

inline clipVertex lerp_clip(const clipVertex &a, const clipVertex &b, float t)
{
    clipVertex r;
    r.pos = a.pos + (b.pos - a.pos) * t;
    r.uv = a.uv + (b.uv - a.uv) * t;
    return r;
}

std::vector<clipVertex> clip_against_plane(const std::vector<clipVertex> &poly, float (*inside_fn)(const clipVertex &))
{
    std::vector<clipVertex> out;
    if (poly.empty())
        return out;
    clipVertex S = poly.back();
    float dS = inside_fn(S);
    for (const auto &E : poly)
    {
        float dE = inside_fn(E);
        bool S_in = dS <= 0.0f;
        bool E_in = dE <= 0.0f;
        if (S_in && E_in)
        {
            out.push_back(E);
        }
        else if (S_in && !E_in)
        {
            float denom = dS - dE;
            if (std::abs(denom) > 1e-6f)
            {
                float t = dS / denom;
                out.push_back(lerp_clip(S, E, t));
            }
        }
        else if (!S_in && E_in)
        {
            float denom = dS - dE;
            if (std::abs(denom) > 1e-6f)
            {
                float t = dS / denom;
                out.push_back(lerp_clip(S, E, t));
            }
            out.push_back(E);
        }
        S = E;
        dS = dE;
    }
    return out;
}

// Main shader sample definition
typedef uint32_t (*pixelShaderFunc)(pixelCoord, void *);
struct pixelShader
{
    pixelShaderFunc func;
    void *data;
};

// Simple Bresenham line drawing
void draw_line(framebuffer &fb, int x0, int y0, int x1, int y1, uint32_t color, bool use_depth = false)
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
            if (use_depth)
            {
                fb.depthBuffer[y0 * fb.width + x0] = 0.0f; // Set depth to 0 for lines if needed
            }
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

void render_triangle(framebuffer &fb, rasterCoord cv0, rasterCoord cv1, rasterCoord cv2, const pixelShader &pixel_shader)
{
    sort_raster_coord_by_y(cv0, cv1, cv2);
    int y_start = std::ceil(cv0.p.y);
    int y_end = std::floor(cv2.p.y);

    // GET THE DIFERENCE OF MIN AND MAX Y
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
            float z_denom = (x2 - x1);
            if (std::abs(z_denom) < 1e-6f)
            {
                continue; // avoid divide by zero for narrow spans
            }
            float z = z1 + (z2 - z1) * ((x - x1) / z_denom);
            if (x >= 0 && x < fb.width && y >= 0 && y < fb.height && z < fb.depthBuffer[y * fb.width + x])
            {
                // printf("DRAWING \n");
                //  Execute barycentric interpolations
                vec3 bar = get_barycentric_coords(x, y, v0, v1, v2);
                float uz = bar.x * v0_uv.x + bar.y * v1_uv.x + bar.z * v2_uv.x;
                float vz = bar.x * v0_uv.y + bar.y * v1_uv.y + bar.z * v2_uv.y;
                float z_inv = bar.x * v0_uv.z + bar.y * v1_uv.z + bar.z * v2_uv.z;

                float s = uz / z_inv;
                float t = vz / z_inv;

                pixelCoord pixelInput{vec2{x, y}, vec2{s, t}};

                fb.colorBuffer[y * fb.width + x] = pixel_shader.func(pixelInput, pixel_shader.data);
                fb.depthBuffer[y * fb.width + x] = z;
            }

            /*
            // Perform depth test
            float denom = (x2 - x1);
            if (std::abs(denom) < 1e-6f)
            {
                continue; // avoid divide by zero for narrow spans
            }
            float z = z1 + (z2 - z1) * ((x - x1) / denom);
            if (z < fb.depthBuffer[y * fb.width + x])
            {
                uint32_t color = 0xFFFFFFFF; // White for now
                vec3 bar = get_barycentric_coords(x, y, v0, v1, v2);
                float uz = bar.x * v0_uv.x + bar.y * v1_uv.x + bar.z * v2_uv.x;
                float vz = bar.x * v0_uv.y + bar.y * v1_uv.y + bar.z * v2_uv.y;
                float z_inv = bar.x * v0_uv.z + bar.y * v1_uv.z + bar.z * v2_uv.z;

                float s = uz / z_inv;
                float t = vz / z_inv;

                // Check if uvs can be outside 0-1 range
                if (s < 0.0f || s > 1.0f || t < 0.0f || t > 1.0f)
                    continue;

                int t_x = int(s * (tex.width));
                int t_y = int(t * (tex.height));

                // printf("Tex coords: %f, %f -> %d, %d\n", s, t, t_x, t_y);

                int target = t_y * tex.width + t_x;
                if (target < 0 || target >= tex.width * tex.height)
                    continue;
                color = tex.data[target];
                // Brightness adjustment
                fb.colorBuffer[y * fb.width + x] = color; // Simple brightness based on depth
                if (use_depth)
                    fb.depthBuffer[y * fb.width + x] = z;
            }*/
        }
    }
}

// Main render function
void render_mesh(framebuffer &fb, const camera &cam, const mesh &m, pixelShader &pixel_shader)
{
    // Matrixs for MODEL -> VIEW -> PROJECTION
    // mat4 model_mat = m.modelMatrix();
    // mat4 vp = cam.projection() * cam.view();
    mat4 mvp = cam.projection() * cam.view() * m.modelMatrix();
    mat4 mv = cam.view() * m.modelMatrix();
    for (int index = 0; index < m.faces.size(); ++index)
    {
        const triangle &tri = m.faces[index];

        // Now compute for brightness :v
        vec3 v0L = mv * m.vertices[tri.v0].p;
        vec3 v1L = mv * m.vertices[tri.v1].p;
        vec3 v2L = mv * m.vertices[tri.v2].p;

        // Produce normal real
        vec3 vNormal_L = get_normal(v0L, v1L, v2L);
        float d = dot(normalize(vec3(0, 0, 1)), normalize(vNormal_L));

        uint32_t target_color = 0xFFFFFFFF; // Default white
        // if (d < 0.0f)
        //     target_color = (uint32_t)((-d + 1.0f) * 0xFF120100); // Dark gray for backfaces

        if (tri.colorIndex < m.colors.size())
            target_color = m.colors[tri.colorIndex];

        uint8_t *p = (uint8_t *)pixel_shader.data;
        uint32_t *color = (uint32_t *)(p + 0);
        float *brightness = (float *)(p + 4);

        //*((float *)(pixel_shader.data)) = 0.5 + fmaxf(0.0f,d) * 0.5;
        *color = target_color;
        *brightness = 0.5 + fmaxf(0.0f, d) * 0.5;

        // pixel_shader.data = (void *)brightness;

        clipVertex v0{mvp * m.vertices[tri.v0].p, m.vertices[tri.v0].t};
        clipVertex v1{mvp * m.vertices[tri.v1].p, m.vertices[tri.v1].t};
        clipVertex v2{mvp * m.vertices[tri.v2].p, m.vertices[tri.v2].t};

        std::vector<clipVertex> poly = {v0, v1, v2};
        poly = clip_against_plane(poly, [](const clipVertex &v)
                                  { return v.pos.x - v.pos.w; }); // x <=  w
        poly = clip_against_plane(poly, [](const clipVertex &v)
                                  { return -v.pos.x - v.pos.w; }); // x >= -w
        poly = clip_against_plane(poly, [](const clipVertex &v)
                                  { return v.pos.y - v.pos.w; }); // y <=  w
        poly = clip_against_plane(poly, [](const clipVertex &v)
                                  { return -v.pos.y - v.pos.w; }); // y >= -w
        poly = clip_against_plane(poly, [](const clipVertex &v)
                                  { return v.pos.z - v.pos.w; }); // z <=  w
        poly = clip_against_plane(poly, [](const clipVertex &v)
                                  { return -v.pos.z - v.pos.w; }); // z >= -w (near)

        if (poly.size() < 3)
            continue;

        // Get the min and max points possible in screenspace;
        // vec2 min_p(fb.width, fb.height), max_p(-1, -1);
        clipVertex base = poly[0];
        for (size_t i = 1; i + 1 < poly.size(); ++i)
        {
            clipVertex a = poly[i];
            clipVertex b = poly[i + 1];

            // GTET THE NORMAL
            vec3 ndc_v0 = base.pos / base.pos.w;
            vec3 ndc_v1 = a.pos / a.pos.w;
            vec3 ndc_v2 = b.pos / b.pos.w;

            // vec3 face_normal = get_normal(ndc_v0, ndc_v1, ndc_v2);
            // float d_face = dot(normalize(vec3(0, 0, 1)), normalize(face_normal));

            // Compute normal here
            vec3 screen_v0 = convert_to_fb(fb, ndc_v0);
            vec3 screen_v1 = convert_to_fb(fb, ndc_v1);
            vec3 screen_v2 = convert_to_fb(fb, ndc_v2);

            // Update min and max points
            // min_p.x = std::min({min_p.x, screen_v0.x, screen_v1.x, screen_v2.x});
            // min_p.y = std::min({min_p.y, screen_v0.y, screen_v1.y, screen_v2.y});
            // max_p.x = std::max({max_p.x, screen_v0.x, screen_v1.x, screen_v2.x});
            // max_p.y = std::max({max_p.y, screen_v0.y, screen_v1.y, screen_v2.y});

            // Check if the triangle is looking towards the camera
            vec2 e0 = screen_v1 - screen_v0;
            vec2 e1 = screen_v2 - screen_v0;

            float area = e0.x * e1.y - e0.y * e1.x;

            if (area > 0)
                continue; // backface

            vec3 uvw0 = vec3(base.uv.x, base.uv.y, 1.0f) / base.pos.w;
            vec3 uvw1 = vec3(a.uv.x, a.uv.y, 1.0f) / a.pos.w;
            vec3 uvw2 = vec3(b.uv.x, b.uv.y, 1.0f) / b.pos.w;

            rasterCoord cv0 = {screen_v0, uvw0};
            rasterCoord cv1 = {screen_v1, uvw1};
            rasterCoord cv2 = {screen_v2, uvw2};

            render_triangle(fb, cv0, cv1, cv2, pixel_shader);

            // vec2 text_pos = (screen_v0 + screen_v1 + screen_v2) / 3.0f;
            // char buf[32];
            // snprintf(buf, sizeof(buf), "%.2f", d_face);
            // if (d < 0.0f)
            //     draw_text(fb, text_pos.x, text_pos.y, buf, 0xFFFF0000);
            // else
            //     draw_text(fb, text_pos.x, text_pos.y, buf, 0xFFFFFF00);

            // draw_line(fb, screen_v0.x, screen_v0.y, screen_v1.x, screen_v1.y, 0xFF00FF00);
            // draw_line(fb, screen_v1.x, screen_v1.y, screen_v2.x, screen_v2.y, 0xFF00FF00);
            // draw_line(fb, screen_v2.x, screen_v2.y, screen_v0.x, screen_v0.y, 0xFF00FF00);

            // RENDER THE POINT NORMAL
            // draw_line(fb, pointStart.x, pointStart.y, pointEnd.x, pointEnd.y, 0xFF0000FF);

            // s.func(fb,cv0,cv1,cv2,s.data);
            ////render_texturized_triangle(fb, cv0, cv1, cv2, tex, use_depth);
            // render_triangle_lines(fb, screen_v0, screen_v1, screen_v2, color * (color + 7 * (index++ + 1)));
        }

        // Draw the debug d value in the mean point
        // vec2 text_pos = (min_p + max_p) / 2.0f;
        // char buf[32];
        // snprintf(buf, sizeof(buf), "%.2f", d);
        // if (d < 0.0f)
        //     draw_text(fb, text_pos.x, text_pos.y, buf, 0xFFFF0000);
        // else
        //     draw_text(fb, text_pos.x, text_pos.y, buf, 0xFFFFFF00);

        // draw_line(fb, vCenterPoint_s.x, vCenterPoint_s.y, vNormalPoint_s.x, vNormalPoint_s.y, 0xFF0000FF,true);
    }
}

inline void draw_gizmo_line(framebuffer &fb, const camera &cam, const vec3 &start, const vec3 &end, uint32_t color)
{
    vec4 start_clip = cam.projection() * cam.view() * vec4(start.x, start.y, start.z, 1.0f);
    vec4 end_clip = cam.projection() * cam.view() * vec4(end.x, end.y, end.z, 1.0f);

    if (start_clip.w <= 0 || end_clip.w <= 0)
        return;

    vec3 start_ndc = start_clip / start_clip.w;
    vec3 end_ndc = end_clip / end_clip.w;

    vec3 start_screen = convert_to_fb(fb, start_ndc);
    vec3 end_screen = convert_to_fb(fb, end_ndc);
    if (start_screen.x < 0 || end_screen.x < 0 || start_screen.x >= fb.width || end_screen.x >= fb.width ||
        start_screen.y < 0 || end_screen.y < 0 || start_screen.y >= fb.height || end_screen.y >= fb.height)
        return;

    draw_line(fb, start_screen.x, start_screen.y, end_screen.x, end_screen.y, color);
}

void render_gizmo(framebuffer &fb, const camera &cam, const vec3 &pos, float size)
{
    // X axis - Red
    draw_gizmo_line(fb, cam, pos, pos + vec3(size, 0, 0), 0xFFFF0000);
    // Y axis - Green
    draw_gizmo_line(fb, cam, pos, pos + vec3(0, size, 0), 0xFF00FF00);
    // Z axis - Blue
    draw_gizmo_line(fb, cam, pos, pos + vec3(0, 0, size), 0xFF0000FF);
}