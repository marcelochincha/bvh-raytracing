#pragma once

#include <cmath>
#include <cstdint>
#include <cmath>
#include <renderer/sr_geometry.hpp>
#include <renderer/sr_texture.hpp>
#include <renderer/sr_text.hpp>
#include <renderer/sr_camera.hpp>
#include <renderer/sr_render_config.hpp>
#include <renderer/sr_framebuffer.hpp>

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

//void sort_vertices_by_y(vec3 &v0, vec3 &v1, vec3 &v2);
//void sort_raster_coord_by_y(rasterCoord &v0, rasterCoord &v1, rasterCoord &v2);
//float interpolate_by_y(vec3 a, vec3 b, float y, vec_component c);
//vec3 convert_to_fb(const framebuffer &fb, const vec3 &v);
//uint32_t brightness_color(uint32_t color, float factor);
//uint32_t sample_texture(const texture *tex, float s, float t);
//vec3 get_barycentric_coords(float x, float y, const vec3 &v0, const vec3 &v1, const vec3 &v2);
//vec3 get_normal(const vec3 &v0, const vec3 &v1, const vec3 &v2);
//clipVertex lerp_clip(const clipVertex &a, const clipVertex &b, float t);
//typedef float (*insidePlaneFn)(const clipVertex &);
//std::vector<clipVertex> clip_against_plane(const std::vector<clipVertex> &poly, insidePlaneFn inside_fn);
//float plane_left(const clipVertex &v);
//float plane_right(const clipVertex &v);
//float plane_top(const clipVertex &v);
//float plane_bottom(const clipVertex &v);
//float plane_far(const clipVertex &v);
//float plane_near(const clipVertex &v);
void draw_line(framebuffer &fb, int x0, int y0, int x1, int y1, uint32_t color, bool use_depth = false);
void render_triangle(framebuffer &fb, rasterCoord cv0, rasterCoord cv1, rasterCoord cv2, const renderConfig &config);
void render_mesh(framebuffer &fb, const camera &cam, const mesh &m, renderConfig &config);
void render_skybox(framebuffer &fb, const camera &cam, std::array<texture, 6> &skyboxTextures);
void draw_gizmo_line(framebuffer &fb, const camera &cam, const vec3 &start, const vec3 &end, uint32_t color);
void render_gizmo(framebuffer &fb, const camera &cam, const vec3 &pos, float size);
