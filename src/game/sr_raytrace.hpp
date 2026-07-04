#pragma once
#include <game/sr_game_state.hpp>
#include <cstdint>

extern const vec3  SUN_DIR;
extern const float AMBIENT;
extern const float SHADOW_EPS;

struct ray {
    vec3 origin;
    vec3 direction;
    ray(const vec3& o, const vec3& d) : origin(o), direction(d) {}
};

uint32_t pack(vec3 c);
vec3  get_ray_direction(const camera& cam, int px, int py, int w, int h);
vec3  trace_ray(const ray& r, const Game& e, int depth, uint32_t& seed, bool skip_emission = false);
vec3  trace_ray(const ray& r, const Game& e);  // seed-free overload for callers
int   worker_thread(void* data);
