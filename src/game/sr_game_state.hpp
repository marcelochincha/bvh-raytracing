#pragma once
// Internal header — included only by game sub-modules, not exposed publicly.
#include <SDL2/SDL.h>
#include <renderer/sr_renderer.hpp>
#include <renderer/bvh.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <array>
#ifdef WITH_EMBREE
#include <renderer/embree_bvh.hpp>
#endif

using RTTri = bvh::Tri;

#define ENABLE_FIELD 1
#define FIELD_SLICES 10
#define FIELD_STACKS 10
#define CITY_SPAN    50.0f

struct thread_data;

enum class RenderBackend {
    Cpu = 0,
    OclGpu = 1,
    Embree = 2,
};

struct Game {
    framebuffer fb;
    camera cam;

    float yaw      = to_radians(0.0f);
    float pitch    = to_radians(-4.0f);
    vec3  position = vec3(0.0f, 1.7f, 1.5f);

    float time          = 0.0f;
    bool  raytrace_mode = true;
    bool  show_bvh      = false;

    bool show_menu   = false;
    int  menu_cursor = 0;
    int  scene_id    = 3;
    int  density     = 1;
    bool reflections = true;
    int  max_bounces = 1;
    int  spp         = 1;  // samples per pixel

    struct Ped { vec3 pos; vec3 skin; vec3 shirt; float phase; float speed; };
    std::vector<Ped> peds;
    float  bob_phase    = 0.0f;
    bool   show_hud     = true;
    bool   hud_simple   = false;
    bool   show_normals = false;
    RenderBackend backend = RenderBackend::Cpu;
    bool   fly_mode     = true;
    Uint32 last_space_ms = 0;

    std::unordered_map<std::string, mesh*> meshes;
    std::vector<RTTri> rt_tris;
    std::unordered_map<std::string, float> reflectivity_map;

    std::vector<bvh::Tri> emissive_tris; // area lights collected from static_bvh


    bvh::BVH           dynamic_bvh;
    bool               use_bvh                = true;
    bvh::BuildStrategy build_strategy         = bvh::SAH;
    bvh::BuildStrategy dynamic_build_strategy = bvh::Morton;
    double             static_build_ms = 0.0;

    mesh*    field_mesh = nullptr;
    bvh::BVH static_bvh;

#ifdef WITH_EMBREE
    embree_ref::Scene     embree_static_scene;
    embree_ref::Scene     embree_dynamic_scene;
    std::vector<bvh::Tri> embree_static_tris;
    std::vector<bvh::Tri> embree_dynamic_tris;
    bool                  embree_static_dirty = true;
#endif

    std::array<texture, 6> skybox_faces;
    bool skybox_enabled = true;

    static constexpr int NUM_WORKERS = 4;
    SDL_Thread*  render_workers[NUM_WORKERS] = {};
    thread_data* worker_args                 = nullptr;
    SDL_sem*     start_sems[NUM_WORKERS]     = {};
    SDL_sem*     done_sem                    = nullptr;
    bool         workers_running             = true;

    Game(int width, int height) : fb(width, height) {}
};

struct thread_data {
    Game* game;
    int   thread_id;
    long  visits = 0;
};
