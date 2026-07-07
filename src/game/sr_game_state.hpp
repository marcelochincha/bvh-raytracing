#pragma once
// Internal header — included only by game sub-modules, not exposed publicly.
#include <SDL2/SDL.h>
#include <renderer/sr_renderer.hpp>
#include <renderer/sr_skin.hpp>
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
    float move_speed = 6.0f;   // camera fly speed, adjusted by the mouse wheel

    float time          = 0.0f;
    bool  raytrace_mode = true;
    bool  show_bvh      = false;
    int   bvh_debug_depth = 12;  // wireframe: draw only nodes up to this depth

    bool show_menu   = false;
    int  menu_cursor = 0;
    int  scene_id    = 4;
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
    // --- Debug: single-ray path demo (press [R]) ---
    // A ray shot from the screen center: its bounce path plus every BVH box each
    // segment tests, so we can SEE the sequence of boxes it walks and why whole
    // subtrees get pruned. Rebuilt on demand from the CPU BVHs (Embree/GPU keep
    // their own, undrawable trees), drawn every frame while `active`.
    struct RayDebug {
        bool active = false;
        std::vector<vec3> path;                      // origin, hit1, hit2, ...
        std::vector<bvh::BVH::VisitedNode> visited;  // boxes tested (all segments)
    };
    RayDebug ray_debug;

    bvh::BuildStrategy build_strategy         = bvh::SAH;
    bvh::BuildStrategy dynamic_build_strategy = bvh::Morton;
    double             static_build_ms = 0.0;

    mesh*    field_mesh = nullptr;
    bvh::BVH static_bvh;

    // ---- Character skinning + camera path animation (scene "Character") ----
    // The skinned mesh lives in `meshes["character"]`, so build_scene_tris folds
    // its (per-frame deformed) triangles into the DYNAMIC BVH every frame — that
    // is the point: skinning + dynamic BVH rebuild coexist.
    SkinnedMesh skin;
    mesh*       skin_mesh = nullptr;

    struct CamKey { vec3 pos; vec3 euler_deg; }; // euler = (pitch, yaw, roll)
    std::vector<CamKey> cam_keys;   // authored/generated path (the loop)
    CamKey              cam_start{}; // pose the camera eased in from on play
    std::string         cam_anim_music;
    float cam_anim_fps  = 1.2f;     // keyframes advanced per second

    float anim_time    = 0.0f;      // unified clock: drives mesh + camera
    bool  anim_playing = false;     // toggled by [P] in the Character scene

#ifdef WITH_EMBREE
    embree_ref::Scene     embree_static_scene;
    embree_ref::Scene     embree_dynamic_scene;
    std::vector<bvh::Tri> embree_static_tris;
    std::vector<bvh::Tri> embree_dynamic_tris;
    bool                  embree_static_dirty = true;
#endif

    std::array<texture, 6> skybox_faces;
    bool skybox_enabled = true;

    static constexpr int MAX_WORKERS = 64;      // fixed cap for the arrays below
    int          num_workers                 = 4; // active count (set from --threads)
    SDL_Thread*  render_workers[MAX_WORKERS] = {};
    thread_data* worker_args                 = nullptr;
    SDL_sem*     start_sems[MAX_WORKERS]     = {};
    SDL_sem*     done_sem                    = nullptr;
    bool         workers_running             = true;

    Game(int width, int height) : fb(width, height) {}
};

struct thread_data {
    Game* game;
    int   thread_id;
    long  visits = 0;
};
