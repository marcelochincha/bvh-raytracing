#include <game/sr_game.hpp>

#include <sr_config.hpp>
#include <unordered_map>
#include <renderer/sr_renderer.hpp>
#include <renderer/bvh.hpp>
#include <algorithm>
#include <string>
#include <cmath>
#include <fstream>
#include <sstream>

// =============================================================================
// Minimal "old-school computer graphics" scene: a floor and a few primitives,
// a free-fly camera (WASD + mouse look). No physics, no gameplay — just a
// static set of meshes to develop/visualize the renderer (and the upcoming
// raytracer) against.
// =============================================================================

// Set to 0 to skip loading the heavy room .ply (faster startup / lighter scene).
// Everything else is guarded on room_bvh/room_mesh being empty, so it just works.
#define ENABLE_ROOM 0

// Procedural "scatter field": a jittered grid of colored spheres, built ONCE
// into a static BVH. Replaces the placeholder primitives with a rich, high-
// triangle scene so the BVH actually matters (brute force becomes hopeless).
// The count is tunable for the scaling experiments.
#define ENABLE_FIELD 1
#define FIELD_SPHERES 256   // grid side = round(sqrt(this)); 256 -> 16x16
#define FIELD_SLICES  12
#define FIELD_STACKS  8
#define CITY_SPAN     50.0f // city footprint side (floor is a bit bigger)

const float kMouseSensitivity = 0.0025f;
const float moveSpeed = 6.0f;

// The raytracer triangle lives in the BVH module now (geometry + shading
// payload). Alias keeps the rest of this file reading the same.
using RTTri = bvh::Tri;

struct Game
{
    framebuffer fb;
    camera cam;

    // Free-fly camera state. Default = standing at street level at the south
    // edge of the city, looking in (-z) into the blocks (a "walking" vantage).
    float yaw   = to_radians(0.0f);
    float pitch = to_radians(-4.0f);
    vec3  position = vec3(0.0f, 1.7f, CITY_SPAN * 0.5f + 1.0f);

    float time = 0.0f; // seconds elapsed, drives scene animation
    bool  raytrace_mode = true; // TAB toggles raytracer <-> rasterizer
    bool  show_bvh = false;     // [V] overlays the BVH boxes

    // --- in-app options menu ([M]) ---
    bool  show_menu   = false;  // panel open?
    int   menu_cursor = 0;      // selected row
    int   scene_id    = 0;      // 0 = city, 1 = spheres (procedural static field)
    int   density     = 1;      // 0 = Small, 1 = Medium, 2 = Large
    bool  reflections = true;   // recursive mirror bounces on/off
    int   max_bounces = 1;      // ray recursion depth

    // --- "alive city" extras ---
    struct Ped { vec3 pos; vec3 shirt; float phase; };
    std::vector<Ped> peds;      // animated pedestrians (dynamic BVH)
    float bob_phase = 0.0f;     // first-person walk head-bob accumulator
    bool  fly_mode = false;     // walk (ground) vs free-fly (double-tap SPACE)
    Uint32 last_space_ms = 0;   // for double-tap detection

    std::unordered_map<std::string, mesh*> meshes;
    std::vector<RTTri> rt_tris; // Scene data preprocessed for ray tracing (triangles with precomputed normals and albedos)
    std::unordered_map<std::string, float> reflectivity_map; // For rendering simple shapes like axes, grids, etc.

    bvh::BVH bvh;            // acceleration structure, rebuilt each frame
    bool     use_bvh = true; // [B] toggles BVH vs brute-force triangle loop
    bvh::BuildStrategy build_strategy = bvh::SAH; // how the trees are split
    double field_build_ms = 0.0;   // last field BVH build time (strategy comparison)

    // The static room (PLY) gets its OWN BVH, built ONCE in game_init. It has
    // hundreds of thousands of triangles, so it must NOT go through the
    // per-frame rebuild that the animated shapes use. trace_ray queries this in
    // addition to the dynamic BVH and keeps whichever hit is nearer. (The
    // [B] brute-force toggle only affects the small dynamic scene; the room is
    // always traced via its BVH — brute-forcing 380k tris/pixel is hopeless.)
    bvh::BVH room_bvh;
    // Same room geometry as a plain mesh (engine space, ceiling already cut) so
    // the rasterizer can draw it too. Kept OUT of `meshes` on purpose, so it
    // never enters the per-frame dynamic BVH rebuild.
    mesh* room_mesh = nullptr;
    // Same field geometry as a plain mesh so the rasterizer can draw it too
    // (the ray tracer reads field_bvh; this mirror lets TAB show the city in
    // both renderers, flat-shaded in raster).
    mesh* field_mesh = nullptr;

    // Procedural static field (jittered grid of spheres). Built ONCE in
    // game_init, exactly like the room. trace_ray queries it in addition to the
    // dynamic and room BVHs, keeping the nearest hit. Going static keeps the
    // high triangle count OUT of the per-frame rebuild.
    bvh::BVH field_bvh;

    // Skybox cubemap (6 faces). Doubles as image-based ambient light for the
    // raytracer: missed rays return the sky color, and the ambient term samples
    // the sky in the surface-normal direction instead of a flat constant.
    std::array<texture, 6> skybox_faces;


    // --- Persistent render thread pool ---
    // Threads are created ONCE (game_init) and live for the whole program.
    // Each frame the main thread posts EACH worker's own start semaphore once,
    // then waits on `done_sem` num_workers times for them to finish.
    // Workers write disjoint horizontal stripes of the framebuffer, so no
    // locking is needed around the pixel writes themselves.
    //
    // NOTE: each worker has its OWN start semaphore instead of a single shared
    // counting one. With a shared start_sem a fast worker that finished its
    // stripe could loop back and steal a token meant for a slower worker that
    // hadn't woken yet, re-render its own stripe, and starve the other worker
    // -> that worker's stripe stayed at the clear color (black stripes), more
    // likely the more workers there are. Per-worker semaphores bind each wake
    // token to exactly one worker, killing the race.
    static constexpr int NUM_WORKERS = 16;
    SDL_Thread*  render_workers[NUM_WORKERS] = {};
    struct thread_data* worker_args = nullptr; // owned array, stable pointers
    SDL_sem*     start_sems[NUM_WORKERS] = {}; // main -> worker i: "render this frame"
    SDL_sem*     done_sem  = nullptr;  // workers -> main: "stripe finished"
    bool         workers_running = true;

    Game(int width, int height) : fb(width, height) {}
};

static vec3 color_from_hash(const std::string& name) {
    uint32_t h = std::hash<std::string>{}(name);
    return vec3(((h>>16)&0xFF)/255.f, ((h>>8)&0xFF)/255.f, (h&0xFF)/255.f);
}

// Forward declarations: these procedural-mesh helpers are defined further down
// (with the rest of the scene generators) but build_scene_tris needs them to
// emit the animated pedestrians each frame.
static void add_humanoid(std::vector<bvh::Tri>& out, const vec3& base,
                         const vec3& skin, const vec3& shirt, float phase);
static void rebuild_field_mesh(Game* e, const std::vector<bvh::Tri>& tris);


void build_scene_tris(Game* e) {
    e->rt_tris.clear();
    for (auto& [name, mp] : e->meshes) {
        const mesh& m = *mp;
        mat4 model = m.modelMatrix();
        vec3 albedo = (name == "FLOOR") ? vec3(0.14f, 0.14f, 0.15f)  // asphalt
                                        : color_from_hash(name);
        for (const triangle& tri : m.faces) {
            vec3 v0 = vec3(model * m.vertices[tri.v0].p);
            vec3 v1 = vec3(model * m.vertices[tri.v1].p);
            vec3 v2 = vec3(model * m.vertices[tri.v2].p);
            vec3 n  = normalize(cross(v1 - v0, v2 - v0));
            e->rt_tris.push_back({v0, v1, v2, n, albedo, (float)e->reflectivity_map[name]});
        }
    }

    // Animated pedestrians (regenerated each frame with the current walk phase).
    const vec3 skin(0.85f, 0.70f, 0.55f);
    for (const auto& p : e->peds) {
        float phase = e->time * 4.0f + p.phase;
        add_humanoid(e->rt_tris, p.pos, skin, p.shirt, phase);
    }

    // Rebuild the BVH over this frame's triangle soup. (The scene is dynamic,
    // so the tree must be rebuilt every frame. For a static scene you'd build
    // it once.) build() copies the triangles in, so rt_tris stays valid for
    // the brute-force comparison path.
    e->bvh.build(e->rt_tris, e->build_strategy);
}


vec3 get_ray_direction(const camera &cam, int pixelX, int pixelY, int width, int height)
{
    float ndcX = (2.0f * (pixelX + 0.5f)) / width  - 1.0f;
    float ndcY = 1.0f - (2.0f * (pixelY + 0.5f)) / height;

    float b = tanf(to_radians(cam._fov) * 0.5f); // tan(fovx/2)
    float a = b / cam._aspectRatio;              // tan(fovy/2)

    vec3 dirCam = vec3(ndcX * b, ndcY * a, -1.0f);

    vec4 dirWorld = cam.rotation() * vec4(dirCam.x, dirCam.y, dirCam.z, 0.0f);
    return normalize(vec3(dirWorld.x, dirWorld.y, dirWorld.z));
}

struct ray
{
    vec3 origin;
    vec3 direction;

    ray(const vec3 &o, const vec3 &d) : origin(o), direction(d) {}
};

//O(1) thingy for ray-triangle intersection, returns whether hit and t value (distance along ray)
bool ray_intersect_triangle(const ray &r, const vec3 &v0, const vec3 &v1, const vec3 &v2, float &t)
{
    // Moller-Trumbore intersection algorithm
    const float EPSILON = 1e-8f;
    vec3 edge1 = v1 - v0;
    vec3 edge2 = v2 - v0;
    vec3 h = cross(r.direction, edge2);
    float a = dot(edge1, h);
    if (fabs(a) < EPSILON)
        return false; // Ray is parallel to triangle

    float f = 1.0f / a;
    vec3 s = r.origin - v0;
    float u = f * dot(s, h);
    if (u < 0.0f || u > 1.0f)
        return false;

    vec3 q = cross(s, edge1);
    float v = f * dot(r.direction, q);
    if (v < 0.0f || u + v > 1.0f)
        return false;

    t = f * dot(edge2, q);
    return t > EPSILON; // Ray intersection
}


const vec3  SUN_DIR = normalize(vec3(-0.3f, 1.0f, -0.2f)); // hacia el "sol", desde arriba
const float AMBIENT = 0.5f;
const float SHADOW_EPS = 1e-4f;

static uint32_t pack(vec3 c) {
    auto ch = [](float v){ return (uint32_t)(std::clamp(v,0.f,1.f)*255.f + 0.5f); };
    return 0xFF000000 | (ch(c.x)<<16) | (ch(c.y)<<8) | ch(c.z);
}

vec3 reflect(vec3 d, vec3 n) {
    return d - n * (2.0f * dot(d, n));   // d normalizado
}

// --- Skybox cubemap sampling -------------------------------------------------
// game_init loads the faces as: 0=rt 1=bk 2=ft 3=lf 4=up 5=dn. sample_sky below
// maps a world direction to the correct face + UV to match the rasterizer.
static vec3 sample_face(const texture& t, float u, float v) {
    if (!t.data || t.width <= 0) return vec3(0.0f, 0.0f, 0.0f);
    int x = (int)(u * t.width);   x = std::clamp(x, 0, t.width  - 1);
    int y = (int)(v * t.height);  y = std::clamp(y, 0, t.height - 1);
    uint32_t c = t.data[y * t.width + x];
    return vec3(((c >> 16) & 0xFF) / 255.f, ((c >> 8) & 0xFF) / 255.f, (c & 0xFF) / 255.f);
}

// Sample the cubemap in world-space direction d. The face selection AND the
// (u,v) per face are derived to match EXACTLY what the rasterizer's skybox does
// (create_skybox_mesh's hand-authored UVs + the game_init load order + how
// sample_texture indexes the data). That way the sky looks identical in both
// renderers. Normalize d by its dominant axis (p has that component = ±1), then
// each face maps the other two components to [0,1] with the raster's signs.
static vec3 sample_sky(const std::array<texture, 6>& f, const vec3& d) {
    float ax = std::fabs(d.x), ay = std::fabs(d.y), az = std::fabs(d.z);
    float m = std::max(ax, std::max(ay, az));
    if (m <= 0.0f) return vec3(0.0f, 0.0f, 0.0f);
    float px = d.x / m, py = d.y / m, pz = d.z / m;

    int idx; float u, v;
    if (ax >= ay && ax >= az) {            // ±X
        if (px > 0) { idx = 2; u = pz * 0.5f + 0.5f;       v = py * 0.5f + 0.5f; } // +X -> ft
        else        { idx = 1; u = 0.5f - pz * 0.5f;       v = py * 0.5f + 0.5f; } // -X -> bk
    } else if (ay >= ax && ay >= az) {     // ±Y
        if (py > 0) { idx = 4; u = px * 0.5f + 0.5f;       v = pz * 0.5f + 0.5f; } // +Y -> up
        else        { idx = 5; u = px * 0.5f + 0.5f;       v = 0.5f - pz * 0.5f; } // -Y -> dn
    } else {                               // ±Z
        if (pz > 0) { idx = 3; u = 0.5f - px * 0.5f;       v = py * 0.5f + 0.5f; } // +Z -> lf
        else        { idx = 0; u = px * 0.5f + 0.5f;       v = py * 0.5f + 0.5f; } // -Z -> rt
    }
    return sample_face(f[idx], u, v);
}

struct SceneHit { float t; const bvh::Tri* tri; };

// Nearest hit against the scene. Switches between the BVH (fast, ~O(log N))
// and the brute-force loop (slow, O(N)) so the two can be compared live.
static bool scene_intersect(const ray& r, const Game& e, SceneHit& out) {
    bool hit = false;
    float best = 1e30f;

    // --- dynamic shapes (rebuilt each frame; [B] toggles BVH vs brute force) ---
    if (e.use_bvh) {
        bvh::Hit h;
        if (e.bvh.intersect(r.origin, r.direction, h)) {
            best = h.t; out.t = h.t; out.tri = &e.bvh.tri(h.tri); hit = true;
        }
    } else {
        const bvh::Tri* bt = nullptr;
        for (const bvh::Tri& tr : e.rt_tris) {
            float t;
            if (ray_intersect_triangle(r, tr.v0, tr.v1, tr.v2, t) && t < best) { best = t; bt = &tr; }
        }
        if (bt) { out.t = best; out.tri = bt; hit = true; }
    }

    // --- static scene (room + field): always via their prebuilt BVHs ---
    auto query_static = [&](const bvh::BVH& b) {
        if (b.empty()) return;
        bvh::Hit h; h.t = best; // clamp the search to the nearest hit so far
        if (b.intersect(r.origin, r.direction, h)) {
            best = h.t; out.t = h.t; out.tri = &b.tri(h.tri); hit = true;
        }
    };
    query_static(e.room_bvh);
    query_static(e.field_bvh);
    return hit;
}

// Any-hit (shadow ray). Same BVH-vs-brute-force switch + the static room.
static bool scene_occluded(const ray& r, const Game& e, float max_t) {
    if (!e.room_bvh.empty() && e.room_bvh.occluded(r.origin, r.direction, max_t))
        return true;
    if (!e.field_bvh.empty() && e.field_bvh.occluded(r.origin, r.direction, max_t))
        return true;
    if (e.use_bvh) return e.bvh.occluded(r.origin, r.direction, max_t);
    for (const bvh::Tri& tr : e.rt_tris) {
        float t;
        if (ray_intersect_triangle(r, tr.v0, tr.v1, tr.v2, t) && t < max_t) return true;
    }
    return false;
}

//Returns the raw color
// Smooth (Phong) shading: interpolate the triangle's per-vertex normals at the
// hit point P via its barycentric coordinates. Falls back to the flat face
// normal for degenerate triangles.
static vec3 tri_smooth_normal(const bvh::Tri& tr, const vec3& P) {
    vec3 e1 = tr.v1 - tr.v0, e2 = tr.v2 - tr.v0, p = P - tr.v0;
    float d11 = dot(e1, e1), d12 = dot(e1, e2), d22 = dot(e2, e2);
    float p1  = dot(p, e1),  p2  = dot(p, e2);
    float det = d11 * d22 - d12 * d12;
    if (std::fabs(det) < 1e-12f) return tr.normal;
    float inv = 1.0f / det;
    float v = (d22 * p1 - d12 * p2) * inv;   // weight of v1
    float w = (d11 * p2 - d12 * p1) * inv;   // weight of v2
    float u = 1.0f - v - w;                   // weight of v0
    return normalize(u * tr.n0 + v * tr.n1 + w * tr.n2);
}

vec3 trace_ray(const ray& r, const Game& e, int depth = 0) {
    SceneHit hit;
    // Miss -> the sky itself (cubemap) is the background.
    if (!scene_intersect(r, e, hit)) return sample_sky(e.skybox_faces, r.direction);

    const bvh::Tri& tr = *hit.tri;
    vec3 P = r.origin + r.direction * hit.t;
    vec3 N = tr.smooth ? tri_smooth_normal(tr, P) : tr.normal;
    if (dot(N, r.direction) > 0.0f) N = -N;

    // --- color local (ambiente del cielo + sol directo con sombra) ---
    ray shadow(P + N * SHADOW_EPS, SUN_DIR);
    bool in_shadow = scene_occluded(shadow, e, 1e30f);
    // Image-based ambient: sample the sky in the normal direction instead of a
    // flat constant, so surfaces pick up the environment's color/brightness.
    vec3 ambient = sample_sky(e.skybox_faces, N) * AMBIENT;
    float diff   = in_shadow ? 0.0f : std::max(0.0f, dot(N, SUN_DIR));
    float sun    = diff * (1.0f - AMBIENT); // white directional light
    // Per-channel light = sky ambient (colored) + sun (white). Modulate by albedo.
    vec3 light(ambient.x + sun, ambient.y + sun, ambient.z + sun);
    vec3 local(tr.albedo.x * light.x, tr.albedo.y * light.y, tr.albedo.z * light.z);

    // --- corte + reflejo ---
    if (depth >= e.max_bounces || tr.reflectivity <= 0.0f || !e.reflections)
        return local;

    vec3 R = reflect(r.direction, N);
    ray  reflected(P + N * SHADOW_EPS, R);
    vec3 refl_color = trace_ray(reflected, e, depth + 1);   // ← recursión

    return local * (1.0f - tr.reflectivity) + refl_color * tr.reflectivity;
}

// GAME APP FUNCTIONS
Game *game_create(int width, int height)
{
    return new Game(width, height);
}

struct thread_data {
    Game* game;
    int thread_id;
    long visits = 0;   // BVH nodes this worker tested this frame (quality metric)
};

int worker_thread(void* data) {
    thread_data* td = (thread_data*)data;
    Game* e = td->game;
    int id = td->thread_id;

    // Persistent worker loop: block on start_sem until the main thread kicks
    // off a frame, render this worker's stripe, then signal done_sem.
    while (true) {
        SDL_SemWait(e->start_sems[id]);     // wait for "render this frame"
        if (!e->workers_running) break;     // shutdown sentinel

        bvh::reset_thread_node_visits();    // count nodes this worker tests

        // Divide the screen into horizontal stripes, one per worker.
        // The last worker absorbs the remainder rows (height % NUM_WORKERS).
        int stripe  = e->fb.height / Game::NUM_WORKERS;
        int y_start = stripe * id;
        int y_end   = (id == Game::NUM_WORKERS - 1) ? e->fb.height : stripe * (id + 1);

        for (int y = y_start; y < y_end; ++y) {
            for (int x = 0; x < e->fb.width; ++x) {
                vec3 rayDir = get_ray_direction(e->cam, x, y, e->fb.width, e->fb.height);
                ray r(e->cam._position, rayDir);
                uint32_t hitColor = pack(trace_ray(r, *e));
                e->fb.colorBuffer[y * e->fb.width + x] = hitColor | 0xFF000000;
            }
        }

        td->visits = bvh::take_thread_node_visits(); // stash for the main thread
        SDL_SemPost(e->done_sem);           // signal "stripe finished"
    }
    return 0;
}


// ===================== Static room (PLY) =====================
// The room scan is Z-up, in its own large coordinate frame, offset from origin.
// We bake a fixed transform into the triangles at load time (it never moves):
//   - subtract the native center so the room sits on the origin in x/z,
//   - drop the floor to engine y = 0,
//   - convert Z-up -> engine Y-up,
//   - scale it down to wrap the little procedural scene.
// All of these are constants you can tweak to fit the shapes differently.
static const float ROOM_SCALE  = 1.0f;
static const vec3  ROOM_CENTER = vec3(8.5485f, -4.81f, -2.519f); // native (cx, cy, floor_z)
static const float ROOM_Y_OFF  = -0.02f; // nudge the room floor just under FLOOR to avoid z-fight
static const vec3  ROOM_OFFSET = vec3(0.0f, -4.81f, 0.0f); // world-space placement knob
// Ceiling cut: the room has a roof that would block the sun and leave the
// interior pitch black. We drop every triangle lying entirely in the top slab,
// i.e. whose 3 vertices all have native z > ROOM_CEIL - ROOM_CUT. This opens
// the top (doll-house cutaway) so the directional light reaches inside.
static const float ROOM_CEIL = 45.314f; // native max z (TECHO)
static const float ROOM_CUT  = 4.0f;    // THRESHOLD, native units (raise to cut more)
static const float ROOM_REFLECT = 0.35f; // how mirror-like the walls/floor are [0..1]

static vec3 room_to_engine(const vec3& p) {
    return vec3((p.x - ROOM_CENTER.x) * ROOM_SCALE,
                (p.z - ROOM_CENTER.z) * ROOM_SCALE + ROOM_Y_OFF, // native z (up) -> engine y
                (p.y - ROOM_CENTER.y) * ROOM_SCALE)
           + ROOM_OFFSET; // world-space placement knob
}

static void load_room(Game* e, const char* path) {
    mesh room = load_ply_ascii(path);
    const vec3 albedo(0.72f, 0.70f, 0.64f); // neutral wall color
    const float cut_z = ROOM_CEIL - ROOM_CUT;

    std::vector<bvh::Tri> tris;
    tris.reserve(room.faces.size());

    // Parallel plain mesh for the rasterizer (engine-space vertices, ceiling
    // already cut). Each kept triangle contributes 3 fresh vertices.
    mesh* rm = new mesh;
    rm->vertices.reserve(room.faces.size() * 3);
    rm->faces.reserve(room.faces.size());

    for (const triangle& f : room.faces) {
        const vec3& a = room.vertices[f.v0].p;
        const vec3& b = room.vertices[f.v1].p;
        const vec3& c = room.vertices[f.v2].p;
        // Skip ceiling polygons (entirely above the cut line, native z).
        if (a.z > cut_z && b.z > cut_z && c.z > cut_z) continue;

        vec3 v0 = room_to_engine(a);
        vec3 v1 = room_to_engine(b);
        vec3 v2 = room_to_engine(c);
        vec3 n  = normalize(cross(v1 - v0, v2 - v0));
        tris.push_back({v0, v1, v2, n, albedo, ROOM_REFLECT});

        uint32_t base = (uint32_t)rm->vertices.size();
        rm->vertices.push_back({v0, {0, 0}});
        rm->vertices.push_back({v1, {1, 0}});
        rm->vertices.push_back({v2, {1, 1}});
        rm->faces.push_back({base, base + 1, base + 2});
    }

    std::cout << "Room: kept " << tris.size() << " / " << room.faces.size()
              << " tris after ceiling cut\n";
    e->room_bvh.build(std::move(tris)); // built ONCE; the room is static
    e->room_mesh = rm;

    std::cout << "Room BVH: " << e->room_bvh.node_count() << " nodes\n";
}

// ===================== Procedural scatter field =====================
// Generates triangles directly (no mesh/transform indirection) for a static
// field that is built once and never animated. Each sphere becomes a
// lat/long UV sphere; its triangle normal is the outward radial direction at
// the triangle centroid, giving smooth-ish shading without per-vertex normals.

static vec3 hsv_to_rgb(float h, float s, float v)
{
    float c  = v * s;
    float hp = h * 6.0f;
    float x  = c * (1.0f - std::fabs(std::fmod(hp, 2.0f) - 1.0f));
    float r = 0, g = 0, b = 0;
    if      (hp < 1) { r = c; g = x; }
    else if (hp < 2) { r = x; g = c; }
    else if (hp < 3) { g = c; b = x; }
    else if (hp < 4) { g = x; b = c; }
    else if (hp < 5) { r = x; b = c; }
    else             { r = c; b = x; }
    float m = v - c;
    return vec3(r + m, g + m, b + m);
}

static void add_sphere(std::vector<bvh::Tri>& out, const vec3& center, float radius,
                       const vec3& albedo, float reflect, int slices, int stacks)
{
    for (int st = 0; st < stacks; ++st) {
        float ph0 = (float) st      / stacks * 3.14159265f;
        float ph1 = (float)(st + 1) / stacks * 3.14159265f;
        for (int sl = 0; sl < slices; ++sl) {
            float th0 = (float) sl      / slices * 6.2831853f;
            float th1 = (float)(sl + 1) / slices * 6.2831853f;
            auto p = [&](float ph, float th) {
                return vec3(std::sin(ph) * std::cos(th),
                            std::cos(ph),
                            std::sin(ph) * std::sin(th));
            };
            vec3 v0 = center + p(ph0, th0) * radius;
            vec3 v1 = center + p(ph1, th0) * radius;
            vec3 v2 = center + p(ph1, th1) * radius;
            vec3 v3 = center + p(ph0, th1) * radius;
            // Smooth shading: per-vertex normals point straight out from the
            // sphere center, so adjacent triangles blend into a round surface.
            vec3 n0n = normalize(p(ph0, th0)), n1n = normalize(p(ph1, th0)),
                 n2n = normalize(p(ph1, th1)), n3n = normalize(p(ph0, th1));
            vec3 fn = normalize(cross(v1 - v0, v2 - v0));
            out.push_back({ v0, v1, v2, fn, albedo, reflect, true, n0n, n1n, n2n });
            out.push_back({ v0, v2, v3, fn, albedo, reflect, true, n0n, n2n, n3n });
        }
    }
}

// Density presets shared by both procedural scenes.
static int scene_count(int density) { static const int c[3] = { 64, 256, 1024 }; return c[density]; }
static int city_grid(int density)   { static const int g[3] = { 10, 16, 24 };    return g[density]; }

static void build_field(Game* e)
{
    const int count = scene_count(e->density);
    std::vector<bvh::Tri> tris;
    tris.reserve((size_t)count * FIELD_SLICES * FIELD_STACKS * 2);

    // Deterministic LCG so every run lays out the same field (reproducible
    // benchmarks; toggle randomness via the seed if desired).
    uint32_t rng = 0xC0FFEEu;
    auto urand = [&]() { rng = rng * 1664525u + 1013904223u; return (rng >> 8) / 16777216.0f; };

    const int   side  = (int)std::round(std::sqrt((float)count));
    const float span  = 14.0f;          // fits inside the 20x20 floor
    const float cell  = span / side;
    int idx = 0;
    for (int z = 0; z < side && idx < count; ++z) {
        for (int x = 0; x < side && idx < count; ++x, ++idx) {
            float jx = (urand() - 0.5f) * cell * 0.6f;
            float jz = (urand() - 0.5f) * cell * 0.6f;
            float px = -span * 0.5f + cell * (x + 0.5f) + jx;
            float pz = -span * 0.5f + cell * (z + 0.5f) + jz;
            float r  = cell * (0.20f + urand() * 0.18f);
            vec3 albedo = hsv_to_rgb(urand(), 0.7f, 0.92f);
            float reflect = 0.05f + urand() * 0.45f;
            add_sphere(tris, vec3(px, r, pz), r, albedo, reflect, FIELD_SLICES, FIELD_STACKS);
        }
    }

    rebuild_field_mesh(e, tris);
    e->field_bvh.build(std::move(tris), e->build_strategy);
    std::cout << "Field: " << count << " spheres, "
              << e->field_bvh.triangle_count() << " tris, "
              << e->field_bvh.node_count() << " BVH nodes\n";
}

// ===================== Procedural city =====================
// A grid of axis-aligned buildings (12 triangles each) with random heights
// (taller toward the center = a downtown), streets between them, and a palette
// of concrete + a few glass (reflective) towers. Boxes are ideal for a BVH:
// each building is itself an AABB, so the tree culls whole blocks cheaply.

static void add_box(std::vector<bvh::Tri>& out,
                    const vec3& lo, const vec3& hi,
                    const vec3& albedo, float reflect)
{
    vec3 v[8] = {
        {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, lo.y, hi.z}, {lo.x, lo.y, hi.z},
        {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}
    };
    auto face = [&](int a, int b, int c, int d, const vec3& n) {
        out.push_back({ v[a], v[b], v[c], n, albedo, reflect });
        out.push_back({ v[a], v[c], v[d], n, albedo, reflect });
    };
    face(4, 5, 6, 7, { 0,  1, 0}); // top  (+y roof)
    face(3, 2, 1, 0, { 0, -1, 0}); // bottom (-y)
    face(0, 3, 7, 4, {-1,  0, 0}); // -x
    face(1, 5, 6, 2, { 1,  0, 0}); // +x
    face(0, 1, 5, 4, { 0,  0, -1}); // -z
    face(3, 7, 6, 2, { 0,  0, 1}); // +z
}

// add_box but specified by center + half-extents (handy for composing figures).
static void add_box_c(std::vector<bvh::Tri>& out, const vec3& center, const vec3& half,
                      const vec3& albedo, float reflect) {
    add_box(out, center - half, center + half, albedo, reflect);
}

// A boxy pedestrian (legs, torso, arms, head) with a simple walk cycle: legs
// and arms swing in opposition and the whole figure bobs. Recognizable as a
// person at street-view distance; cheap (6 boxes = 72 triangles).
static void add_humanoid(std::vector<bvh::Tri>& out, const vec3& base,
                         const vec3& skin, const vec3& shirt, float phase)
{
    float s   = std::sin(phase);
    float bob = std::fabs(std::sin(phase)) * 0.06f;   // step bob
    float y   = base.y + bob;
    float leg = s * 0.16f;                              // leg/arm swing (along z)
    const vec3 pants(0.12f, 0.12f, 0.18f);

    add_box_c(out, vec3(base.x - 0.07f, y + 0.35f, base.z + leg),  vec3(0.06f, 0.35f, 0.06f), pants, 0.02f);
    add_box_c(out, vec3(base.x + 0.07f, y + 0.35f, base.z - leg),  vec3(0.06f, 0.35f, 0.06f), pants, 0.02f);
    add_box_c(out, vec3(base.x,         y + 0.85f, base.z),        vec3(0.16f, 0.28f, 0.10f), shirt, 0.03f);
    add_box_c(out, vec3(base.x - 0.22f, y + 0.85f, base.z - leg),  vec3(0.05f, 0.24f, 0.05f), shirt, 0.03f);
    add_box_c(out, vec3(base.x + 0.22f, y + 0.85f, base.z + leg),  vec3(0.05f, 0.24f, 0.05f), shirt, 0.03f);
    add_box_c(out, vec3(base.x,         y + 1.27f, base.z),        vec3(0.09f, 0.09f, 0.09f), skin,  0.02f);
}

// A building = dark concrete shell + a grid of window panes on each facade.
// Some panes are "lit" (warm), the rest are dark reflective glass. The window
// grid multiplies the triangle count — which is exactly what makes the BVH
// earn its keep — and reads as a real skyline.
static void add_windowed_building(std::vector<bvh::Tri>& out,
                                  const vec3& lo, const vec3& hi, uint32_t& rng)
{
    auto urand = [&]() { rng = rng * 1664525u + 1013904223u; return (rng >> 8) / 16777216.0f; };

    add_box(out, lo, hi, vec3(0.10f, 0.10f, 0.12f), 0.02f); // dark mass

    const float spacing = 1.2f;   // target window cell size
    const float inset   = 0.14f;  // frame margin inside a cell
    const float off     = 0.02f;  // push panes out from the wall
    const vec3  up(0.0f, 1.0f, 0.0f);

    auto wall = [&](const vec3& o, const vec3& uax, const vec3& n, float len, float hgt) {
        int cols = (int)std::clamp(len / spacing, 1.0f, 10.0f);
        int rows = (int)std::clamp(hgt / spacing, 1.0f, 16.0f);
        float cu = len / cols, cv = hgt / rows;
        for (int j = 0; j < rows; ++j)
            for (int i = 0; i < cols; ++i) {
                float u0 = i * cu + inset, u1 = (i + 1) * cu - inset;
                float v0 = j * cv + inset, v1 = (j + 1) * cv - inset;
                vec3 p0 = o + uax * u0 + up * v0 + n * off;
                vec3 p1 = o + uax * u1 + up * v0 + n * off;
                vec3 p2 = o + uax * u1 + up * v1 + n * off;
                vec3 p3 = o + uax * u0 + up * v1 + n * off;
                bool  lit = urand() < 0.34f;
                vec3  alb = lit ? vec3(0.96f, 0.85f, 0.52f) : vec3(0.15f, 0.23f, 0.33f);
                float rfl = lit ? 0.0f : 0.30f;
                out.push_back({ p0, p1, p2, n, alb, rfl });
                out.push_back({ p0, p2, p3, n, alb, rfl });
            }
    };

    float H = hi.y - lo.y;
    wall(vec3(lo.x, lo.y, hi.z), vec3( 1.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f,  1.0f), hi.x - lo.x, H); // +z
    wall(vec3(lo.x, lo.y, lo.z), vec3( 1.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, -1.0f), hi.x - lo.x, H); // -z
    wall(vec3(hi.x, lo.y, lo.z), vec3( 0.0f, 0.0f, 1.0f), vec3( 1.0f, 0.0f, 0.0f), hi.z - lo.z, H); // +x
    wall(vec3(lo.x, lo.y, lo.z), vec3( 0.0f, 0.0f, 1.0f), vec3(-1.0f, 0.0f, 0.0f), hi.z - lo.z, H); // -x
}

// Load a Wavefront .obj as smooth-shaded triangles. Parses `v` (positions) and
// `f` (faces, fan-triangulated; vt/vn indices after slashes are ignored). Smooth
// normals are computed by accumulating each face normal into its vertices, so
// any OBJ renders round regardless of whether it shipped normals.
static void load_obj(const char* path, const vec3& base, float scale,
                     const vec3& albedo, float reflect,
                     std::vector<bvh::Tri>& out)
{
    std::ifstream f(path);
    if (!f) { std::cout << "load_obj: could not open " << path << "\n"; return; }

    std::vector<vec3> verts;
    std::vector<std::vector<int>> faces;
    std::string line;
    while (std::getline(f, line)) {
        if (line.size() < 2) continue;
        if (line[0] == 'v' && line[1] == ' ') {
            std::istringstream ss(line.c_str() + 2);
            float x, y, z; ss >> x >> y >> z;
            verts.push_back(vec3(x, y, z));
        } else if (line[0] == 'f' && (line[1] == ' ' || line[1] == '\t')) {
            std::istringstream ss(line.c_str() + 2);
            std::vector<int> idx;
            std::string tok;
            while (ss >> tok) {
                int vi = 0;
                bool ok = !tok.empty() && (tok[0] == '-' || (tok[0] >= '0' && tok[0] <= '9'));
                if (ok) {
                    try { vi = std::stoi(tok); }       // stops at first '/'
                    catch (...) { ok = false; }
                }
                if (!ok) continue;
                if (vi < 0) vi = (int)verts.size() + vi + 1;   // negative = relative
                idx.push_back(vi - 1);                          // -> 0-based
            }
            if (idx.size() >= 3) faces.push_back(idx);
        }
    }

    auto valid = [&](int i) { return i >= 0 && i < (int)verts.size(); };

    // Per-vertex smooth normals.
    std::vector<vec3> vn(verts.size(), vec3(0.0f, 0.0f, 0.0f));
    for (auto& fc : faces)
        for (size_t i = 1; i + 1 < fc.size(); ++i) {
            int a = fc[0], b = fc[i], c = fc[i + 1];
            if (!valid(a) || !valid(b) || !valid(c)) continue;
            vec3 n = normalize(cross(verts[b] - verts[a], verts[c] - verts[a]));
            vn[a] = vn[a] + n; vn[b] = vn[b] + n; vn[c] = vn[c] + n;
        }
    for (auto& n : vn) n = normalize(n);

    for (auto& fc : faces)
        for (size_t i = 1; i + 1 < fc.size(); ++i) {
            int a = fc[0], b = fc[i], c = fc[i + 1];
            if (!valid(a) || !valid(b) || !valid(c)) continue;
            vec3 v0 = base + verts[a] * scale;
            vec3 v1 = base + verts[b] * scale;
            vec3 v2 = base + verts[c] * scale;
            vec3 fn = normalize(cross(v1 - v0, v2 - v0));
            out.push_back({ v0, v1, v2, fn, albedo, reflect, true, vn[a], vn[b], vn[c] });
        }

    std::cout << "load_obj: " << path << " -> " << verts.size() << " verts, "
              << faces.size() << " faces\n";
}

static void build_city(Game* e)
{
    const int   grid   = city_grid(e->density);
    const float span   = CITY_SPAN;          // city footprint
    const float cell   = span / grid;        // one block
    const float street = cell * 0.15f;       // gap between buildings = streets
    const float avenue = cell * 0.8f;        // half-width of the carved main avenues

    uint32_t rng = 0x1234u;
    auto urand = [&]() { rng = rng * 1664525u + 1013904223u; return (rng >> 8) / 16777216.0f; };

    std::vector<bvh::Tri> tris;
    int built = 0;
    for (int z = 0; z < grid; ++z) {
        for (int x = 0; x < grid; ++x) {
            if (urand() < 0.12f) continue;                    // empty lot / plaza
            float cx = -span * 0.5f + cell * (x + 0.5f);
            float cz = -span * 0.5f + cell * (z + 0.5f);
            if (std::fabs(cx) < avenue || std::fabs(cz) < avenue)
                continue;                                      // carve a cross of avenues
            float half = cell * 0.5f - street;
            float d = std::sqrt(cx * cx + cz * cz) / (span * 0.5f); // 0 center .. 1 edge
            float h = cell * (1.0f + (1.0f - std::clamp(d, 0.0f, 1.0f)) * 7.0f
                                 + urand() * 4.0f);
            add_windowed_building(tris, vec3(cx - half, 0.0f, cz - half),
                                         vec3(cx + half, h,      cz + half), rng);
            ++built;
        }
    }

    // --- street props (static) ---
    // Dashed center lines on both avenues (the cross carved above).
    const float dash = 0.6f, gap = 0.6f, lw = 0.06f;
    const vec3  laneCol(0.90f, 0.80f, 0.20f);
    for (float p = -span * 0.5f; p < span * 0.5f; p += dash + gap) {
        add_box_c(tris, vec3(0.0f, 0.012f, p + dash * 0.5f), vec3(lw, 0.006f, dash * 0.5f), laneCol, 0.0f);          // N-S avenue
        add_box_c(tris, vec3(p + dash * 0.5f, 0.012f, 0.0f), vec3(dash * 0.5f, 0.006f, lw), laneCol, 0.0f);          // E-W avenue
    }

    // Traffic light on a corner of the intersection (pole + arm + 3 lights,
    // green ON and bright so the ray tracer shows it glowing).
    {
        vec3 pole(avenue - 0.4f, 0.0f, avenue - 0.4f);   // a corner of the intersection
        add_box_c(tris, vec3(pole.x, 1.5f, pole.z),    vec3(0.05f, 1.5f, 0.05f), vec3(0.14f, 0.14f, 0.14f), 0.02f);
        add_box_c(tris, vec3(pole.x - 0.35f, 3.0f, pole.z), vec3(0.35f, 0.05f, 0.05f), vec3(0.14f, 0.14f, 0.14f), 0.02f);
        vec3 hl(pole.x - 0.6f, 2.78f, pole.z);
        add_box_c(tris, hl, vec3(0.08f, 0.30f, 0.08f), vec3(0.09f, 0.09f, 0.09f), 0.02f);
        add_box_c(tris, hl + vec3(0, 0.19f, 0), vec3(0.05f, 0.05f, 0.04f), vec3(0.10f, 0.04f, 0.04f), 0.0f); // red  (off)
        add_box_c(tris, hl + vec3(0, 0.00f, 0), vec3(0.05f, 0.05f, 0.04f), vec3(0.10f, 0.09f, 0.03f), 0.0f); // yellow (off)
        add_box_c(tris, hl + vec3(0,-0.19f, 0), vec3(0.07f, 0.07f, 0.05f), vec3(0.15f, 1.00f, 0.30f), 0.0f); // green (ON)
    }

    // A few parked cars along the N-S avenue (body + glass cabin + 4 wheels).
    auto add_car = [&](const vec3& c, const vec3& col) {
        add_box_c(tris, c + vec3(0, 0.25f, 0),      vec3(0.34f, 0.17f, 0.70f), col,                         0.25f);
        add_box_c(tris, c + vec3(0, 0.50f, 0.03f),  vec3(0.30f, 0.12f, 0.34f), vec3(0.14f, 0.20f, 0.26f),  0.10f);
        const float wx = 0.33f, wz = 0.46f;
        add_box_c(tris, c + vec3( wx, 0.08f,  wz), vec3(0.08f, 0.08f, 0.07f), vec3(0.04f, 0.04f, 0.04f), 0.0f);
        add_box_c(tris, c + vec3(-wx, 0.08f,  wz), vec3(0.08f, 0.08f, 0.07f), vec3(0.04f, 0.04f, 0.04f), 0.0f);
        add_box_c(tris, c + vec3( wx, 0.08f, -wz), vec3(0.08f, 0.08f, 0.07f), vec3(0.04f, 0.04f, 0.04f), 0.0f);
        add_box_c(tris, c + vec3(-wx, 0.08f, -wz), vec3(0.08f, 0.08f, 0.07f), vec3(0.04f, 0.04f, 0.04f), 0.0f);
    };
    const int   nCars = (int)(span / 5.0f);
    for (int i = 0; i < nCars; ++i) {
        float zc = -span * 0.5f + 4.0f + i * ((span - 8.0f) / std::max(1, nCars));
        add_car(vec3(avenue * 0.4f, 0.0f, zc), hsv_to_rgb(urand(), 0.6f, 0.7f));
    }

    // Centerpiece: a smooth (OBJ) sculpture on a pedestal at the intersection.
    add_box_c(tris, vec3(0.0f, 0.5f, 0.0f), vec3(0.22f, 0.5f, 0.22f), vec3(0.13f, 0.13f, 0.14f), 0.02f); // pedestal
    load_obj("res/models/sculpture.obj", vec3(0.0f, 1.15f, 0.0f), 0.8f,
             vec3(0.96f, 0.78f, 0.35f), 0.45f, tris);                              // gold reflective torus

    rebuild_field_mesh(e, tris);
    e->field_bvh.build(std::move(tris), e->build_strategy);
    std::cout << "City: " << grid << "x" << grid << " grid, " << built
              << " buildings, " << e->field_bvh.triangle_count() << " tris, "
              << e->field_bvh.node_count() << " BVH nodes\n";
}

// Rebuild the static field according to the current scene + density settings
// (called from game_init and from the options menu when those change).
static void rebuild_field(Game* e)
{
    uint64_t t0 = SDL_GetPerformanceCounter();
    if (e->scene_id == 0) build_city(e);
    else                  build_field(e);
    e->field_build_ms = (SDL_GetPerformanceCounter() - t0) * 1000.0 / SDL_GetPerformanceFrequency();
    std::cout << "  -> build " << (e->scene_id == 0 ? "city" : "field")
              << " (" << (e->build_strategy == bvh::SAH ? "SAH"
                       : e->build_strategy == bvh::Median ? "Median" : "Morton")
              << "): " << e->field_build_ms << " ms\n";
}

// Build (or rebuild) the rasterizer-side mirror of the field: one mesh whose
// triangles are the field's, so render_mesh can draw the city in raster mode.
// Flat-shaded there (the software rasterizer is per-mesh color), but it shows
// the full geometry instead of an empty street.
static void rebuild_field_mesh(Game* e, const std::vector<bvh::Tri>& tris)
{
    delete e->field_mesh;
    e->field_mesh = new mesh;
    e->field_mesh->vertices.reserve(tris.size() * 3);
    e->field_mesh->faces.reserve(tris.size());
    for (const auto& t : tris) {
        uint32_t b = (uint32_t)e->field_mesh->vertices.size();
        e->field_mesh->vertices.push_back({ t.v0, {0.0f, 0.0f} });
        e->field_mesh->vertices.push_back({ t.v1, {0.0f, 0.0f} });
        e->field_mesh->vertices.push_back({ t.v2, {0.0f, 0.0f} });
        e->field_mesh->faces.push_back({ b, b + 1, b + 2 });
    }
    e->field_mesh->_modelMatrixDirty = true; // verts are already world space
}

void game_init(Game *e)
{
    SDL_SetRelativeMouseMode(SDL_TRUE);

    e->cam = camera(e->position, vec3(e->pitch, e->yaw, 0.0f), 90.0f,
                    float(e->fb.width) / float(e->fb.height), 0.01f, 1000.0f);

    // Skybox cubemap. Order matches the face indices used by sample_sky():
    // 0=+X(rt) 1=-Z(bk) 2=+Z(ft) 3=-X(lf) 4=+Y(up) 5=-Y(dn).
    load_png_texture("res/textures/skybox3/null_plainsky512_rt.png", e->skybox_faces[0]);
    load_png_texture("res/textures/skybox3/null_plainsky512_bk.png", e->skybox_faces[1]);
    load_png_texture("res/textures/skybox3/null_plainsky512_ft.png", e->skybox_faces[2]);
    load_png_texture("res/textures/skybox3/null_plainsky512_lf.png", e->skybox_faces[3]);
    load_png_texture("res/textures/skybox3/null_plainsky512_up.png", e->skybox_faces[4]);
    load_png_texture("res/textures/skybox3/null_plainsky512_dn.png", e->skybox_faces[5]);

    // Floor
    mesh* floor_mesh = new mesh;
    create_plane(floor_mesh, CITY_SPAN + 10.0f, CITY_SPAN + 10.0f);
    floor_mesh->setPosition(vec3(0.0f, 0.0f, 0.0f));
    e->meshes["FLOOR"] = floor_mesh;
    e->reflectivity_map["FLOOR"] = 0.2f; // Example reflectivity for the floor


    // ===================== Objetos animados ======================
    // Esfera que orbita (se mueve en game_update).
    mesh* orbiter = new mesh;
    create_sphere(orbiter, 0.4f, 8, 12);
    e->meshes["ORBITER"] = orbiter;
    e->reflectivity_map["ORBITER"] = 0.6f;

    // Cubo que "late" (cambia de tamaño en game_update).
    mesh* pulse = new mesh;
    create_cube(pulse, 1.0f);
    e->meshes["PULSER"] = pulse;
    e->reflectivity_map["PULSER"] = 0.1f;

    // Cilindro que rebota verticalmente.
    mesh* bouncer = new mesh;
    create_cylinder(bouncer, 0.35f, 0.8f, 16);
    e->meshes["BOUNCER"] = bouncer;
    e->reflectivity_map["BOUNCER"] = 0.2f;

    // Pedestrians strolling along the N-S avenue sidewalks.
    {
        const float cell = CITY_SPAN / city_grid(e->density);
        const float av   = cell * 0.8f;            // matches the carved avenue width
        uint32_t r = 0xABCDEFu;
        auto ur = [&]() { r = r * 1664525u + 1013904223u; return (r >> 8) / 16777216.0f; };
        const int N = 18;
        for (int i = 0; i < N; ++i) {
            float z = -CITY_SPAN * 0.45f + (i + 0.5f) * (CITY_SPAN * 0.9f / N);
            float side = (i & 1) ? (av - 0.4f) : -(av - 0.4f);
            Game::Ped p;
            p.pos   = vec3(side, 0.0f, z);
            p.shirt = hsv_to_rgb(ur(), 0.65f, 0.8f);
            p.phase = ur() * 6.283f;
            e->peds.push_back(p);
        }
    }

    // Static room scan wraps the procedural scene (its own BVH, built once).
#if ENABLE_ROOM
    load_room(e, "res/textures/iscv2.ply");
#endif

#if ENABLE_FIELD
    rebuild_field(e);   // static high-triangle scene (city or spheres), built once
#endif

    build_scene_tris(e);

    // --- Spin up the persistent render thread pool (once) ---
    e->done_sem  = SDL_CreateSemaphore(0);
    e->worker_args = new thread_data[Game::NUM_WORKERS];
    for (int i = 0; i < Game::NUM_WORKERS; ++i) {
        e->start_sems[i] = SDL_CreateSemaphore(0);
        e->worker_args[i] = thread_data{e, i};
        e->render_workers[i] = SDL_CreateThread(worker_thread, "RenderWorker", &e->worker_args[i]);
    }
}

void game_update(Game *e, float dt)
{
    if (e->show_menu) return; // paused while the options menu is open

    const Uint8 *keys = SDL_GetKeyboardState(NULL);

    int mdx = 0, mdy = 0;
    SDL_GetRelativeMouseState(&mdx, &mdy);

    e->yaw   += float(mdx) * kMouseSensitivity;
    e->pitch += -float(mdy) * kMouseSensitivity;
    e->pitch  = std::clamp(e->pitch, to_radians(-85.0f), to_radians(85.0f));

    vec3 inputDir(0.0f, 0.0f, 0.0f);
    if (keys[SDL_SCANCODE_W])      inputDir.z -= 1.0f;
    if (keys[SDL_SCANCODE_S])      inputDir.z += 1.0f;
    if (keys[SDL_SCANCODE_A])      inputDir.x -= 1.0f;
    if (keys[SDL_SCANCODE_D])      inputDir.x += 1.0f;
    // Vertical (climb/descend) only in fly mode; on the ground SPACE/SHIFT do nothing.
    if (e->fly_mode) {
        if (keys[SDL_SCANCODE_LSHIFT]) inputDir.y -= 1.0f;
        if (keys[SDL_SCANCODE_SPACE])  inputDir.y += 1.0f;
    }

    float s = sinf(e->yaw);
    float c = cosf(e->yaw);

    vec3 moveDir;
    moveDir.x = (inputDir.x * c - inputDir.z * s) * moveSpeed;
    moveDir.z = (inputDir.x * s + inputDir.z * c) * moveSpeed;
    moveDir.y =  inputDir.y * moveSpeed;

    e->position = e->position + moveDir * dt;

    // While walking (not flying) the camera stays at eye height on the ground.
    if (!e->fly_mode) e->position.y = 1.7f;

    // First-person head-bob: advance a phase with horizontal speed, then sway
    // the camera up/down a touch so walking reads as footsteps (disabled in fly).
    float horizSpeed = std::sqrt(moveDir.x * moveDir.x + moveDir.z * moveDir.z);
    if (!e->fly_mode && horizSpeed > 0.001f) e->bob_phase += dt * horizSpeed * 2.0f;
    float bobAmt = e->fly_mode ? 0.0f : std::min(1.0f, horizSpeed / moveSpeed);
    float bob = std::sin(e->bob_phase) * 0.05f * bobAmt;

    e->cam.setPosition(e->position + vec3(0.0f, bob, 0.0f));
    e->cam.setRotation(vec3(e->pitch, e->yaw, 0.0f));

    // ===================== Animación de la escena =====================
    e->time += dt;
    float t = e->time;

    // Esfera orbitando alrededor del origen en el plano XZ.
    float orbit_r = 2.5f;
    e->meshes["ORBITER"]->setPosition(vec3(
        std::cos(t) * orbit_r,
        0.6f + 0.3f * std::sin(t * 2.0f),  // pequeño sube-baja
        std::sin(t) * orbit_r));

    // Cubo que late: escala oscilando entre ~0.5 y ~1.5.
    float pulse = 1.0f + 0.5f * std::sin(t * 3.0f);
    e->meshes["PULSER"]->setScale(vec3(pulse, pulse, pulse));
    e->meshes["PULSER"]->setPosition(vec3(-1.5f, 2.5f, -2.0f));
    e->meshes["PULSER"]->updateRotation(vec3(0.0f, dt * 1.5f, 0.0f)); // gira en Y

    // Cilindro que rebota verticalmente (valor absoluto del seno = rebote).
    float bounce = 0.4f + std::fabs(std::sin(t * 2.5f)) * 1.8f;
    e->meshes["BOUNCER"]->setPosition(vec3(2.5f, bounce, 2.5f));
}



// Planar projected shadows for the rasterizer path. Flattens every caster
// mesh onto the ground plane (y = plane_y) along the light direction and draws
// it dark. Directional light => the projection is a plain affine matrix:
//     Q = P - L * (P.y - plane_y) / L.y
static void render_raster_shadows(Game *e)
{
    const vec3  L       = SUN_DIR;     // points toward the sun (L.y > 0)
    const float plane_y = 0.02f;       // just above the floor to avoid z-fight

    // Build the flatten-onto-plane matrix (acts on world-space points).
    mat4 S(1.0f);
    S(0,1) = -L.x / L.y;  S(1,1) = 0.0f;  S(2,1) = -L.z / L.y;
    S(0,3) = (L.x / L.y) * plane_y;
    S(1,3) = plane_y;
    S(2,3) = (L.z / L.y) * plane_y;

    renderConfig scfg;
    scfg.baseColor   = 0xFF1A1A1A; // dark grey shadow
    scfg.ignoreLight = true;

    mesh tmp; // reused: identity transform, vertices already in world space
    for (auto &[name, mp] : e->meshes)
    {
        if (name == "FLOOR") continue; // the receiver doesn't cast on itself

        const mesh &m = *mp;
        mat4 MS = S * m.modelMatrix(); // model -> world -> flattened

        tmp.vertices.clear();
        tmp.vertices.reserve(m.vertices.size());
        for (const vertex &v : m.vertices) {
            vec4 w = MS * v.p;
            tmp.vertices.push_back({ vec3(w.x, w.y, w.z), v.t });
        }
        tmp.faces = m.faces;
        tmp._modelMatrixDirty = true; // recompute -> identity (verts are world)

        // Draw both windings so backface culling can't punch holes in the
        // flattened (mixed-winding) shadow.
        tmp.inverseFaces = false; render_mesh(e->fb, e->cam, tmp, scfg);
        tmp.inverseFaces = true;  render_mesh(e->fb, e->cam, tmp, scfg);
    }
}

// Draw the 12 edges of an AABB as 3D gizmo lines.
static void draw_aabb_wire(framebuffer &fb, const camera &cam,
                           const AABB &b, uint32_t color)
{
    vec3 c[8] = {
        {b.min.x, b.min.y, b.min.z}, {b.max.x, b.min.y, b.min.z},
        {b.max.x, b.min.y, b.max.z}, {b.min.x, b.min.y, b.max.z},
        {b.min.x, b.max.y, b.min.z}, {b.max.x, b.max.y, b.min.z},
        {b.max.x, b.max.y, b.max.z}, {b.min.x, b.max.y, b.max.z},
    };
    static const int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0}, // bottom
        {4,5},{5,6},{6,7},{7,4}, // top
        {0,4},{1,5},{2,6},{3,7}, // verticals
    };
    for (auto &ed : edges)
        draw_gizmo_line(fb, cam, c[ed[0]], c[ed[1]], color);
}

// Visualize the BVH: draw every node's box, colored by tree depth, so you can
// literally see how the hierarchy carves space into nested bounding volumes.
static void draw_bvh_debug(Game *e)
{
    static std::vector<bvh::BVH::DebugNode> nodes;

    // Depth -> color palette (cycles), so each level reads distinctly.
    static const uint32_t palette[] = {
        0xFFFF4040, 0xFFFFA040, 0xFFFFFF40, 0xFF40FF40,
        0xFF40FFFF, 0xFF4080FF, 0xFFC040FF, 0xFFFF40C0,
    };
    auto draw_bvh = [&](const bvh::BVH& b) {
        b.debug_nodes(nodes);
        for (const auto &n : nodes) {
            uint32_t col = palette[n.depth % (sizeof(palette) / sizeof(palette[0]))];
            draw_aabb_wire(e->fb, e->cam, n.bounds, col);
        }
    };
    draw_bvh(e->field_bvh); // the big static scene (city/spheres)
    draw_bvh(e->room_bvh);
    draw_bvh(e->bvh);       // small dynamic scene
}

// Tiny timer helper: returns elapsed milliseconds since `since` and resets it.
static double tick_ms(uint64_t& since) {
    uint64_t now = SDL_GetPerformanceCounter();
    double ms = (now - since) * 1000.0 / SDL_GetPerformanceFrequency();
    since = now;
    return ms;
}

// ===================== In-app options menu =====================
// A translucent on-screen panel ([M]) that exposes the render settings live,
// navigated with Up/Down and changed with Left/Right (or Enter).

static const int MENU_ITEMS = 8;

static const char* menu_label(int i) {
    static const char* L[MENU_ITEMS] = {
        "Render mode", "Acceleration", "Show BVH", "Scene",
        "Density", "Reflections", "Ray bounces", "BVH build"
    };
    return L[i];
}

static const char* menu_value(const Game* e, int i) {
    switch (i) {
        case 0: return e->raytrace_mode ? "Raytrace" : "Raster";
        case 1: return e->use_bvh       ? "BVH"      : "Brute force";
        case 2: return e->show_bvh      ? "On"       : "Off";
        case 3: return e->scene_id == 0 ? "City"     : "Spheres";
        case 4: return e->density == 0  ? "Small"    : (e->density == 1 ? "Medium" : "Large");
        case 5: return e->reflections   ? "On"       : "Off";
        case 6: { static char b[8]; snprintf(b, sizeof(b), "%d", e->max_bounces); return b; }
        default:
            return e->build_strategy == bvh::SAH    ? "SAH"
                 : e->build_strategy == bvh::Median ? "Median" : "Morton";
    }
}

// Apply a value change to the selected row (dir = -1 left / +1 right).
static void menu_apply(Game* e, int dir) {
    switch (e->menu_cursor) {
        case 0: e->raytrace_mode = !e->raytrace_mode; break;
        case 1: e->use_bvh       = !e->use_bvh;       break;
        case 2: e->show_bvh      = !e->show_bvh;      break;
        case 3: e->scene_id ^= 1; rebuild_field(e);   break;
        case 4: e->density = (e->density + (dir < 0 ? 2 : 1)) % 3; rebuild_field(e); break;
        case 5: e->reflections   = !e->reflections;   break;
        case 6: e->max_bounces = 1 + ((e->max_bounces - 1 + (dir < 0 ? 2 : 1)) % 3); break;
        case 7: { // cycle SAH -> Median -> Morton -> SAH, then rebuild
            int s = (int)e->build_strategy;
            s = (s + (dir < 0 ? 2 : 1)) % 3;
            e->build_strategy = (bvh::BuildStrategy)s;
            rebuild_field(e);
            break;
        }
    }
}

// Alpha-blended filled rectangle (a panel over the scene, not a flat box).
static void fill_rect_alpha(framebuffer& fb, int x0, int y0, int x1, int y1, uint32_t col) {
    float a = (float)((col >> 24) & 0xFF) / 255.0f;
    int cr = (col >> 16) & 0xFF, cg = (col >> 8) & 0xFF, cb = col & 0xFF;
    for (int y = std::max(0, y0); y < std::min(fb.height, y1); ++y)
        for (int x = std::max(0, x0); x < std::min(fb.width, x1); ++x) {
            uint32_t d = fb.colorBuffer[y * fb.width + x];
            int dr = (d >> 16) & 0xFF, dg = (d >> 8) & 0xFF, db = d & 0xFF;
            int r = (int)(cr * a + dr * (1.0f - a));
            int g = (int)(cg * a + dg * (1.0f - a));
            int b = (int)(cb * a + db * (1.0f - a));
            fb.colorBuffer[y * fb.width + x] = 0xFF000000u | (r << 16) | (g << 8) | b;
        }
}

static void draw_menu(Game* e) {
    const int lh = 18, pad = 14;
    const int w = 312;
    const int h = pad + 24 + pad + lh * MENU_ITEMS + pad + 18;
    const int x  = e->fb.width - w - 12;
    const int y0 = 18;

    fill_rect_alpha(e->fb, x, y0, x + w, y0 + h, 0xE80B0E18);   // panel
    fill_rect_alpha(e->fb, x, y0, x + w, y0 + 24, 0xFF1B6FB5);  // title bar
    draw_text(e->fb, x + 10, y0 + 7, "OPTIONS   [M] close", 0xFFFFFFFF);

    for (int i = 0; i < MENU_ITEMS; ++i) {
        int ry = y0 + 24 + pad + i * lh;
        if (i == e->menu_cursor)
            fill_rect_alpha(e->fb, x + 6, ry - 2, x + w - 6, ry + lh - 3, 0x603A66B0);
        char line[96];
        snprintf(line, sizeof(line), "%-13s %s", menu_label(i), menu_value(e, i));
        draw_text(e->fb, x + 14, ry, line, i == e->menu_cursor ? 0xFFFFE27A : 0xFFE8E8E8);
    }
    draw_text(e->fb, x + 10, y0 + h - 16, "^/v select   </>  change", 0xFFA8B0BC);
}

// ===================== Benchmark (experiment harness) =====================
// Sweeps every build strategy x density, renders a fixed number of frames for
// each, and records build time, node count, traversal nodes/ray and render ms.
// Prints a table to stdout and writes docs/benchmark.csv for the report. Bind to
// a key (F1) -> the data for the comparison tables/graphs.

struct FrameMeas { double render_ms; long nodes; };

static FrameMeas bench_frame(Game* e) {
    build_scene_tris(e);
    e->cam.rotation();
    uint64_t t0 = SDL_GetPerformanceCounter();
    for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_SemPost(e->start_sems[i]);
    for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_SemWait(e->done_sem);
    long nodes = 0;
    for (int i = 0; i < Game::NUM_WORKERS; ++i) nodes += e->worker_args[i].visits;
    double ms = (SDL_GetPerformanceCounter() - t0) * 1000.0 / SDL_GetPerformanceFrequency();
    return { ms, nodes };
}

void run_benchmark(Game* e) {
    e->raytrace_mode = true;
    e->use_bvh = true;
    const char* sn[] = { "SAH", "Median", "Morton" };
    const char* dn[] = { "Small", "Medium", "Large" };
    const int   WARMUP = 3, MEASURE = 10;

    std::ofstream csv("docs/benchmark.csv");
    csv << "strategy,density,tris,node_count,build_ms,render_ms,nodes_per_ray\n";

    std::cout << "\n=========== BENCHMARK (warmup " << WARMUP << " + avg "
              << MEASURE << " frames) ===========\n";
    for (int s = 0; s < 3; ++s)
        for (int d = 0; d < 3; ++d) {
            e->build_strategy = (bvh::BuildStrategy)s;
            e->density = d;
            rebuild_field(e);

            for (int i = 0; i < WARMUP; ++i) bench_frame(e);

            double rsum = 0; long nsum = 0;
            for (int i = 0; i < MEASURE; ++i) {
                FrameMeas m = bench_frame(e);
                rsum += m.render_ms; nsum += m.nodes;
            }
            double ravg = rsum / MEASURE;
            long   rays = (long)e->fb.width * e->fb.height;
            double npr  = rays > 0 ? (double)nsum / MEASURE / rays : 0.0;
            size_t tris = e->field_bvh.triangle_count();
            size_t nn   = e->field_bvh.node_count();

            std::cout << sn[s] << "\t" << dn[d]
                      << "\ttris=" << tris << "\tnodes=" << nn
                      << "\tbuild=" << e->field_build_ms << "ms"
                      << "\trender=" << ravg << "ms"
                      << "\tn/ray=" << npr << "\n";
            csv << sn[s] << "," << dn[d] << "," << tris << "," << nn << ","
                << e->field_build_ms << "," << ravg << "," << npr << "\n";
        }
    std::cout << "=========== done -> docs/benchmark.csv ===========\n\n";
}


void game_render(Game *e, SDL_Texture *sdl_fb_texture, float dt)
{
    uint64_t t = SDL_GetPerformanceCounter();
    double ms_build = 0, ms_core = 0; // build_scene_tris vs. the actual render

    e->fb.clear(0xFF222222);

    long node_visits = 0;   // total BVH nodes tested this frame (all threads)

    if (e->raytrace_mode)
    {
        // --- RAYTRACE PATH ---
        build_scene_tris(e);
        ms_build = tick_ms(t);

        // Prime the camera's lazily-computed, mutable-cached matrices on THIS
        // (main) thread. get_ray_direction() reads cam.rotation(); if its dirty
        // flag is still set when the workers fire, all of them would race to
        // recompute and write the shared mutable cache at once (torn reads ->
        // garbage rays -> thin black lines). After this call the workers only read.
        e->cam.rotation();

        // Kick the persistent pool, then wait for it. Posting each worker's own
        // start semaphore releases exactly that worker; each posts done_sem when
        // its stripe is done.
        for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_SemPost(e->start_sems[i]);
        for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_SemWait(e->done_sem);

        // Sum the per-thread node-visit counters (tree-quality metric).
        for (int i = 0; i < Game::NUM_WORKERS; ++i) node_visits += e->worker_args[i].visits;

        ms_core = tick_ms(t);
    }
    else
    {
        // --- RASTER PATH (the original renderer) ---
        renderConfig cfg;

        render_skybox(e->fb, e->cam, e->skybox_faces); // environment backdrop

        // Static room first. It lives outside `meshes`, so draw it explicitly.
        // The scan's triangle winding is inconsistent, so single-sided culling
        // would punch holes in the walls -> draw it twice, once per winding, to
        // make it effectively double-sided.
        if (e->room_mesh)
        {
            cfg.baseColor = pack(vec3(0.72f, 0.70f, 0.64f));
            e->room_mesh->inverseFaces = false;
            render_mesh(e->fb, e->cam, *e->room_mesh, cfg);
            e->room_mesh->inverseFaces = true;
            render_mesh(e->fb, e->cam, *e->room_mesh, cfg);
            e->room_mesh->inverseFaces = false;
        }

        // The procedural city/spheres (ray tracer reads field_bvh; the raster
        // path reads this mirror mesh). Flat-shaded gray here: raster has no
        // per-triangle color, so you see the geometry masses (buildings, props,
        // sculpture) but not the lit windows/colors that the ray tracer shows.
        if (e->field_mesh)
        {
            cfg.baseColor = pack(vec3(0.55f, 0.55f, 0.58f));
            e->field_mesh->inverseFaces = false;
            render_mesh(e->fb, e->cam, *e->field_mesh, cfg);
            e->field_mesh->inverseFaces = true;
            render_mesh(e->fb, e->cam, *e->field_mesh, cfg);
            e->field_mesh->inverseFaces = false;
        }

        for (auto &[name, mesh_ptr] : e->meshes)
        {
            cfg.baseColor = pack(color_from_hash(name)); // same hue as RT albedo
            render_mesh(e->fb, e->cam, *mesh_ptr, cfg);
        }
        render_raster_shadows(e); // planar projected shadows on the floor
        ms_core = tick_ms(t);
    }

    render_gizmo(e->fb, e->cam, vec3{}, 1.0f);

    if (e->show_bvh) {
        // In raster mode the BVH isn't rebuilt during render, so refresh it
        // here to match the current (animated) scene before drawing.
        if (!e->raytrace_mode) build_scene_tris(e);
        draw_bvh_debug(e);
    }
    double ms_present = tick_ms(t); // gizmo + HUD build so far (cheap)
    long   rays = (long)e->fb.width * e->fb.height;
    double nodes_per_ray = rays > 0 ? (double)node_visits / rays : 0.0;
    char HUD[460] = {0};
    snprintf(HUD, sizeof(HUD),
        "=== SR-LEC (%s) ===\n"
        "Frametime: %.2fms (cap)\n"
        "FPS: %.2f\n"
        "render: %.1fms  build: %.1fms\n"
        "rest: %.1fms\n"
        "Tris: %zu  Nodes: %zu  Room: %zu  Field: %zu\n"
        "Accel: %s   Build: %s\n"
        "nodes/ray: %.1f   field build: %.2fms\n"
        "Threads: %d\n"
        "Move: %s\n"
        "[TAB] mode [B] BVH [V] vis [M] menu [F1] bench\n[SPACE x2] walk/fly\n",
        e->raytrace_mode ? "RAYTRACE" : "RASTER",
        dt * 1000.0f,
        1.0f / dt,
        ms_core, ms_build,
        ms_present,
        e->bvh.triangle_count(), e->bvh.node_count(), e->room_bvh.triangle_count(),
        e->field_bvh.triangle_count(),
        e->use_bvh ? "BVH (fast)" : "BRUTE FORCE (slow)",
        e->build_strategy == bvh::SAH    ? "SAH"
      : e->build_strategy == bvh::Median ? "Median" : "Morton",
        nodes_per_ray, e->field_build_ms,
        Game::NUM_WORKERS,
        e->fly_mode ? "FLY" : "WALK");
    draw_text(e->fb, 22, 22, HUD, 0x88000000, 0x88000000);
    draw_text(e->fb, 20, 20, HUD, 0xFFFFFFFF);

    if (e->show_menu) draw_menu(e); // always on top, last

    SDL_UpdateTexture(sdl_fb_texture, NULL, e->fb.colorBuffer, e->fb.width * sizeof(uint32_t));
}

void game_handle_events(Game *e, SDL_Event &event, bool &running)
{
    if (event.type == SDL_QUIT) running = false;
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
        if (e->show_menu) { e->show_menu = false; SDL_SetRelativeMouseMode(SDL_TRUE); }
        else running = false;
        return;
    }

    // [M] toggles the options menu and releases the mouse cursor while open.
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_m) {
        e->show_menu = !e->show_menu;
        SDL_SetRelativeMouseMode(e->show_menu ? SDL_FALSE : SDL_TRUE);
        return;
    }

    // Double-tap SPACE toggles walk <-> fly.
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) {
        Uint32 now = SDL_GetTicks();
        if (now - e->last_space_ms < 300) e->fly_mode = !e->fly_mode;
        e->last_space_ms = now;
    }

    // [F1] runs the full benchmark sweep (writes docs/benchmark.csv).
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_F1) {
        run_benchmark(e);
        return;
    }

    // While the menu is open, arrows/enter drive it (not the camera).
    if (e->show_menu && event.type == SDL_KEYDOWN) {
        SDL_Keycode k = event.key.keysym.sym;
        if (k == SDLK_UP)        { e->menu_cursor = (e->menu_cursor - 1 + MENU_ITEMS) % MENU_ITEMS; return; }
        if (k == SDLK_DOWN)      { e->menu_cursor = (e->menu_cursor + 1) % MENU_ITEMS; return; }
        if (k == SDLK_LEFT)      { menu_apply(e, -1); return; }
        if (k == SDLK_RIGHT ||
            k == SDLK_RETURN)    { menu_apply(e, +1); return; }
        return; // ignore other keys while the menu is up
    }

    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_TAB)
        e->raytrace_mode = !e->raytrace_mode;
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_b)
        e->use_bvh = !e->use_bvh;
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_v)
        e->show_bvh = !e->show_bvh;
}

void game_shutdown(Game *e)
{
    // Tell workers to exit, wake them so they observe the flag, then join.
    e->workers_running = false;
    for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_SemPost(e->start_sems[i]);
    for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_WaitThread(e->render_workers[i], NULL);
    for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_DestroySemaphore(e->start_sems[i]);
    SDL_DestroySemaphore(e->done_sem);
    delete[] e->worker_args;
    delete e->field_mesh;
}
