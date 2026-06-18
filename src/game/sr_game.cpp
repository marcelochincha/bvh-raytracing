#include <game/sr_game.hpp>

#include <sr_config.hpp>
#include <unordered_map>
#include <renderer/sr_renderer.hpp>
#include <renderer/bvh.hpp>
#include <algorithm>
#include <string>
#include <cmath>

// =============================================================================
// Minimal "old-school computer graphics" scene: a floor and a few primitives,
// a free-fly camera (WASD + mouse look). No physics, no gameplay — just a
// static set of meshes to develop/visualize the renderer (and the upcoming
// raytracer) against.
// =============================================================================

// Set to 0 to skip loading the heavy room .ply (faster startup / lighter scene).
// Everything else is guarded on room_bvh/room_mesh being empty, so it just works.
#define ENABLE_ROOM 0

const float kMouseSensitivity = 0.0025f;
const float moveSpeed = 6.0f;

// The raytracer triangle lives in the BVH module now (geometry + shading
// payload). Alias keeps the rest of this file reading the same.
using RTTri = bvh::Tri;

struct Game
{
    framebuffer fb;
    camera cam;

    // Free-fly camera state.
    float yaw   = to_radians(-135.0f);
    float pitch = to_radians(-20.0f);
    vec3  position = vec3(4.0f, 3.0f, 4.0f);

    float time = 0.0f; // seconds elapsed, drives scene animation
    bool  raytrace_mode = true; // TAB toggles raytracer <-> rasterizer
    bool  show_bvh = false;     // [V] overlays the BVH boxes

    std::unordered_map<std::string, mesh*> meshes;
    std::vector<RTTri> rt_tris; // Scene data preprocessed for ray tracing (triangles with precomputed normals and albedos)
    std::unordered_map<std::string, float> reflectivity_map; // For rendering simple shapes like axes, grids, etc.

    bvh::BVH bvh;            // acceleration structure, rebuilt each frame
    bool     use_bvh = true; // [B] toggles BVH vs brute-force triangle loop

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


void build_scene_tris(Game* e) {
    e->rt_tris.clear();
    for (auto& [name, mp] : e->meshes) {
        const mesh& m = *mp;
        mat4 model = m.modelMatrix();
        vec3 albedo = color_from_hash(name);
        for (const triangle& tri : m.faces) {
            vec3 v0 = vec3(model * m.vertices[tri.v0].p);
            vec3 v1 = vec3(model * m.vertices[tri.v1].p);
            vec3 v2 = vec3(model * m.vertices[tri.v2].p);
            vec3 n  = normalize(cross(v1 - v0, v2 - v0));
            e->rt_tris.push_back({v0, v1, v2, n, albedo, (float)e->reflectivity_map[name]});
        }
    }

    // Rebuild the BVH over this frame's triangle soup. (The scene is dynamic,
    // so the tree must be rebuilt every frame. For a static scene you'd build
    // it once.) build() copies the triangles in, so rt_tris stays valid for
    // the brute-force comparison path.
    e->bvh.build(e->rt_tris);
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
const int MAX_RAY_DEPTH = 3;
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

    // --- static room (always via its prebuilt BVH) ---
    if (!e.room_bvh.empty()) {
        bvh::Hit h; h.t = best; // clamp the search to the nearest hit so far
        if (e.room_bvh.intersect(r.origin, r.direction, h)) {
            out.t = h.t; out.tri = &e.room_bvh.tri(h.tri); hit = true;
        }
    }
    return hit;
}

// Any-hit (shadow ray). Same BVH-vs-brute-force switch + the static room.
static bool scene_occluded(const ray& r, const Game& e, float max_t) {
    if (!e.room_bvh.empty() && e.room_bvh.occluded(r.origin, r.direction, max_t))
        return true;
    if (e.use_bvh) return e.bvh.occluded(r.origin, r.direction, max_t);
    for (const bvh::Tri& tr : e.rt_tris) {
        float t;
        if (ray_intersect_triangle(r, tr.v0, tr.v1, tr.v2, t) && t < max_t) return true;
    }
    return false;
}

//Returns the raw color
vec3 trace_ray(const ray& r, const Game& e, int depth = 0) {
    SceneHit hit;
    // Miss -> the sky itself (cubemap) is the background.
    if (!scene_intersect(r, e, hit)) return sample_sky(e.skybox_faces, r.direction);

    const bvh::Tri& tr = *hit.tri;
    vec3 P = r.origin + r.direction * hit.t;
    vec3 N = tr.normal;
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
    if (depth >= MAX_RAY_DEPTH || tr.reflectivity <= 0.0f)
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
    create_plane(floor_mesh, 20.0f, 20.0f);
    floor_mesh->setPosition(vec3(0.0f, 0.0f, 0.0f));
    e->meshes["FLOOR"] = floor_mesh;
    e->reflectivity_map["FLOOR"] = 0.2f; // Example reflectivity for the floor


    // A cube
    mesh* cube = new mesh;
    create_cube(cube, 1.0f);
    cube->setPosition(vec3(-1.5f, 0.5f, 0.0f));
    e->meshes["CUBE"] = cube;
    e->reflectivity_map["CUBE"] = 0.1f; // Example reflectivity for the cube

    // A sphere
    mesh* sphere = new mesh;
    create_sphere(sphere, 0.75f, 4, 4);
    sphere->setPosition(vec3(1.5f, 0.75f, 0.0f));
    e->meshes["SPHERE"] = sphere;
    e->reflectivity_map["SPHERE"] = 0.3f; // Example reflectivity for the sphere

    // A cylinder
    mesh* cyl = new mesh;
    create_cylinder(cyl, 0.5f, 1.5f, 4);
    cyl->setPosition(vec3(0.0f, 0.75f, -2.0f));
    e->meshes["CYLINDER"] = cyl;
    e->reflectivity_map["CYLINDER"] = 0.15f; // Example reflectivity for the cylinder

    // ===================== Una casita ============================
    // Cuerpo (cubo achatado) + techo (cuña) + puerta + chimenea.
    const vec3 house_at = vec3(-3.5f, 0.0f, 3.0f);

    mesh* house_body = new mesh;
    create_cube(house_body, 1.0f);
    house_body->setScale(vec3(2.0f, 1.4f, 2.0f));
    house_body->setPosition(house_at + vec3(0.0f, 0.7f, 0.0f));
    e->meshes["HOUSE_BODY"] = house_body;
    e->reflectivity_map["HOUSE_BODY"] = 0.0f;

    mesh* house_roof = new mesh;
    create_wedge(house_roof, 2.4f, 1.0f, 2.4f); // ancho, alto, profundidad
    house_roof->setPosition(house_at + vec3(0.0f, 1.9f, 0.0f));
    e->meshes["HOUSE_ROOF"] = house_roof;
    e->reflectivity_map["HOUSE_ROOF"] = 0.0f;

    mesh* house_door = new mesh;
    create_cube(house_door, 1.0f);
    house_door->setScale(vec3(0.5f, 0.9f, 0.1f));
    house_door->setPosition(house_at + vec3(0.0f, 0.45f, 1.0f));
    e->meshes["HOUSE_DOOR"] = house_door;
    e->reflectivity_map["HOUSE_DOOR"] = 0.0f;

    mesh* chimney = new mesh;
    create_cube(chimney, 1.0f);
    chimney->setScale(vec3(0.3f, 0.9f, 0.3f));
    chimney->setPosition(house_at + vec3(0.6f, 2.1f, -0.4f));
    e->meshes["HOUSE_CHIMNEY"] = chimney;
    e->reflectivity_map["HOUSE_CHIMNEY"] = 0.0f;

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

    // Static room scan wraps the procedural scene (its own BVH, built once).
#if ENABLE_ROOM
    load_room(e, "res/textures/iscv2.ply");
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
    if (keys[SDL_SCANCODE_LSHIFT]) inputDir.y -= 1.0f;
    if (keys[SDL_SCANCODE_SPACE])  inputDir.y += 1.0f;

    float s = sinf(e->yaw);
    float c = cosf(e->yaw);

    vec3 moveDir;
    moveDir.x = (inputDir.x * c - inputDir.z * s) * moveSpeed;
    moveDir.z = (inputDir.x * s + inputDir.z * c) * moveSpeed;
    moveDir.y =  inputDir.y * moveSpeed;

    e->position = e->position + moveDir * dt;

    e->cam.setPosition(e->position);
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
    e->bvh.debug_nodes(nodes);

    // Depth -> color palette (cycles), so each level reads distinctly.
    static const uint32_t palette[] = {
        0xFFFF4040, 0xFFFFA040, 0xFFFFFF40, 0xFF40FF40,
        0xFF40FFFF, 0xFF4080FF, 0xFFC040FF, 0xFFFF40C0,
    };
    for (const auto &n : nodes) {
        uint32_t col = palette[n.depth % (sizeof(palette)/sizeof(palette[0]))];
        draw_aabb_wire(e->fb, e->cam, n.bounds, col);
    }
}

// Tiny timer helper: returns elapsed milliseconds since `since` and resets it.
static double tick_ms(uint64_t& since) {
    uint64_t now = SDL_GetPerformanceCounter();
    double ms = (now - since) * 1000.0 / SDL_GetPerformanceFrequency();
    since = now;
    return ms;
}

void game_render(Game *e, SDL_Texture *sdl_fb_texture, float dt)
{
    uint64_t t = SDL_GetPerformanceCounter();
    double ms_build = 0, ms_core = 0; // build_scene_tris vs. the actual render

    e->fb.clear(0xFF222222);

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
    char HUD[460] = {0};
    snprintf(HUD, sizeof(HUD),
        "=== SR-LEC (%s) ===\n"
        "Frametime: %.2fms (cap)\n"
        "FPS: %.2f\n"
        "render: %.1fms  build: %.1fms\n"
        "rest: %.1fms\n"
        "Tris: %zu  Nodes: %zu  Room: %zu\n"
        "Accel: %s\n"
        "Threads: %d\n"
        "[TAB] raster/raytrace\n[B] BVH on/off\n[V] show BVH\n",
        e->raytrace_mode ? "RAYTRACE" : "RASTER",
        dt * 1000.0f,
        1.0f / dt,
        ms_core, ms_build,
        ms_present,
        e->bvh.triangle_count(), e->bvh.node_count(), e->room_bvh.triangle_count(),
        e->use_bvh ? "BVH (fast)" : "BRUTE FORCE (slow)",
        Game::NUM_WORKERS);
    draw_text(e->fb, 22, 22, HUD, 0x88000000, 0x88000000);
    draw_text(e->fb, 20, 20, HUD, 0xFFFFFFFF);

    SDL_UpdateTexture(sdl_fb_texture, NULL, e->fb.colorBuffer, e->fb.width * sizeof(uint32_t));
}

void game_handle_events(Game *e, SDL_Event &event, bool &running)
{
    if (event.type == SDL_QUIT) running = false;
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) running = false;
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
}
