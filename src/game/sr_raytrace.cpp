#include <game/sr_raytrace.hpp>
#include <renderer/sr_texture.hpp>
#include <algorithm>
#include <cmath>

const vec3  SUN_DIR    = normalize(vec3(-0.3f, 1.0f, -0.2f));
const float AMBIENT    = 0.15f;
const float SHADOW_EPS = 1e-4f;

uint32_t pack(vec3 c) {
    auto ch = [](float v){ return (uint32_t)(std::clamp(v, 0.f, 1.f) * 255.f + 0.5f); };
    return 0xFF000000 | (ch(c.x) << 16) | (ch(c.y) << 8) | ch(c.z);
}

vec3 get_ray_direction(const camera& cam, int px, int py, int w, int h) {
    float ndcX = (2.0f * (px + 0.5f)) / w - 1.0f;
    float ndcY = 1.0f - (2.0f * (py + 0.5f)) / h;
    float b = tanf(to_radians(cam._fov) * 0.5f);
    float a = b / cam._aspectRatio;
    vec3 dirCam(ndcX * b, ndcY * a, -1.0f);
    vec4 dw = cam.rotation() * vec4(dirCam.x, dirCam.y, dirCam.z, 0.0f);
    return normalize(vec3(dw.x, dw.y, dw.z));
}

static bool ray_intersect_triangle(const ray& r,
                                    const vec3& v0, const vec3& v1, const vec3& v2,
                                    float& t)
{
    const float EPS = 1e-8f;
    vec3 e1 = v1 - v0, e2 = v2 - v0;
    vec3 h  = cross(r.direction, e2);
    float a = dot(e1, h);
    if (std::fabs(a) < EPS) return false;
    float f = 1.0f / a;
    vec3  s = r.origin - v0;
    float u = f * dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;
    vec3  q = cross(s, e1);
    float v = f * dot(r.direction, q);
    if (v < 0.0f || u + v > 1.0f) return false;
    t = f * dot(e2, q);
    return t > EPS;
}

static vec3 sample_face(const texture& t, float u, float v) {
    if (!t.data || t.width <= 0) return vec3(0.0f, 0.0f, 0.0f);
    int x = std::clamp((int)(u * t.width),  0, t.width  - 1);
    int y = std::clamp((int)(v * t.height), 0, t.height - 1);
    uint32_t c = t.data[y * t.width + x];
    return vec3(((c >> 16) & 0xFF) / 255.f, ((c >> 8) & 0xFF) / 255.f, (c & 0xFF) / 255.f);
}

static vec3 sample_sky(const std::array<texture, 6>& f, const vec3& d) {
    float ax = std::fabs(d.x), ay = std::fabs(d.y), az = std::fabs(d.z);
    float m  = std::max(ax, std::max(ay, az));
    if (m <= 0.0f) return vec3(0.0f, 0.0f, 0.0f);
    float px = d.x / m, py = d.y / m, pz = d.z / m;
    int idx; float u, v;
    if (ax >= ay && ax >= az) {
        if (px > 0) { idx = 2; u = pz * 0.5f + 0.5f;   v = py * 0.5f + 0.5f; }
        else        { idx = 1; u = 0.5f - pz * 0.5f;   v = py * 0.5f + 0.5f; }
    } else if (ay >= ax && ay >= az) {
        if (py > 0) { idx = 4; u = px * 0.5f + 0.5f;   v = pz * 0.5f + 0.5f; }
        else        { idx = 5; u = px * 0.5f + 0.5f;   v = 0.5f - pz * 0.5f; }
    } else {
        if (pz > 0) { idx = 3; u = 0.5f - px * 0.5f;  v = py * 0.5f + 0.5f; }
        else        { idx = 0; u = px * 0.5f + 0.5f;   v = py * 0.5f + 0.5f; }
    }
    return sample_face(f[idx], u, v);
}

struct SceneHit { float t; const bvh::Tri* tri; };

static bool scene_intersect(const ray& r, const Game& e, SceneHit& out) {
#ifdef WITH_EMBREE
    if (e.backend == RenderBackend::Embree) {
        float ts = 1e30f, td = 1e30f;
        unsigned ps = 0, pd = 0;
        bool hs = e.embree_static_scene.intersect(r.origin, r.direction, ts, ps);
        bool hd = e.embree_dynamic_scene.intersect(r.origin, r.direction, td, pd);
        if (!hs && !hd) return false;
        if (hd && (!hs || td < ts) && pd < e.embree_dynamic_tris.size())
            { out.t = td; out.tri = &e.embree_dynamic_tris[pd]; }
        else if (hs && ps < e.embree_static_tris.size())
            { out.t = ts; out.tri = &e.embree_static_tris[ps]; }
        else return false;
        return true;
    }
#endif

    bool  hit  = false;
    float best = 1e30f;

    if (e.use_bvh) {
        bvh::Hit h;
        if (e.dynamic_bvh.intersect(r.origin, r.direction, h)) {
            best = h.t; out.t = h.t; out.tri = &e.dynamic_bvh.tri(h.tri); hit = true;
        }
    } else {
        const bvh::Tri* bt = nullptr;
        for (const bvh::Tri& tr : e.rt_tris) {
            float t;
            if (ray_intersect_triangle(r, tr.v0, tr.v1, tr.v2, t) && t < best)
                { best = t; bt = &tr; }
        }
        if (bt) { out.t = best; out.tri = bt; hit = true; }
    }

    auto query_static = [&](const bvh::BVH& b) {
        if (b.empty()) return;
        bvh::Hit h; h.t = best;
        if (b.intersect(r.origin, r.direction, h)) {
            best = h.t; out.t = h.t; out.tri = &b.tri(h.tri); hit = true;
        }
    };
    query_static(e.static_bvh);
    return hit;
}

static bool scene_occluded(const ray& r, const Game& e, float max_t) {
#ifdef WITH_EMBREE
    if (e.backend == RenderBackend::Embree)
        return e.embree_static_scene.occluded(r.origin, r.direction, max_t) ||
               e.embree_dynamic_scene.occluded(r.origin, r.direction, max_t);
#endif

    if (!e.static_bvh.empty() && e.static_bvh.occluded(r.origin, r.direction, max_t))
        return true;
    if (e.use_bvh) return e.dynamic_bvh.occluded(r.origin, r.direction, max_t);
    for (const bvh::Tri& tr : e.rt_tris) {
        float t;
        if (ray_intersect_triangle(r, tr.v0, tr.v1, tr.v2, t) && t < max_t) return true;
    }
    return false;
}

static vec3 tri_smooth_normal(const bvh::Tri& tr, const vec3& P) {
    vec3 e1 = tr.v1 - tr.v0, e2 = tr.v2 - tr.v0, p = P - tr.v0;
    float d11 = dot(e1, e1), d12 = dot(e1, e2), d22 = dot(e2, e2);
    float p1  = dot(p, e1),  p2  = dot(p, e2);
    float det = d11 * d22 - d12 * d12;
    if (std::fabs(det) < 1e-12f) return tr.normal;
    float inv = 1.0f / det;
    float v   = (d22 * p1 - d12 * p2) * inv;
    float w   = (d11 * p2 - d12 * p1) * inv;
    float u   = 1.0f - v - w;
    return normalize(tr.n0 * u + tr.n1 * v + tr.n2 * w);
}

static vec2 tri_interp_uv(const bvh::Tri& tr, const vec3& P) {
    vec3 e1 = tr.v1 - tr.v0, e2 = tr.v2 - tr.v0, p = P - tr.v0;
    float d11 = dot(e1, e1), d12 = dot(e1, e2), d22 = dot(e2, e2);
    float p1  = dot(p, e1),  p2  = dot(p, e2);
    float det = d11 * d22 - d12 * d12;
    if (std::fabs(det) < 1e-12f) return tr.uv0;
    float inv = 1.0f / det;
    float vv  = (d22 * p1 - d12 * p2) * inv;
    float ww  = (d11 * p2 - d12 * p1) * inv;
    float uu  = 1.0f - vv - ww;
    return vec2(tr.uv0.x * uu + tr.uv1.x * vv + tr.uv2.x * ww,
                tr.uv0.y * uu + tr.uv1.y * vv + tr.uv2.y * ww);
}

static vec3 reflect_dir(vec3 d, vec3 n) { return d - n * (2.0f * dot(d, n)); }

static uint32_t lcg(uint32_t& s) { s = s * 1664525u + 1013904223u; return s; }
static float    randf(uint32_t& s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return (s >> 8) * (1.0f / 16777216.0f);
}

vec3 trace_ray(const ray& r, const Game& e, int depth, uint32_t& seed, bool skip_emission) {
    SceneHit hit;
    if (!scene_intersect(r, e, hit))
        return e.skybox_enabled ? sample_sky(e.skybox_faces, r.direction) : vec3(AMBIENT, AMBIENT, AMBIENT) ;

    const bvh::Tri& tr = *hit.tri;

    if (tr.emission.x + tr.emission.y + tr.emission.z > 0.0f)
        return skip_emission ? vec3(0.0f, 0.0f, 0.0f) : tr.emission;

    vec3 P      = r.origin + r.direction * hit.t;
    vec3 N_geom = dot(tr.normal, r.direction) < 0.0f ? tr.normal : -tr.normal;
    vec3 N      = tr.smooth ? tri_smooth_normal(tr, P) : tr.normal;
    if (dot(N, r.direction) > 0.0f) N = -N;

    // Use texture albedo if available, otherwise fall back to material color
    vec3 albedo = tr.albedo;
    if (tr.tex) {
        vec2 uv = tri_interp_uv(tr, P);
        albedo = sample_face(*tr.tex, uv.x, uv.y);
    }

    vec3 light_color;
    if (e.emissive_tris.empty()) {
        ray  shadow(P + N_geom * SHADOW_EPS, SUN_DIR);
        bool in_shadow = scene_occluded(shadow, e, 1e30f);
        float diff = in_shadow ? 0.0f : std::max(0.0f, dot(N, SUN_DIR));
        float l = AMBIENT + diff * (1.0f - AMBIENT);
        light_color = vec3(l, l, l);
    } else {
        // NEE: pick one random emissive triangle, weight by N_lt (unbiased)
        vec3 direct(0.0f, 0.0f, 0.0f);
        int N_lt = (int)e.emissive_tris.size();
        int li   = std::min((int)(randf(seed) * N_lt), N_lt - 1);
        const bvh::Tri& lt = e.emissive_tris[li];
        float r1 = randf(seed), r2 = randf(seed);
        float sq1 = std::sqrt(r1);
        vec3  lp    = lt.v0 * (1.0f - sq1) + lt.v1 * (sq1 * (1.0f - r2)) + lt.v2 * (sq1 * r2);
        vec3  ldir  = lp - P;
        float ldist = std::sqrt(dot(ldir, ldir));
        if (ldist >= 1e-4f) {
            ldir = ldir / ldist;
            float NdotL = dot(N, ldir);
            if (NdotL > 0.0f) {
                ray shadow(P + N_geom * SHADOW_EPS, ldir);
                if (!scene_occluded(shadow, e, ldist - SHADOW_EPS)) {
                    vec3  e1     = lt.v1 - lt.v0, e2 = lt.v2 - lt.v0;
                    float lt_area = std::sqrt(dot(cross(e1, e2), cross(e1, e2))) * 0.5f;
                    float cos_lt  = std::fabs(dot(lt.normal, -ldir));
                    float G       = lt_area * cos_lt / (ldist * ldist);
                    direct = lt.emission * (NdotL * G * (float)N_lt);
                }
            }
        }
        light_color = direct;
    }

    float k_d  = (1.0f - tr.metallic) + tr.metallic * tr.roughness;
    vec3  local = vec3(albedo.x * light_color.x,
                       albedo.y * light_color.y,
                       albedo.z * light_color.z) * k_d;

    if (!e.emissive_tris.empty() && depth < e.max_bounces && tr.metallic < 0.5f) {
        float di1 = randf(seed), di2 = randf(seed);
        float phi   = 2.0f * 3.14159265f * di1;
        float cos_t = std::sqrt(di2);
        float sin_t = std::sqrt(std::max(0.0f, 1.0f - di2));
        vec3 up = std::fabs(N.x) < 0.9f ? vec3(1.0f, 0.0f, 0.0f) : vec3(0.0f, 1.0f, 0.0f);
        vec3 T  = normalize(cross(up, N));
        vec3 B  = cross(N, T);
        vec3 bd = T * (sin_t * std::cos(phi)) + B * (sin_t * std::sin(phi)) + N * cos_t;
        vec3 ind = trace_ray(ray(P + N_geom * SHADOW_EPS, bd), e, depth + 1, seed, true);
        ind = vec3(std::min(ind.x, 2.0f), std::min(ind.y, 2.0f), std::min(ind.z, 2.0f));
        local = local + vec3(albedo.x * ind.x, albedo.y * ind.y, albedo.z * ind.z) * k_d;
    }

    float cosV = std::max(0.0f, dot(N, -r.direction));
    float F0   = tr.metallic > 0.5f
                 ? (albedo.x + albedo.y + albedo.z) / 3.0f
                 : 0.04f;
    float fres = F0 + (1.0f - F0) * std::pow(1.0f - cosV, 5.0f);
    float spec = fres * (1.0f - tr.roughness);

    if (depth >= e.max_bounces || spec <= 0.01f || !e.reflections) return local;

    vec3 R = reflect_dir(r.direction, N);
    float RdotG = dot(R, N_geom);
    if (RdotG < 0.0f) {
        R = normalize(R - N_geom * RdotG);
        R = normalize(R + N_geom * 1e-4f);
    }
    vec3 refl = trace_ray(ray(P + N_geom * SHADOW_EPS, R), e, depth + 1, seed);

    vec3 tint = tr.metallic > 0.5f ? albedo : vec3(1.0f, 1.0f, 1.0f);
    return local * (1.0f - spec) + vec3(refl.x * spec * tint.x,
                                        refl.y * spec * tint.y,
                                        refl.z * spec * tint.z);
}

vec3 trace_ray(const ray& r, const Game& e) {
    uint32_t seed = 0;
    return trace_ray(r, e, 0, seed);
}

int worker_thread(void* data) {
    thread_data* td = (thread_data*)data;
    Game* e = td->game;
    int   id = td->thread_id;

    while (true) {
        SDL_SemWait(e->start_sems[id]);
        if (!e->workers_running) break;

        bvh::reset_thread_node_visits();

        int stripe  = e->fb.height / Game::NUM_WORKERS;
        int y_start = stripe * id;
        int y_end   = (id == Game::NUM_WORKERS - 1) ? e->fb.height : stripe * (id + 1);

        int spp = e->spp;
        for (int y = y_start; y < y_end; ++y)
            for (int x = 0; x < e->fb.width; ++x) {
                vec3 sum(0.0f, 0.0f, 0.0f);
                for (int s = 0; s < spp; ++s) {
                    ray r(e->cam._position, get_ray_direction(e->cam, x, y, e->fb.width, e->fb.height));
                    uint32_t seed = ((uint32_t)(y * e->fb.width + x) * 2654435761u)
                                  ^ ((uint32_t)s * 805459861u);
                    sum = sum + trace_ray(r, *e, 0, seed);
                }
                vec3 col = sum * (1.0f / spp);
                e->fb.colorBuffer[y * e->fb.width + x] = pack(col) | 0xFF000000;
            }

        td->visits = bvh::take_thread_node_visits();
        SDL_SemPost(e->done_sem);
    }
    return 0;
}
