#include <game/sr_scene.hpp>
#include <game/sr_raytrace.hpp>
#include <renderer/sr_ocl.hpp>
#include <renderer/sr_texture.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <unordered_map>
#include <memory>

#define FIELD_SPHERES 8

vec3 color_from_hash(const std::string& name) {
    uint32_t h = std::hash<std::string>{}(name);
    return vec3(((h >> 16) & 0xFF) / 255.f, ((h >> 8) & 0xFF) / 255.f, (h & 0xFF) / 255.f);
}

static int scene_count(int d) { static const int c[3] = { 64, 256, 1024 }; return c[d]; }
static int city_grid(int d)   { static const int g[3] = { 10, 16, 24 };    return g[d]; }

static vec3 hsv_to_rgb(float h, float s, float v) {
    float c = v * s, hp = h * 6.0f;
    float x = c * (1.0f - std::fabs(std::fmod(hp, 2.0f) - 1.0f));
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
                       const vec3& albedo, float roughness, float metallic, float ior,
                       int slices, int stacks, bool smooth = true)
{
    for (int st = 0; st < stacks; ++st) {
        float ph0 = (float)st       / stacks * 3.14159265f;
        float ph1 = (float)(st + 1) / stacks * 3.14159265f;
        for (int sl = 0; sl < slices; ++sl) {
            float th0 = (float)sl       / slices * 6.2831853f;
            float th1 = (float)(sl + 1) / slices * 6.2831853f;
            auto p = [&](float ph, float th) {
                return vec3(std::sin(ph) * std::cos(th), std::cos(ph), std::sin(ph) * std::sin(th));
            };
            vec3 v0 = center + p(ph0, th0) * radius, v1 = center + p(ph1, th0) * radius;
            vec3 v2 = center + p(ph1, th1) * radius, v3 = center + p(ph0, th1) * radius;
            vec3 n0n = normalize(p(ph0, th0)), n1n = normalize(p(ph1, th0));
            vec3 n2n = normalize(p(ph1, th1)), n3n = normalize(p(ph0, th1));
            vec3 fn = normalize(cross(v1 - v0, v2 - v0));
            out.push_back({ v0, v1, v2, fn, albedo, roughness, metallic, ior, smooth, n0n, n1n, n2n });
            out.push_back({ v0, v2, v3, fn, albedo, roughness, metallic, ior, smooth, n0n, n2n, n3n });
        }
    }
}

static void add_box(std::vector<bvh::Tri>& out,
                    const vec3& lo, const vec3& hi,
                    const vec3& albedo, float roughness,
                    float metallic = 0.0f, float ior = 1.5f)
{
    vec3 v[8] = {
        {lo.x,lo.y,lo.z},{hi.x,lo.y,lo.z},{hi.x,lo.y,hi.z},{lo.x,lo.y,hi.z},
        {lo.x,hi.y,lo.z},{hi.x,hi.y,lo.z},{hi.x,hi.y,hi.z},{lo.x,hi.y,hi.z}
    };
    auto face = [&](int a, int b, int c, int d, const vec3& n) {
        out.push_back({ v[a],v[b],v[c], n, albedo, roughness, metallic, ior });
        out.push_back({ v[a],v[c],v[d], n, albedo, roughness, metallic, ior });
    };
    face(4,5,6,7,{ 0, 1, 0}); face(3,2,1,0,{ 0,-1, 0});
    face(0,3,7,4,{-1, 0, 0}); face(1,5,6,2,{ 1, 0, 0});
    face(0,1,5,4,{ 0, 0,-1}); face(3,7,6,2,{ 0, 0, 1});
}

static void add_box_c(std::vector<bvh::Tri>& out, const vec3& center, const vec3& half,
                      const vec3& albedo, float roughness,
                      float metallic = 0.0f, float ior = 1.5f) {
    add_box(out, center - half, center + half, albedo, roughness, metallic, ior);
}

// Places the free camera at a fixed starting pose so every scene frames its
// subject on load, instead of leaving the camera wherever the previous scene
// left it (which forces a manual fly-around each time). `pos` looks at `target`
// using the same forward convention as euler_deg_looking_at / gameplay look.
static void set_start_view(Game* e, const vec3& pos, const vec3& target) {
    vec3 d = normalize(target - pos);
    float pitch = std::asin(std::clamp(d.y, -1.0f, 1.0f));
    float yaw   = std::atan2(d.x, -d.z);
    e->position = pos;
    e->pitch    = pitch;
    e->yaw      = yaw;
    e->cam.setPosition(pos);
    e->cam.setRotation(vec3(pitch, yaw, 0.0f));
}

static void add_humanoid(std::vector<bvh::Tri>& out, const vec3& base,
                         const vec3& skin, const vec3& shirt, float phase)
{
    float s   = std::sin(phase);
    float bob = std::fabs(std::sin(phase)) * 0.06f;
    float y   = base.y + bob, leg = s * 0.16f;
    const vec3 pants(0.12f, 0.12f, 0.18f);
    add_box_c(out, vec3(base.x - 0.07f, y + 0.35f, base.z + leg),  vec3(0.06f, 0.35f, 0.06f), pants, 0.02f);
    add_box_c(out, vec3(base.x + 0.07f, y + 0.35f, base.z - leg),  vec3(0.06f, 0.35f, 0.06f), pants, 0.02f);
    add_box_c(out, vec3(base.x,         y + 0.85f, base.z),        vec3(0.16f, 0.28f, 0.10f), shirt, 0.03f);
    add_box_c(out, vec3(base.x - 0.22f, y + 0.85f, base.z - leg),  vec3(0.05f, 0.24f, 0.05f), shirt, 0.03f);
    add_box_c(out, vec3(base.x + 0.22f, y + 0.85f, base.z + leg),  vec3(0.05f, 0.24f, 0.05f), shirt, 0.03f);
    add_box_c(out, vec3(base.x,         y + 1.27f, base.z),        vec3(0.09f, 0.09f, 0.09f), skin,  0.02f);
}

static void add_windowed_building(std::vector<bvh::Tri>& out,
                                  const vec3& lo, const vec3& hi, uint32_t& rng)
{
    auto urand = [&]() { rng = rng * 1664525u + 1013904223u; return (rng >> 8) / 16777216.0f; };
    add_box(out, lo, hi, vec3(0.10f, 0.10f, 0.12f), 0.02f);

    const float spacing = 1.2f, inset = 0.14f, off = 0.02f;
    const vec3  up(0.0f, 1.0f, 0.0f);
    auto wall = [&](const vec3& o, const vec3& uax, const vec3& n, float len, float hgt) {
        int cols = (int)std::clamp(len / spacing, 1.0f, 10.0f);
        int rows = (int)std::clamp(hgt / spacing, 1.0f, 16.0f);
        float cu = len / cols, cv = hgt / rows;
        for (int j = 0; j < rows; ++j)
            for (int i = 0; i < cols; ++i) {
                float u0 = i*cu+inset, u1 = (i+1)*cu-inset;
                float v0 = j*cv+inset, v1 = (j+1)*cv-inset;
                vec3 p0 = o+uax*u0+up*v0+n*off, p1 = o+uax*u1+up*v0+n*off;
                vec3 p2 = o+uax*u1+up*v1+n*off, p3 = o+uax*u0+up*v1+n*off;
                bool  lit = urand() < 0.34f;
                vec3  alb = lit ? vec3(0.96f,0.85f,0.52f) : vec3(0.15f,0.23f,0.33f);
                float ro  = lit ? 0.9f : 0.05f;
                out.push_back({ p0,p1,p2, n, alb, ro }); out.push_back({ p0,p2,p3, n, alb, ro });
            }
    };
    float H = hi.y - lo.y;
    wall(vec3(lo.x,lo.y,hi.z), vec3( 1,0,0), vec3(0,0, 1), hi.x-lo.x, H);
    wall(vec3(lo.x,lo.y,lo.z), vec3( 1,0,0), vec3(0,0,-1), hi.x-lo.x, H);
    wall(vec3(hi.x,lo.y,lo.z), vec3(0,0, 1), vec3( 1,0,0), hi.z-lo.z, H);
    wall(vec3(lo.x,lo.y,lo.z), vec3(0,0, 1), vec3(-1,0,0), hi.z-lo.z, H);
}

// Texture cache: avoids reloading the same file multiple times.
static std::map<std::string, std::unique_ptr<texture>> s_tex_cache;

static const texture* load_cached_texture(const std::string& path) {
    auto it = s_tex_cache.find(path);
    if (it != s_tex_cache.end()) return it->second.get();
    auto t = std::make_unique<texture>();
    if (!load_png_texture(path, *t)) {
        std::cout << "load_obj: texture not found: " << path << "\n";
        return nullptr;
    }
    const texture* ptr = t.get();
    s_tex_cache[path] = std::move(t);
    return ptr;
}

static void load_obj(const char* path, const vec3& base, float scale,
                     const vec3& albedo_def, float roughness_def,
                     std::vector<bvh::Tri>& out,
                     float metallic_def = 0.0f, float ior_def = 1.5f)
{
    struct ObjMat {
        vec3 albedo; float roughness, metallic, ior; vec3 emission;
        std::string map_kd;
    };
    const ObjMat default_mat = { albedo_def, roughness_def, metallic_def, ior_def, vec3(0,0,0), "" };

    std::string dir(path);
    auto sl = dir.find_last_of("/\\");
    dir = (sl != std::string::npos) ? dir.substr(0, sl + 1) : "";

    std::map<std::string, ObjMat> mats;
    auto load_mtl = [&](const std::string& fname) {
        std::ifstream mf(dir + fname);
        if (!mf) return;
        std::string cur, line;
        ObjMat m = default_mat;
        vec3 ks(0, 0, 0);
        bool has_pr = false;

        auto flush = [&]() {
            if (cur.empty()) return;
            float ks_lum = (ks.x + ks.y + ks.z) / 3.0f;
            float kd_lum = (m.albedo.x + m.albedo.y + m.albedo.z) / 3.0f;
            if (!has_pr && ks_lum > 0.3f && kd_lum < 0.2f) {
                m.metallic = std::clamp(ks_lum, 0.0f, 1.0f);
                m.albedo   = ks;
            }
            mats[cur] = m;
        };

        while (std::getline(mf, line)) {
            std::istringstream ss(line);
            std::string key; ss >> key;
            if      (key == "newmtl") { flush(); ss >> cur; m = default_mat; ks = vec3(0,0,0); has_pr = false; }
            else if (key == "Kd")     { ss >> m.albedo.x >> m.albedo.y >> m.albedo.z; }
            else if (key == "Ks")     { ss >> ks.x >> ks.y >> ks.z; }
            else if (key == "Ke")     { ss >> m.emission.x >> m.emission.y >> m.emission.z; }
            else if (key == "Ns")     { float n; ss >> n; if (!has_pr) m.roughness = 1.0f - std::clamp(n / 900.0f, 0.0f, 1.0f); }
            else if (key == "Pr")     { ss >> m.roughness; has_pr = true; }
            else if (key == "Pm")     { ss >> m.metallic; }
            else if (key == "Ni")     { ss >> m.ior; }
            else if (key == "map_Kd") { std::string fn; std::getline(ss, fn); size_t a = fn.find_first_not_of(" \t"); if (a != std::string::npos) m.map_kd = fn.substr(a); }
        }
        flush();
    };

    std::ifstream f(path);
    if (!f) { std::cout << "load_obj: could not open " << path << "\n"; return; }

    std::vector<vec3> verts;
    std::vector<vec2> uvs;

    struct FaceVert { int vi, ti; };
    struct Face { std::vector<FaceVert> corners; std::string mat; };
    std::vector<Face> faces;
    std::string cur_mat, line;

    // Fast hand-parsed geometry loop (avoids istringstream allocation per line)
    while (std::getline(f, line)) {
        const char* s = line.c_str();
        while (*s == ' ' || *s == '\t') ++s;

        if (s[0] == 'v' && (s[1] == ' ' || s[1] == '\t')) {
            char* e; const char* p = s + 2;
            float x = std::strtof(p, &e); p = e;
            float y = std::strtof(p, &e); p = e;
            float z = std::strtof(p, &e);
            verts.push_back(vec3(x, y, z));
        }
        else if (s[0] == 'v' && s[1] == 't' && (s[2] == ' ' || s[2] == '\t')) {
            char* e; const char* p = s + 3;
            float u = std::strtof(p, &e); p = e;
            float v = std::strtof(p, &e);
            uvs.push_back(vec2(u, v));
        }
        else if (s[0] == 'v' && s[1] == 'n') {
            // normals recomputed as smooth normals below; skip
        }
        else if (s[0] == 'f' && (s[1] == ' ' || s[1] == '\t')) {
            Face fc; fc.mat = cur_mat;
            const int nv = (int)verts.size(), nt = (int)uvs.size();
            const char* p = s + 2;
            while (*p) {
                while (*p == ' ' || *p == '\t') ++p;
                if (!*p) break;
                char* e;
                long vi = std::strtol(p, &e, 10); p = e;
                long ti = 0; bool hasT = false;
                if (*p == '/') {
                    ++p;
                    if (*p != '/') { ti = std::strtol(p, &e, 10); p = e; hasT = true; }
                    if (*p == '/') { ++p; std::strtol(p, &e, 10); p = e; } // skip vn
                }
                if (vi == 0) continue;
                int vIdx = (vi > 0) ? (int)vi - 1 : nv + (int)vi;
                int tIdx = (hasT && ti != 0) ? ((ti > 0) ? (int)ti - 1 : nt + (int)ti) : -1;
                fc.corners.push_back({vIdx, tIdx});
            }
            if (fc.corners.size() >= 3) faces.push_back(std::move(fc));
        }
        else if (s[0] == 'u' || s[0] == 'm') {
            std::istringstream ss(s);
            std::string tag; ss >> tag;
            if      (tag == "usemtl") { cur_mat.clear(); ss >> cur_mat; }
            else if (tag == "mtllib") { std::string fn; ss >> fn; load_mtl(fn); }
        }
    }

    // Accumulate smooth normals (area-weighted sum of face normals per vertex)
    auto valid_v = [&](int i) { return i >= 0 && i < (int)verts.size(); };
    auto valid_t = [&](int i) { return i >= 0 && i < (int)uvs.size(); };
    std::vector<vec3> vn(verts.size(), vec3(0, 0, 0));
    for (auto& fc : faces)
        for (size_t i = 1; i + 1 < fc.corners.size(); ++i) {
            int a = fc.corners[0].vi, b = fc.corners[i].vi, c = fc.corners[i+1].vi;
            if (!valid_v(a) || !valid_v(b) || !valid_v(c)) continue;
            vec3 n = cross(verts[b] - verts[a], verts[c] - verts[a]); // area-weighted
            vn[a] = vn[a] + n; vn[b] = vn[b] + n; vn[c] = vn[c] + n;
        }
    for (auto& n : vn) n = normalize(n);

    std::map<std::string, const texture*> mat_tex;

    for (auto& fc : faces) {
        auto it = mats.find(fc.mat);
        const ObjMat& mat = (it != mats.end()) ? it->second : default_mat;

        const texture* tex = nullptr;
        if (!mat.map_kd.empty()) {
            auto tt = mat_tex.find(mat.map_kd);
            if (tt == mat_tex.end()) {
                tex = load_cached_texture(dir + mat.map_kd);
                mat_tex[mat.map_kd] = tex;
            } else {
                tex = tt->second;
            }
        }

        for (size_t i = 1; i + 1 < fc.corners.size(); ++i) {
            int a = fc.corners[0].vi, b = fc.corners[i].vi, c = fc.corners[i+1].vi;
            if (!valid_v(a) || !valid_v(b) || !valid_v(c)) continue;
            vec3 v0 = base + verts[a] * scale, v1 = base + verts[b] * scale, v2 = base + verts[c] * scale;
            vec3 fn = normalize(cross(v1 - v0, v2 - v0));

            int ta = fc.corners[0].ti, tb = fc.corners[i].ti, tc = fc.corners[i+1].ti;
            vec2 t0 = valid_t(ta) ? uvs[ta] : vec2(0,0);
            vec2 t1 = valid_t(tb) ? uvs[tb] : vec2(0,0);
            vec2 t2 = valid_t(tc) ? uvs[tc] : vec2(0,0);

            bvh::Tri tri { v0, v1, v2, fn, mat.albedo, mat.roughness, mat.metallic, mat.ior,
                           false, vn[a], vn[b], vn[c] };
            tri.emission = mat.emission;
            tri.uv0 = t0; tri.uv1 = t1; tri.uv2 = t2;
            tri.tex = tex;
            out.push_back(tri);
        }
    }
    std::cout << "load_obj: " << path << " -> " << verts.size() << " verts, "
              << faces.size() << " faces, " << mats.size() << " materials\n";
}

static void rebuild_field_mesh(Game* e, const std::vector<bvh::Tri>& tris) {
    delete e->field_mesh;
    e->field_mesh = new mesh;
    e->field_mesh->vertices.reserve(tris.size() * 3);
    e->field_mesh->faces.reserve(tris.size());
    for (const auto& t : tris) {
        uint32_t b = (uint32_t)e->field_mesh->vertices.size();
        e->field_mesh->vertices.push_back({ t.v0, {0.0f, 0.0f} });
        e->field_mesh->vertices.push_back({ t.v1, {0.0f, 0.0f} });
        e->field_mesh->vertices.push_back({ t.v2, {0.0f, 0.0f} });
        e->field_mesh->faces.push_back({ b, b+1, b+2 });
    }
    e->field_mesh->_modelMatrixDirty = true;
}

static void build_field(Game* e) {
    e->peds.clear();
    const int count = scene_count(e->density);
    std::vector<bvh::Tri> tris;
    tris.reserve((size_t)count * FIELD_SLICES * FIELD_STACKS * 2);
    uint32_t rng = 0xC0FFEEu;
    auto urand = [&]() { rng = rng*1664525u+1013904223u; return (rng>>8)/16777216.0f; };
    const int   side = (int)std::round(std::sqrt((float)count));
    const float span = 14.0f, cell = span / side;
    int idx = 0;
    for (int z = 0; z < side && idx < count; ++z)
        for (int x = 0; x < side && idx < count; ++x, ++idx) {
            float jx = (urand()-0.5f)*cell*0.6f, jz = (urand()-0.5f)*cell*0.6f;
            float px = -span*0.5f+cell*(x+0.5f)+jx, pz = -span*0.5f+cell*(z+0.5f)+jz;
            float r  = cell*(0.20f+urand()*0.18f);
            vec3  alb = hsv_to_rgb(urand(), 0.7f, 0.92f);
            add_sphere(tris, vec3(px,r,pz), r, alb, 0.2f+urand()*0.6f, 0.0f, 1.5f, FIELD_SLICES, FIELD_STACKS);
        }
    add_box(tris, vec3(-10.f,-0.02f,-10.f), vec3(10.f,0.f,10.f), vec3(0.14f,0.14f,0.15f), 0.8f);
    set_start_view(e, vec3(0.0f, 5.0f, 13.0f), vec3(0.0f, 1.0f, 0.0f));
    rebuild_field_mesh(e, tris);
    e->static_bvh.build(std::move(tris), e->build_strategy);
    std::cout << "Field: " << count << " spheres, " << e->static_bvh.triangle_count()
              << " tris, " << e->static_bvh.node_count() << " BVH nodes\n";
}

static void build_city(Game* e) {
    const int   grid   = city_grid(e->density);
    const float span   = CITY_SPAN, cell = span / grid;
    const float street = cell * 0.15f, avenue = cell * 0.8f;
    uint32_t rng = 0x1234u;
    auto urand = [&]() { rng = rng*1664525u+1013904223u; return (rng>>8)/16777216.0f; };
    std::vector<bvh::Tri> tris;
    int built = 0;
    for (int z = 0; z < grid; ++z)
        for (int x = 0; x < grid; ++x) {
            if (urand() < 0.12f) continue;
            float cx = -span*0.5f+cell*(x+0.5f), cz = -span*0.5f+cell*(z+0.5f);
            if (std::fabs(cx) < avenue || std::fabs(cz) < avenue) continue;
            float half = cell*0.5f-street;
            float d = std::sqrt(cx*cx+cz*cz)/(span*0.5f);
            float h = cell*(1.0f+(1.0f-std::clamp(d,0.0f,1.0f))*7.0f+urand()*4.0f);
            add_windowed_building(tris, vec3(cx-half,0,cz-half), vec3(cx+half,h,cz+half), rng);
            ++built;
        }

    const float dash=0.6f, gap=0.6f, lw=0.06f;
    const vec3  laneCol(0.90f,0.80f,0.20f);
    for (float p = -span*0.5f; p < span*0.5f; p += dash+gap) {
        add_box_c(tris, vec3(0,0.012f,p+dash*0.5f),     vec3(lw,0.006f,dash*0.5f), laneCol, 0.0f);
        add_box_c(tris, vec3(p+dash*0.5f,0.012f,0),     vec3(dash*0.5f,0.006f,lw), laneCol, 0.0f);
    }
    {
        vec3 pole(avenue-0.4f,0,avenue-0.4f);
        add_box_c(tris, vec3(pole.x,1.5f,pole.z),       vec3(0.05f,1.5f,0.05f),  vec3(0.14f,0.14f,0.14f), 0.02f);
        add_box_c(tris, vec3(pole.x-0.35f,3.0f,pole.z), vec3(0.35f,0.05f,0.05f), vec3(0.14f,0.14f,0.14f), 0.02f);
        vec3 hl(pole.x-0.6f,2.78f,pole.z);
        add_box_c(tris, hl,                              vec3(0.08f,0.30f,0.08f), vec3(0.09f,0.09f,0.09f), 0.02f);
        add_box_c(tris, hl+vec3(0, 0.19f,0),            vec3(0.05f,0.05f,0.04f), vec3(0.10f,0.04f,0.04f), 0.0f);
        add_box_c(tris, hl+vec3(0, 0.00f,0),            vec3(0.05f,0.05f,0.04f), vec3(0.10f,0.09f,0.03f), 0.0f);
        add_box_c(tris, hl+vec3(0,-0.19f,0),            vec3(0.07f,0.07f,0.05f), vec3(0.15f,1.00f,0.30f), 0.0f);
    }
    auto add_car = [&](const vec3& c, const vec3& col) {
        add_box_c(tris, c+vec3(0,0.25f,0),     vec3(0.34f,0.17f,0.70f), col,                        0.25f);
        add_box_c(tris, c+vec3(0,0.50f,0.03f), vec3(0.30f,0.12f,0.34f), vec3(0.14f,0.20f,0.26f),   0.10f);
        const float wx=0.33f,wz=0.46f;
        add_box_c(tris, c+vec3( wx,0.08f, wz), vec3(0.08f,0.08f,0.07f), vec3(0.04f,0.04f,0.04f), 0.0f);
        add_box_c(tris, c+vec3(-wx,0.08f, wz), vec3(0.08f,0.08f,0.07f), vec3(0.04f,0.04f,0.04f), 0.0f);
        add_box_c(tris, c+vec3( wx,0.08f,-wz), vec3(0.08f,0.08f,0.07f), vec3(0.04f,0.04f,0.04f), 0.0f);
        add_box_c(tris, c+vec3(-wx,0.08f,-wz), vec3(0.08f,0.08f,0.07f), vec3(0.04f,0.04f,0.04f), 0.0f);
    };
    const int nCars = (int)(span / 5.0f);
    for (int i = 0; i < nCars; ++i) {
        float zc = -span*0.5f+4.0f+i*((span-8.0f)/std::max(1,nCars));
        add_car(vec3(avenue*0.4f,0,zc), hsv_to_rgb(urand(),0.6f,0.7f));
    }
    add_box_c(tris, vec3(0,0.5f,0), vec3(0.22f,0.5f,0.22f), vec3(0.13f,0.13f,0.14f), 0.02f);
    load_obj("res/models/sculpture.obj", vec3(0,1.15f,0), 0.8f, vec3(0.96f,0.78f,0.35f), 0.15f, tris, 1.0f);

    e->peds.clear();
    {
        uint32_t r = 0xABCDEFu;
        auto ur = [&]() { r=r*1664525u+1013904223u; return (r>>8)/16777216.0f; };
        const int N = 18;
        for (int i = 0; i < N; ++i) {
            float z = -CITY_SPAN*0.45f+(i+0.5f)*(CITY_SPAN*0.9f/N);
            float side = (i & 1) ? (avenue-0.4f) : -(avenue-0.4f);
            Game::Ped p;
            p.pos   = vec3(side,0,z);
            p.skin  = vec3(0.85f, 0.70f, 0.55f);
            p.shirt = hsv_to_rgb(ur(),0.65f,0.8f);
            p.phase = ur()*6.283f;
            p.speed = ((i & 1) ? +1.0f : -1.0f) * (0.6f+ur()*0.6f);
            e->peds.push_back(p);
        }
    }
    const float hs = CITY_SPAN * 0.5f + 5.f;
    add_box(tris, vec3(-hs,-0.02f,-hs), vec3(hs,0.f,hs), vec3(0.14f,0.14f,0.15f), 0.8f);
    // Stand on the clear central avenue looking toward the sculpture at origin.
    set_start_view(e, vec3(0.0f, 2.5f, CITY_SPAN * 0.4f), vec3(0.0f, 2.0f, 0.0f));
    rebuild_field_mesh(e, tris);
    e->static_bvh.build(std::move(tris), e->build_strategy);
    std::cout << "City: " << grid << "x" << grid << " grid, " << built
              << " buildings, " << e->static_bvh.triangle_count() << " tris\n";
}

static void build_pbr_test(Game* e) {
    e->peds.clear();
    std::vector<bvh::Tri> tris;
    const int   N = 4;
    const float radius = 0.45f, step = 1.4f, x0 = -(N-1)*step*0.5f;
    const int   PBR_SLICES = 16, PBR_STACKS = 12;
    vec3 gold(1.00f,0.78f,0.34f);
    for (int i = 0; i < N; ++i)
        add_sphere(tris, vec3(x0+i*step,radius,-2.0f), radius, gold, (float)i/(N-1), 1.0f, 1.5f, PBR_SLICES, PBR_STACKS, true);
    vec3 orange(0.80f,0.35f,0.10f);
    for (int i = 0; i < N; ++i)
        add_sphere(tris, vec3(x0+i*step,radius,0.0f), radius, orange, 0.3f, (float)i/(N-1), 1.5f, PBR_SLICES, PBR_STACKS, true);
    add_box(tris, vec3(-6.f,-0.02f,-6.f), vec3(6.f,0.f,6.f), vec3(0.14f,0.14f,0.15f), 0.8f);
    set_start_view(e, vec3(0.0f, 1.2f, 4.5f), vec3(0.0f, 0.45f, -1.0f));
    rebuild_field_mesh(e, tris);
    e->static_bvh.build(std::move(tris), e->build_strategy);
    std::cout << "PBR test: " << e->static_bvh.triangle_count() << " tris\n";
}

void build_scene_tris(Game* e) {
    e->rt_tris.clear();
    for (auto& [name, mp] : e->meshes) {
        const mesh& m = *mp;
        mat4 model = m.modelMatrix();
        vec3 albedo = color_from_hash(name);
        float rough = 1.0f - (float)e->reflectivity_map[name];
        for (const triangle& tri : m.faces) {
            const vertex& a = m.vertices[tri.v0];
            const vertex& b = m.vertices[tri.v1];
            const vertex& c = m.vertices[tri.v2];
            vec3 v0 = vec3(model * a.p);
            vec3 v1 = vec3(model * b.p);
            vec3 v2 = vec3(model * c.p);
            vec3 n  = normalize(cross(v1-v0, v2-v0));
            bvh::Tri t{ v0, v1, v2, n, albedo, rough };
            if (m.tex) { t.tex = m.tex; t.uv0 = a.t; t.uv1 = b.t; t.uv2 = c.t; }
            e->rt_tris.push_back(t);
        }
    }
    for (const auto& p : e->peds)
        add_humanoid(e->rt_tris, p.pos, p.skin, p.shirt, p.phase);
    e->dynamic_bvh.build(e->rt_tris, e->dynamic_build_strategy);
}

static void build_cornell_box(Game* e) {
    e->peds.clear();
    std::vector<bvh::Tri> tris;
    load_obj("res/cornell/CornellBox-Original.obj", vec3(0.0f, 1.5f, 0.0f), 3.0f,
             vec3(0.75f, 0.70f, 0.68f), 0.85f, tris);
    // Look into the open face of the box from the front.
    set_start_view(e, vec3(0.0f, 3.5f, 11.0f), vec3(0.0f, 3.5f, 0.0f));
    rebuild_field_mesh(e, tris);
    e->static_bvh.build(std::move(tris), e->build_strategy);
    std::cout << "Cornell box: " << e->static_bvh.triangle_count() << " tris\n";
}

// -----------------------------------------------------------------------------
// Character scene: a skinned OBJ mesh animated by baked skinning matrices, plus
// a camera path. The mesh is registered in e->meshes so its per-frame deformed
// triangles feed the DYNAMIC BVH (rebuilt every frame). Data comes from Blender
// exports if present (res/anim/*), otherwise a procedural fallback is generated
// so the scene still animates out of the box.
//
//   res/anim/character.obj            OBJ rigged in Blender (bind pose)
//   res/anim/character.skin.weights   binding: bones/verts + one bone per 'v'
//   res/anim/character.skin.anim      fps/bones/frames + F*B skinning matrices
//   res/anim/camera.camanim           optional camera keyframes (see loader)
// -----------------------------------------------------------------------------
static bool load_cam_anim(const std::string& path,
                          std::vector<Game::CamKey>& keys, float& fps,
                          std::string& music) {
    std::ifstream in(path);
    if (!in) return false;
    keys.clear();
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tok; ss >> tok;
        if (tok == "fps")   { ss >> fps; continue; }
        if (tok == "music") { std::getline(ss >> std::ws, music); continue; }
        // Otherwise the first token was px; parse the remaining 5 floats.
        Game::CamKey k{};
        k.pos.x = std::stof(tok);
        ss >> k.pos.y >> k.pos.z >> k.euler_deg.x >> k.euler_deg.y >> k.euler_deg.z;
        keys.push_back(k);
    }
    return !keys.empty();
}

// Euler (pitch, yaw, roll) in the engine's gameplay convention such that the
// camera at `pos` looks toward `target`. Forward maps as (sin yaw, sin pitch,
// -cos yaw), matching how movement/look use e->yaw/e->pitch.
static vec3 euler_deg_looking_at(const vec3& pos, const vec3& target) {
    vec3 d = normalize(target - pos);
    float pitch = std::asin(std::clamp(d.y, -1.0f, 1.0f));
    float yaw   = std::atan2(d.x, -d.z);
    return vec3(to_degrees(pitch), to_degrees(yaw), 0.0f);
}

static void build_character(Game* e) {
    e->peds.clear();

    // --- skinned character: Blender exports (res/data + res/robloxian) ----
    e->skin = SkinnedMesh{};
    mesh* cm = new mesh;
    bool loaded = false;
    if (load_obj_mesh("res/robloxian/robloxian.obj", *cm, 1.0f)) {
        // fps only sets each clip's real-time duration (frames/fps); apply()
        // samples by time and interpolates, so a 30 or 60 fps bake plays the
        // same. Camera + mesh are independent clips driven off one clock.
        loaded = e->skin.load("res/data/skin.weights",
                              "res/data/skin.anim", *cm);
        if (!loaded)
            std::cout << "build_character: OBJ loaded but skin weights/anim "
                         "missing or mismatched; using procedural fallback\n";
    }
    if (!loaded) {
        build_procedural_character(*cm, e->skin);
        std::cout << "build_character: procedural character ("
                  << cm->vertices.size() << " verts, " << e->skin.bones
                  << " bones, " << e->skin.frames << " frames)\n";
    } else {
        std::cout << "build_character: loaded rig (" << cm->vertices.size()
                  << " verts, " << e->skin.bones << " bones, "
                  << e->skin.frames << " frames, matched " << e->skin.matched << ")\n";
    }
    cm->setPosition(vec3(0.0f, 0.0f, 0.0f)); // OBJ already in world space
    // Pose to frame 0 so the initial (static) view and the floor line up with
    // the animation rather than with the raw bind pose.
    if (e->skin.valid()) e->skin.apply(*cm, 0.0f);
    e->skin_mesh = cm;
    e->meshes["character"] = cm;

    // Bounds of the (frame-0) deformed mesh: used to sit a ground plane under
    // the feet and to aim the procedural orbit fallback.
    vec3 lo( 1e9f,  1e9f,  1e9f), hi(-1e9f, -1e9f, -1e9f);
    for (const auto& v : cm->vertices) {
        lo.x = std::min(lo.x, v.p.x); lo.y = std::min(lo.y, v.p.y); lo.z = std::min(lo.z, v.p.z);
        hi.x = std::max(hi.x, v.p.x); hi.y = std::max(hi.y, v.p.y); hi.z = std::max(hi.z, v.p.z);
    }
    if (lo.x > hi.x) { lo = vec3(-1, 0, -1); hi = vec3(1, 2, 1); }
    vec3 center((lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f, (lo.z + hi.z) * 0.5f);

    // --- static environment: a large ground plane under the character -----
    std::vector<bvh::Tri> tris;
    // const float G = 40.0f;
    // add_box(tris, vec3(-G, lo.y - 0.05f, -G), vec3(G, lo.y, G),
    //         vec3(0.16f, 0.16f, 0.18f), 0.85f);
    load_obj("res/mall/mall.obj", vec3(0, 0, 0), 1.0f, vec3(0.8f, 0.8f, 0.8f), 0.5f, tris);
    rebuild_field_mesh(e, tris);
    e->static_bvh.build(std::move(tris), e->build_strategy);
    //e->cam.setFov(40.0f);


    // --- camera path: Blender export, else a generated orbit --------------
    e->cam_anim_music.clear();
    e->cam_anim_fps = 30.0f;
    if (!load_cam_anim("res/data/cam_anim_2.txt", e->cam_keys, e->cam_anim_fps, e->cam_anim_music)) {
        e->cam_keys.clear();
        const int   K = 8;
        const float R = std::max(hi.x - lo.x, hi.z - lo.z) * 1.6f + 3.0f;
        for (int i = 0; i < K; ++i) {
            float a = (float)i / K * 6.2831853f;
            vec3 p(center.x + std::cos(a) * R, center.y + (hi.y - lo.y) * 0.3f,
                   center.z + std::sin(a) * R);
            e->cam_keys.push_back({ p, euler_deg_looking_at(p, center) });
        }
    }

    // Frame the scene with the first camera keyframe (before pressing [P]).
    if (!e->cam_keys.empty()) {
        const Game::CamKey& k0 = e->cam_keys.front();
        e->position = k0.pos;
        e->pitch    = to_radians(k0.euler_deg.x);
        e->yaw      = to_radians(k0.euler_deg.y);
        e->cam.setPosition(e->position);
        e->cam.setRotation(vec3(e->pitch, e->yaw, to_radians(k0.euler_deg.z)));
    }

    e->anim_playing = false;
    e->anim_time    = 0.0f;
    std::cout << "Character scene: " << e->static_bvh.triangle_count()
              << " static tris, " << e->cam_keys.size() << " camera keys\n";
}

void rebuild_field(Game* e) {
    uint64_t t0 = SDL_GetPerformanceCounter();

    // Character scene owns a mesh in e->meshes; drop it when leaving/rebuilding
    // so other scenes don't fold a stale skinned mesh into the dynamic BVH.
    e->meshes.erase("character");
    delete e->skin_mesh;
    e->skin_mesh   = nullptr;
    e->anim_playing = false;
    e->anim_time    = 0.0f;

    if      (e->scene_id == 0) build_city(e);
    else if (e->scene_id == 1) build_field(e);
    else if (e->scene_id == 2) build_pbr_test(e);
    else if (e->scene_id == 3) build_cornell_box(e);
    else                       build_character(e);
    e->skybox_enabled = (e->scene_id != 3);
    e->static_build_ms = (SDL_GetPerformanceCounter()-t0)*1000.0/SDL_GetPerformanceFrequency();
    #ifdef WITH_EMBREE
        e->embree_static_dirty = true;
    #endif

    e->emissive_tris.clear();
    for (int i = 0; i < (int)e->static_bvh.triangle_count(); ++i) {
        const bvh::Tri& t = e->static_bvh.tri(i);
        if (t.emission.x + t.emission.y + t.emission.z > 0.0f)
            e->emissive_tris.push_back(t);
    }

    const char* sn = e->scene_id==0 ? "city" : e->scene_id==1 ? "field" : e->scene_id==2 ? "pbr_test" : e->scene_id==3 ? "cornell" : "character";
    const char* bs = e->build_strategy==bvh::SAH ? "SAH"
                   : e->build_strategy==bvh::Median ? "Median" : "Morton";
    std::cout << "  -> build " << sn << " (" << bs << "): " << e->static_build_ms << " ms\n";

    if (ocl::available()) {
        std::vector<float> nb, tf; std::vector<int> nl;
        if (!e->static_bvh.empty()) {
            bvh::BVH::GPUAtlas atlas;
            std::unordered_map<const texture*, int> texmap;
            e->static_bvh.build_atlas(atlas, texmap);
            e->static_bvh.flatten(nb, nl, tf, &texmap);
            ocl::set_room(nb.data(), nl.data(), tf.data(),
                          (int)e->static_bvh.node_count(), (int)e->static_bvh.triangle_count());
            ocl::set_room_textures(atlas.pixels.data(), (int)atlas.pixels.size(),
                                   atlas.off.data(), atlas.w.data(), atlas.h.data(),
                                   (int)atlas.off.size());
        } else {
            ocl::set_room(nullptr, nullptr, nullptr, 0, 0);
            ocl::set_room_textures(nullptr, 0, nullptr, nullptr, nullptr, 0);
        }
        ocl_upload_emissive(*e);
    }
}
