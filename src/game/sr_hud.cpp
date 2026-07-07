#include <game/sr_hud.hpp>
#include <game/sr_raytrace.hpp>
#include <game/sr_scene.hpp>
#include <renderer/sr_ocl.hpp>
#include <algorithm>
#include <cstdio>

double tick_ms(uint64_t& since) {
    uint64_t now = SDL_GetPerformanceCounter();
    double ms = (now - since) * 1000.0 / SDL_GetPerformanceFrequency();
    since = now;
    return ms;
}

void render_raster_shadows(Game* e) {
    const vec3  L       = SUN_DIR;
    const float plane_y = 0.02f;
    mat4 S(1.0f);
    S(0,1) = -L.x/L.y;  S(1,1) = 0.0f;  S(2,1) = -L.z/L.y;
    S(0,3) = (L.x/L.y)*plane_y;
    S(1,3) = plane_y;
    S(2,3) = (L.z/L.y)*plane_y;

    renderConfig scfg;
    scfg.baseColor   = 0xFF1A1A1A;
    scfg.ignoreLight = true;

    mesh tmp;
    for (auto& [name, mp] : e->meshes) {
        if (name == "FLOOR") continue;
        const mesh& m = *mp;
        mat4 MS = S * m.modelMatrix();
        tmp.vertices.clear();
        tmp.vertices.reserve(m.vertices.size());
        for (const vertex& v : m.vertices) {
            vec4 w = MS * v.p;
            tmp.vertices.push_back({ vec3(w.x,w.y,w.z), v.t });
        }
        tmp.faces              = m.faces;
        tmp._modelMatrixDirty  = true;
        tmp.inverseFaces = false; render_mesh(e->fb, e->cam, tmp, scfg);
        tmp.inverseFaces = true;  render_mesh(e->fb, e->cam, tmp, scfg);
    }
}

static void draw_aabb_wire(framebuffer& fb, const camera& cam, const AABB& b, uint32_t color) {
    vec3 c[8] = {
        {b.min.x,b.min.y,b.min.z},{b.max.x,b.min.y,b.min.z},
        {b.max.x,b.min.y,b.max.z},{b.min.x,b.min.y,b.max.z},
        {b.min.x,b.max.y,b.min.z},{b.max.x,b.max.y,b.min.z},
        {b.max.x,b.max.y,b.max.z},{b.min.x,b.max.y,b.max.z},
    };
    static const int edges[12][2] = {
        {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}
    };
    for (auto& ed : edges) draw_gizmo_line(fb, cam, c[ed[0]], c[ed[1]], color);
}

void draw_bvh_debug(Game* e) {
    static std::vector<bvh::BVH::DebugNode> nodes;
    static const uint32_t palette[] = {
        0xFFFF4040,0xFFFFA040,0xFFFFFF40,0xFF40FF40,
        0xFF40FFFF,0xFF4080FF,0xFFC040FF,0xFFFF40C0,
    };
    auto draw_bvh = [&](const bvh::BVH& b) {
        b.debug_nodes(nodes);
        for (const auto& n : nodes) {
            uint32_t col = palette[n.depth % (sizeof(palette)/sizeof(palette[0]))];
            draw_aabb_wire(e->fb, e->cam, n.bounds, col);
        }
    };
    draw_bvh(e->static_bvh);
    draw_bvh(e->dynamic_bvh);
}

void draw_ray_debug(Game* e) {
    if (!e->ray_debug.active) return;

    // 1) Every box the ray tested along its path. Green = the box was hit, so
    //    traversal DESCENDED into it; red = the box was missed, so that whole
    //    subtree was PRUNED (this is what the demo is meant to show).
    for (const auto& v : e->ray_debug.visited) {
        uint32_t col = v.box_hit ? 0xFF30FF30 : 0xFF803030;
        draw_aabb_wire(e->fb, e->cam, v.bounds, col);
    }

    // 2) The ray's bounce path on top, bright yellow, with a small marker box at
    //    each hit point so the bounce vertices stand out.
    const auto& p = e->ray_debug.path;
    for (std::size_t i = 1; i < p.size(); ++i)
        draw_gizmo_line(e->fb, e->cam, p[i - 1], p[i], 0xFFFFFF00);
    for (std::size_t i = 1; i + 1 < p.size(); ++i) {
        const float s = 0.03f;
        AABB m{ p[i] - vec3(s, s, s), p[i] + vec3(s, s, s) };
        draw_aabb_wire(e->fb, e->cam, m, 0xFFFFFF00);
    }
}

void draw_normals_debug(Game* e) {
    const vec3  cam_pos   = e->cam._position;
    const float scale     = 0.3f;
    const float threshold = 0.5f;

    auto draw_vn = [&](const vec3& pos, const vec3& n) {
        if (dot(n, normalize(pos - cam_pos)) >= threshold) return;
        draw_gizmo_line(e->fb, e->cam, pos, pos + n * scale, 0xFF00FF88);
    };
    auto draw_tri_vn = [&](const bvh::Tri& t) {
        if (t.smooth) { draw_vn(t.v0,t.n0); draw_vn(t.v1,t.n1); draw_vn(t.v2,t.n2); }
        else          { draw_vn(t.v0,t.normal); draw_vn(t.v1,t.normal); draw_vn(t.v2,t.normal); }
    };
    for (const auto& t : e->rt_tris) draw_tri_vn(t);
    for (std::size_t i = 0; i < e->static_bvh.triangle_count(); ++i)
        draw_tri_vn(e->static_bvh.tri((int)i));
}

static const int MENU_ITEMS = 11;

static const char* menu_label(int i) {
    static const char* L[MENU_ITEMS] = {
        "Render mode","Backend","Acceleration","Show BVH","Scene",
        "Density","Reflections","Ray bounces","Static build","Dyn build","Samples"
    };
    return L[i];
}

static const char* menu_value(const Game* e, int i) {
    switch (i) {
        case 0: return e->raytrace_mode ? "Raytrace"   : "Raster";
        case 1:
            switch (e->backend) {
                case RenderBackend::Cpu:    return "CPU SOFTWARE";
                case RenderBackend::OclGpu: return "OCL GPU";
                case RenderBackend::Embree: return "EMBREE";
            }
            return "CPU SOFTWARE";
        case 2: return e->use_bvh       ? "BVH"        : "Brute force";
        case 3: return e->show_bvh      ? "On"         : "Off";
        case 4: return e->scene_id==0 ? "City" : e->scene_id==1 ? "Spheres" : e->scene_id==2 ? "PBR Test" : e->scene_id==3 ? "Cornell Box" : "Character";
        case 5: return e->density==0    ? "Small"      : (e->density==1 ? "Medium" : "Large");
        case 6: return e->reflections   ? "On"         : "Off";
        case 7: { static char b[8]; snprintf(b,sizeof(b),"%d",e->max_bounces); return b; }
        case 8: return e->build_strategy==bvh::SAH    ? "SAH"
                     : e->build_strategy==bvh::Median ? "Median" : "Morton";
        case 9: return e->dynamic_build_strategy==bvh::SAH    ? "SAH"
                     : e->dynamic_build_strategy==bvh::Median ? "Median" : "Morton";
        default: { static char b[8]; snprintf(b,sizeof(b),"%d",e->spp); return b; }
    }
}

void menu_apply(Game* e, int dir) {
    auto backend_ok = [&](RenderBackend b) {
        if (b == RenderBackend::Cpu) return true;
        if (b == RenderBackend::OclGpu) return ocl::available();
        #ifdef WITH_EMBREE
                if (b == RenderBackend::Embree) return true;
        #endif
        return false;
    };

    switch (e->menu_cursor) {
        case 0: e->raytrace_mode = !e->raytrace_mode; break;
        case 1: {
            int cur = (int)e->backend;
            for (int step = 0; step < 3; ++step) {
                cur = (cur + (dir < 0 ? 2 : 1)) % 3;
                RenderBackend next = (RenderBackend)cur;
                if (backend_ok(next)) { e->backend = next; break; }
            }
            break;
        }
        case 2: e->use_bvh       = !e->use_bvh;       break;
        case 3: e->show_bvh      = !e->show_bvh;      break;
        case 4: e->scene_id = (e->scene_id+(dir<0?4:1))%5; rebuild_field(e); break;
        case 5: e->density  = (e->density +(dir<0?2:1))%3; rebuild_field(e); break;
        case 6: e->reflections = !e->reflections; break;
        case 7: e->max_bounces = 1+((e->max_bounces-1+(dir<0?2:1))%3); break;
        case 8: {
            int s = (int)e->build_strategy;
            s = (s+(dir<0?2:1))%3;
            e->build_strategy = (bvh::BuildStrategy)s;
            rebuild_field(e);
            break;
        }
        case 9: {
            int s = (int)e->dynamic_build_strategy;
            s = (s+(dir<0?2:1))%3;
            e->dynamic_build_strategy = (bvh::BuildStrategy)s;
            break;
        }
        case 10: {
            static const int spps[] = {1,2,4,8};
            int cur = 0;
            for (int i = 0; i < 4; ++i) if (spps[i] == e->spp) { cur = i; break; }
            cur = (cur + (dir < 0 ? 3 : 1)) % 4;
            e->spp = spps[cur];
            break;
        }
    }
}

static void fill_rect_alpha(framebuffer& fb, int x0, int y0, int x1, int y1, uint32_t col) {
    float a = (float)((col>>24)&0xFF)/255.0f;
    int cr=(col>>16)&0xFF, cg=(col>>8)&0xFF, cb=col&0xFF;
    for (int y = std::max(0,y0); y < std::min(fb.height,y1); ++y)
        for (int x = std::max(0,x0); x < std::min(fb.width,x1); ++x) {
            uint32_t d = fb.colorBuffer[y*fb.width+x];
            int dr=(d>>16)&0xFF, dg=(d>>8)&0xFF, db=d&0xFF;
            int r=(int)(cr*a+dr*(1.0f-a)), g=(int)(cg*a+dg*(1.0f-a)), b=(int)(cb*a+db*(1.0f-a));
            fb.colorBuffer[y*fb.width+x] = 0xFF000000u|(r<<16)|(g<<8)|b;
        }
}

void draw_menu(Game* e) {
    const int lh=18, pad=14, w=312;
    const int h = pad+24+pad+lh*MENU_ITEMS+pad+18;
    const int x = e->fb.width-w-12, y0=18;
    fill_rect_alpha(e->fb, x, y0, x+w, y0+h, 0xE80B0E18);
    fill_rect_alpha(e->fb, x, y0, x+w, y0+24, 0xFF1B6FB5);
    draw_text(e->fb, x+10, y0+7, "OPTIONS   [M] close", 0xFFFFFFFF);
    for (int i = 0; i < MENU_ITEMS; ++i) {
        int ry = y0+24+pad+i*lh;
        if (i == e->menu_cursor)
            fill_rect_alpha(e->fb, x+6, ry-2, x+w-6, ry+lh-3, 0x603A66B0);
        char line[96];
        snprintf(line, sizeof(line), "%-13s %s", menu_label(i), menu_value(e,i));
        draw_text(e->fb, x+14, ry, line, i==e->menu_cursor ? 0xFFFFE27A : 0xFFE8E8E8);
    }
    draw_text(e->fb, x+10, y0+h-16, "^/v select   </>  change", 0xFFA8B0BC);
}
