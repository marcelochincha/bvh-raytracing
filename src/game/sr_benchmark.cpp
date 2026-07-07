#include <game/sr_benchmark.hpp>
#include <game/sr_raytrace.hpp>
#include <game/sr_scene.hpp>
#include <renderer/embree_bvh.hpp>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <vector>

struct FrameMeas { double render_ms; long nodes; };

static FrameMeas bench_frame(Game* e) {
    build_scene_tris(e);
    e->cam.rotation();
    uint64_t t0 = SDL_GetPerformanceCounter();
    for (int i = 0; i < e->num_workers; ++i) SDL_SemPost(e->start_sems[i]);
    for (int i = 0; i < e->num_workers; ++i) SDL_SemWait(e->done_sem);
    long nodes = 0;
    for (int i = 0; i < e->num_workers; ++i) nodes += e->worker_args[i].visits;
    double ms = (SDL_GetPerformanceCounter()-t0)*1000.0/SDL_GetPerformanceFrequency();
    return { ms, nodes };
}

static double traversal_only(Game* e) {
    e->cam.rotation();
    const camera& cam = e->cam;
    int W = e->fb.width, H = e->fb.height;
    uint64_t t0 = SDL_GetPerformanceCounter();
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            bvh::Hit h;
            e->static_bvh.intersect(cam._position, get_ray_direction(cam, x, y, W, H), h);
        }
    return (SDL_GetPerformanceCounter()-t0)*1000.0/SDL_GetPerformanceFrequency();
}

void run_benchmark(Game* e) {
    e->raytrace_mode = true;
    e->use_bvh       = true;
    const char* sn[] = { "SAH", "Median", "Morton" };
    const char* dn[] = { "Small", "Medium", "Large" };
    const int WARMUP = 3, MEASURE = 10, TRAV = 5;

    std::ofstream csv("docs/benchmark.csv");
    csv << "strategy,density,tris,node_count,build_ms,render_ms,traversal_ms,nodes_per_ray\n";
    std::cout << "\n=========== BENCHMARK (warmup " << WARMUP << " + avg " << MEASURE << " frames) ===========\n";

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
            double trav = 0;
            for (int i = 0; i < TRAV; ++i) trav += traversal_only(e);
            trav /= TRAV;
            size_t tris = e->static_bvh.triangle_count();
            size_t nn   = e->static_bvh.node_count();
            std::cout << sn[s] << "\t" << dn[d]
                      << "\ttris=" << tris << "\tnodes=" << nn
                      << "\tbuild=" << e->static_build_ms << "ms"
                      << "\trender=" << ravg << "ms"
                      << "\ttrav=" << trav << "ms"
                      << "\tn/ray=" << npr << "\n";
            csv << sn[s] << "," << dn[d] << "," << tris << "," << nn << ","
                << e->static_build_ms << "," << ravg << "," << trav << "," << npr << "\n";
        }

#ifdef WITH_EMBREE
    std::cout << "--- Embree (external reference) ---\n";
    e->cam.rotation();
    const camera& cam = e->cam;
    int W = e->fb.width, H = e->fb.height;
    for (int d = 0; d < 3; ++d) {
        e->density = d; e->build_strategy = bvh::SAH;
        rebuild_field(e);
        std::vector<bvh::Tri> tris(e->static_bvh.triangle_count());
        for (size_t i = 0; i < tris.size(); ++i) tris[i] = e->static_bvh.tri(i);
        embree_ref::Scene esc;
        uint64_t tb = SDL_GetPerformanceCounter();
        esc.build(tris, e->build_strategy);
        double ebuild = (SDL_GetPerformanceCounter()-tb)*1000.0/SDL_GetPerformanceFrequency();
        auto embree_trav = [&]() {
            uint64_t t0 = SDL_GetPerformanceCounter();
            for (int y = 0; y < H; ++y)
                for (int x = 0; x < W; ++x) {
                    float t; unsigned prim;
                    esc.intersect(cam._position, get_ray_direction(cam,x,y,W,H), t, prim);
                }
            return (SDL_GetPerformanceCounter()-t0)*1000.0/SDL_GetPerformanceFrequency();
        };
        for (int i = 0; i < WARMUP; ++i) embree_trav();
        double etrav = 0;
        for (int i = 0; i < TRAV; ++i) etrav += embree_trav();
        etrav /= TRAV;
        std::cout << "Embree\t" << dn[d] << "\ttris=" << tris.size()
                  << "\tbuild=" << ebuild << "ms\ttrav=" << etrav << "ms\n";
        csv << "Embree," << dn[d] << "," << tris.size() << ",0,"
            << ebuild << ",0," << etrav << ",0\n";
    }
#else
    std::cout << "(build without WITH_EMBREE -> Embree row skipped)\n";
#endif
    std::cout << "=========== done -> docs/benchmark.csv ===========\n\n";
}

static void save_frame_bmp(const framebuffer& fb, const char* path) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return;
    int W = fb.width, H = fb.height;
    int rowSize = (W*3+3)&~3, imgSize = rowSize*H;
    unsigned char hdr[54] = {};
    hdr[0]='B'; hdr[1]='M';
    *(int*)(hdr+2)  = 54+imgSize; *(int*)(hdr+10) = 54;
    *(int*)(hdr+14) = 40;         *(int*)(hdr+18) = W; *(int*)(hdr+22) = H;
    *(short*)(hdr+26) = 1;        *(short*)(hdr+28) = 24;
    *(int*)(hdr+34) = imgSize;    *(int*)(hdr+38) = 2835; *(int*)(hdr+42) = 2835;
    f.write((char*)hdr, 54);
    std::vector<unsigned char> row(rowSize, 0);
    for (int y = H-1; y >= 0; --y) {
        for (int x = 0; x < W; ++x) {
            uint32_t p = fb.colorBuffer[y*W+x];
            row[x*3+0]=(unsigned char)p; row[x*3+1]=(unsigned char)(p>>8); row[x*3+2]=(unsigned char)(p>>16);
        }
        f.write((char*)row.data(), rowSize);
    }
}

static float demo_smooth(float a, float b, float x) {
    float t = std::clamp((x-a)/(b-a), 0.0f, 1.0f);
    return t*t*(3.0f-2.0f*t);
}

static void demo_camera(Game* e, float t) {
    float s = demo_smooth(0.15f, 0.65f, t);
    e->position = vec3(3.0f*std::sin(t*3.14f), 20.0f-17.0f*s, 22.0f-44.0f*t);
    e->yaw   = 0.10f*std::sin(t*3.14f);
    e->pitch = to_radians(-25.0f+22.0f*s);
    e->cam.setPosition(e->position);
    e->cam.setRotation(vec3(e->pitch, e->yaw, 0.0f));
}

// Forward declare game_render (defined in sr_game.cpp)
void game_render(Game* e, SDL_Texture* sdl_fb_texture, float dt);

void run_demo(Game* e, SDL_Texture* tex) {
    e->raytrace_mode = true;
    e->use_bvh       = true;
    e->fly_mode      = true;
    e->show_hud      = false;
    const int N = 240; const float fps = 24.0f;
    bool running = true;
    for (int i = 0; i < N && running; ++i) {
        float t = (float)i / (N-1);
        demo_camera(e, t);
        e->show_bvh = (t > 0.60f && t < 0.78f);
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) if (ev.type == SDL_QUIT) running = false;
        game_render(e, tex, 1.0f/fps);
        char path[64];
        snprintf(path, sizeof(path), "docs/demo_%04d.bmp", i);
        save_frame_bmp(e->fb, path);
    }
    e->show_hud = true;
    std::cout << "\nDemo: " << N << " frames -> docs/demo_XXXX.bmp\n"
              << "Encode: ffmpeg -y -framerate " << (int)fps
              << " -i docs/demo_%04d.bmp -vf \"scale=trunc(iw/2)*2:trunc(ih/2)*2\""
              << " -c:v libx264 -pix_fmt yuv420p docs/demo.mp4\n\n";
}
