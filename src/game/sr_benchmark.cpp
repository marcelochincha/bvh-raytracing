#include <game/sr_benchmark.hpp>
#include <game/sr_raytrace.hpp>
#include <game/sr_scene.hpp>
#include <renderer/embree_bvh.hpp>
#include <renderer/sr_ocl.hpp>
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

// One full frame traced on the GPU, mirroring the OpenCL block in game_render():
// upload the (per-frame) dynamic BVH, hand the kernel the same camera basis and
// lighting knobs, and time the blocking render+readback. The STATIC field BVH is
// already resident on the device — rebuild_field() re-uploads it for the current
// build strategy — so this measures the GPU traversing the SAME tree the CPU and
// GPU rows share. Returns wall-clock ms for the frame.
static double gpu_bench_frame(Game* e) {
    build_scene_tris(e);                       // rebuilds the dynamic BVH
    mat4 R = e->cam.rotation();
    std::vector<float> nb, tf; std::vector<int> nl;
    e->dynamic_bvh.flatten(nb, nl, tf);
    ocl::set_dynamic(nb.data(), nl.data(), tf.data(),
                     (int)e->dynamic_bvh.node_count(), (int)e->dynamic_bvh.triangle_count());
    vec4 cx = R*vec4(1,0,0,0), cy = R*vec4(0,1,0,0), cz = R*vec4(0,0,1,0);
    float tb = tanf(to_radians(e->cam._fov)*0.5f), ta = tb/e->cam._aspectRatio;
    uint64_t t0 = SDL_GetPerformanceCounter();
    ocl::render(e->cam._position.x, e->cam._position.y, e->cam._position.z,
                cx.x,cx.y,cx.z, cy.x,cy.y,cy.z, cz.x,cz.y,cz.z,
                tb, ta, SUN_DIR.x, SUN_DIR.y, SUN_DIR.z,
                e->spp, e->skybox_enabled ? 1 : 0, e->reflections ? 1 : 0,
                e->fb.width, e->fb.height, e->fb.colorBuffer);
    return (SDL_GetPerformanceCounter()-t0)*1000.0/SDL_GetPerformanceFrequency();
}

#ifdef WITH_EMBREE
// One FULL frame rendered through the Embree backend, so it is directly comparable
// to the CPU and GPU render/frame. We build the per-frame dynamic Embree scene,
// flip the game onto the Embree path (scene_intersect then queries the Embree
// scenes) and run the SAME worker pool the CPU row uses. The only difference vs
// the CPU frame is the intersection engine — Embree instead of our BVH. The static
// Embree scene is (re)built by the caller once per (strategy, size). Timing starts
// after the dynamic build, matching bench_frame()/gpu_bench_frame().
static double embree_bench_frame(Game* e) {
    build_scene_tris(e);
    e->embree_dynamic_tris = e->rt_tris;
    e->embree_dynamic_scene.build(e->embree_dynamic_tris, e->dynamic_build_strategy);
    e->cam.rotation();
    uint64_t t0 = SDL_GetPerformanceCounter();
    for (int i = 0; i < e->num_workers; ++i) SDL_SemPost(e->start_sems[i]);
    for (int i = 0; i < e->num_workers; ++i) SDL_SemWait(e->done_sem);
    return (SDL_GetPerformanceCounter()-t0)*1000.0/SDL_GetPerformanceFrequency();
}
#endif

void run_benchmark(Game* e) {
    e->raytrace_mode = true;
    e->use_bvh       = true;
    // The density sweep (Small/Medium/Large) only means something for a scene whose
    // triangle count is driven by `density` — that's the sphere field. Pin it so the
    // benchmark is reproducible regardless of which scene was open.
    e->scene_id      = 1;
    const char* sn[] = { "SAH", "Median", "Morton" };
    const char* dn[] = { "Small", "Medium", "Large" };
    const int WARMUP = 3, MEASURE = 10, TRAV = 5;
    const bool have_gpu = ocl::available();

    std::ofstream csv("docs/benchmark.csv");
    // One row per (backend, strategy, density). All three backends now report a
    // full-frame render_ms (Custom, GPU and Embree), so they are directly
    // comparable. Columns left 0 don't apply: Embree exposes no node stats
    // (node_count/nodes_per_ray = 0, its BVH is internal); the GPU shares the CPU
    // tree so its traversal_ms = 0 (its cost lives in render_ms).
    csv << "backend,strategy,density,tris,node_count,build_ms,render_ms,traversal_ms,nodes_per_ray\n";
    std::cout << "\n=========== BENCHMARK (warmup " << WARMUP << " + avg " << MEASURE << " frames) ===========\n";
    std::cout << (have_gpu ? "GPU backend: " : "GPU backend: (unavailable)")
              << (have_gpu ? ocl::device_name() : "") << "\n";

    // ---- Backend 1 (custom CPU BVH) and Backend 2 (GPU, same tree) -------------
    // Both traverse OUR tree, so the build strategy (SAH/Median/Morton) and the
    // resulting tree quality (nodes_per_ray) are identical between them — only the
    // hardware doing the traversal differs. We build once per (strategy, density)
    // and measure both.
    for (int s = 0; s < 3; ++s)
        for (int d = 0; d < 3; ++d) {
            e->build_strategy = (bvh::BuildStrategy)s;
            e->density = d;
            e->backend = RenderBackend::Cpu;
            rebuild_field(e);   // builds the tree AND (if a GPU exists) uploads it

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
            size_t tris   = e->static_bvh.triangle_count();
            size_t nn     = e->static_bvh.node_count();
            double build  = e->static_build_ms;

            std::cout << "Custom\t" << sn[s] << "\t" << dn[d]
                      << "\ttris=" << tris << "\tnodes=" << nn
                      << "\tbuild=" << build << "ms"
                      << "\trender=" << ravg << "ms"
                      << "\ttrav=" << trav << "ms"
                      << "\tn/ray=" << npr << "\n";
            csv << "Custom," << sn[s] << "," << dn[d] << "," << tris << "," << nn << ","
                << build << "," << ravg << "," << trav << "," << npr << "\n";

            if (have_gpu) {
                e->backend = RenderBackend::OclGpu;
                for (int i = 0; i < WARMUP; ++i) gpu_bench_frame(e);
                double gsum = 0;
                for (int i = 0; i < MEASURE; ++i) gsum += gpu_bench_frame(e);
                double gavg = gsum / MEASURE;
                e->backend = RenderBackend::Cpu;
                std::cout << "GPU\t" << sn[s] << "\t" << dn[d]
                          << "\ttris=" << tris << "\tnodes=" << nn
                          << "\tbuild=" << build << "ms\trender=" << gavg << "ms"
                          << "\t(same tree, n/ray=" << npr << ")\n";
                // nodes_per_ray is a property of the tree, identical to the CPU row.
                csv << "GPU," << sn[s] << "," << dn[d] << "," << tris << "," << nn << ","
                    << build << "," << gavg << ",0," << npr << "\n";
            }

            // ---- Backend 3 (Intel Embree) — same geometry, its own BVH ----------
            // Embree builds its OWN tree at quality HIGH/MEDIUM/LOW (mapped from
            // SAH/Median/Morton). We measure both its full-frame render (worker pool
            // + Embree intersection, comparable to the Custom/GPU render above) and
            // its isolated primary-ray traversal (comparable to the Custom traversal).
#ifdef WITH_EMBREE
            {
                e->backend = RenderBackend::Embree;
                e->embree_static_tris.clear();
                for (size_t i = 0; i < e->static_bvh.triangle_count(); ++i)
                    e->embree_static_tris.push_back(e->static_bvh.tri((int)i));
                uint64_t tb = SDL_GetPerformanceCounter();
                e->embree_static_scene.build(e->embree_static_tris, (bvh::BuildStrategy)s);
                double ebuild = (SDL_GetPerformanceCounter()-tb)*1000.0/SDL_GetPerformanceFrequency();

                for (int i = 0; i < WARMUP; ++i) embree_bench_frame(e);
                double esum = 0;
                for (int i = 0; i < MEASURE; ++i) esum += embree_bench_frame(e);
                double erender = esum / MEASURE;

                e->cam.rotation();
                const camera& cam = e->cam;
                int W = e->fb.width, H = e->fb.height;
                auto etrav_pass = [&]() {
                    uint64_t t0 = SDL_GetPerformanceCounter();
                    for (int y = 0; y < H; ++y)
                        for (int x = 0; x < W; ++x) {
                            float t; unsigned prim;
                            e->embree_static_scene.intersect(cam._position, get_ray_direction(cam,x,y,W,H), t, prim);
                        }
                    return (SDL_GetPerformanceCounter()-t0)*1000.0/SDL_GetPerformanceFrequency();
                };
                for (int i = 0; i < WARMUP; ++i) etrav_pass();
                double etrav = 0;
                for (int i = 0; i < TRAV; ++i) etrav += etrav_pass();
                etrav /= TRAV;
                e->backend = RenderBackend::Cpu;

                std::cout << "Embree\t" << sn[s] << "\t" << dn[d] << "\ttris=" << tris
                          << "\tbuild=" << ebuild << "ms\trender=" << erender << "ms\ttrav=" << etrav << "ms\n";
                // Embree keeps its BVH internal: no node_count / nodes-per-ray.
                csv << "Embree," << sn[s] << "," << dn[d] << "," << tris << ",0,"
                    << ebuild << "," << erender << "," << etrav << ",0\n";
            }
#endif
        }

    // ---- Dynamic BVH sweep: the tree is REBUILT EVERY FRAME --------------------
    // The static sweep above amortizes the build over many frames, so SAH's slow
    // build is "free" and its superior tree wins. A DYNAMIC tree pays its build on
    // EVERY frame, so the honest per-frame cost is build + traversal — and there a
    // cheap builder (Morton) can beat SAH. We synthesize a scalable crowd of the
    // same humanoids the pedestrians use and sweep the three builders over it.
    // (Note: this is why the app keeps dynamic_build_strategy = Morton by default.)
    std::cout << "--- Dynamic BVH (rebuilt per frame: cost = build + traversal) ---\n";
    e->build_strategy = bvh::SAH;
    e->density = 1;
    rebuild_field(e);                    // pins the field camera view; clears peds
    const int crowd[3] = { 64, 256, 1024 };
    for (int d = 0; d < 3; ++d) {
        // Lay the crowd out on a grid on the ground in front of the camera so the
        // primary rays actually traverse it (72 tris per humanoid).
        e->peds.clear();
        int N = crowd[d];
        int side = (int)std::round(std::sqrt((float)N));
        float span = 14.0f, cell = span / side;
        uint32_t rng = 0x51EEDu;
        auto ur = [&]() { rng = rng*1664525u+1013904223u; return (rng>>8)/16777216.0f; };
        int made = 0;
        for (int z = 0; z < side && made < N; ++z)
            for (int x = 0; x < side && made < N; ++x, ++made) {
                Game::Ped p;
                p.pos   = vec3(-span*0.5f+cell*(x+0.5f), 0.0f, -span*0.5f+cell*(z+0.5f));
                p.skin  = vec3(0.85f, 0.70f, 0.55f);
                p.shirt = vec3(0.30f, 0.40f, 0.70f);
                p.phase = ur()*6.283f;
                p.speed = 0.0f;
                e->peds.push_back(p);
            }
        for (int s = 0; s < 3; ++s) {
            e->dynamic_build_strategy = (bvh::BuildStrategy)s;
            build_scene_tris(e);         // folds the crowd -> rt_tris, builds dynamic_bvh with s
            // Per-frame build cost, averaged over fresh rebuilds (build() moves in).
            double bsum = 0;
            for (int i = 0; i < MEASURE; ++i) {
                std::vector<bvh::Tri> copy = e->rt_tris;
                bvh::BVH tmp;
                uint64_t t0 = SDL_GetPerformanceCounter();
                tmp.build(std::move(copy), (bvh::BuildStrategy)s);
                bsum += (SDL_GetPerformanceCounter()-t0)*1000.0/SDL_GetPerformanceFrequency();
            }
            double dbuild = bsum / MEASURE;
            // Traversal cost + tree quality (nodes/ray) over the dynamic tree only.
            e->cam.rotation();
            const camera& cam = e->cam;
            int W = e->fb.width, H = e->fb.height;
            long rays = (long)W * H;
            double tsum = 0; long nsum = 0;
            for (int it = 0; it < TRAV; ++it) {
                bvh::reset_thread_node_visits();
                uint64_t t0 = SDL_GetPerformanceCounter();
                for (int y = 0; y < H; ++y)
                    for (int x = 0; x < W; ++x) {
                        bvh::Hit h;
                        e->dynamic_bvh.intersect(cam._position, get_ray_direction(cam,x,y,W,H), h);
                    }
                tsum += (SDL_GetPerformanceCounter()-t0)*1000.0/SDL_GetPerformanceFrequency();
                nsum += bvh::take_thread_node_visits();
            }
            double dtrav = tsum / TRAV;
            double dnpr  = rays > 0 ? (double)nsum / TRAV / rays : 0.0;
            size_t dtris = e->dynamic_bvh.triangle_count();
            size_t dnn   = e->dynamic_bvh.node_count();
            std::cout << "Dynamic\t" << sn[s] << "\t" << dn[d]
                      << "\ttris=" << dtris << "\tbuild=" << dbuild << "ms"
                      << "\ttrav=" << dtrav << "ms\ttotal=" << (dbuild+dtrav) << "ms"
                      << "\tn/ray=" << dnpr << "\n";
            // render_ms column carries the per-frame TOTAL (build+traversal), the
            // metric that matters when the tree is rebuilt every frame.
            csv << "Dynamic," << sn[s] << "," << dn[d] << "," << dtris << "," << dnn << ","
                << dbuild << "," << (dbuild+dtrav) << "," << dtrav << "," << dnpr << "\n";
        }
    }
    e->dynamic_build_strategy = bvh::Morton;  // restore the app default

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
