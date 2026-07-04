#include <game/sr_game.hpp>
#include <game/sr_game_state.hpp>
#include <game/sr_raytrace.hpp>
#include <game/sr_scene.hpp>
#include <game/sr_hud.hpp>
#include <game/sr_benchmark.hpp>

#include <renderer/sr_ocl.hpp>
#include <sound/sr_sound.hpp>
#include <sr_config.hpp>
#include <algorithm>
#include <cmath>

static const float kMouseSensitivity = 0.0025f;
static const float moveSpeed         = 6.0f;

static const char* backend_name(RenderBackend b) {
    switch (b) {
        case RenderBackend::Cpu:    return "CPU SOFTWARE";
        case RenderBackend::OclGpu: return "OCL GPU";
        case RenderBackend::Embree: return "EMBREE";
        default:                    return "UNKNOWN";
    }
}

static bool backend_available(const Game* e, RenderBackend b) {
    switch (b) {
        case RenderBackend::Cpu:
            return true;
        case RenderBackend::OclGpu:
            return ocl::available();
        case RenderBackend::Embree:
#ifdef WITH_EMBREE
            return true;
#else
            (void)e;
            return false;
#endif
    }
    (void)e;
    return false;
}

static void cycle_backend(Game* e, int dir) {
    const int kCount = 3;
    int cur = (int)e->backend;
    for (int step = 0; step < kCount; ++step) {
        cur = (cur + (dir < 0 ? kCount - 1 : 1)) % kCount;
        RenderBackend next = (RenderBackend)cur;
        if (backend_available(e, next)) {
            e->backend = next;
            return;
        }
    }
    e->backend = RenderBackend::Cpu;
}

Game* game_create(int width, int height) {
    return new Game(width, height);
}

void game_init(Game* e) {
    SDL_SetRelativeMouseMode(SDL_TRUE);

    e->cam = camera(e->position, vec3(e->pitch, e->yaw, 0.0f), 90.0f,
                    float(e->fb.width) / float(e->fb.height), 0.01f, 1000.0f);

    load_png_texture("res/textures/skybox3/null_plainsky512_rt.png", e->skybox_faces[0]);
    load_png_texture("res/textures/skybox3/null_plainsky512_bk.png", e->skybox_faces[1]);
    load_png_texture("res/textures/skybox3/null_plainsky512_ft.png", e->skybox_faces[2]);
    load_png_texture("res/textures/skybox3/null_plainsky512_lf.png", e->skybox_faces[3]);
    load_png_texture("res/textures/skybox3/null_plainsky512_up.png", e->skybox_faces[4]);
    load_png_texture("res/textures/skybox3/null_plainsky512_dn.png", e->skybox_faces[5]);


    #if ENABLE_FIELD
        rebuild_field(e);
    #endif
    build_scene_tris(e);

    if (ocl::init(e->max_bounces, AMBIENT, SHADOW_EPS)) {
        std::vector<float> nb, tf; std::vector<int> nl;
        if (!e->static_bvh.empty()) {
            e->static_bvh.flatten(nb, nl, tf);
            ocl::set_room(nb.data(), nl.data(), tf.data(),
                          (int)e->static_bvh.node_count(), (int)e->static_bvh.triangle_count());
        } else {
            ocl::set_room(nullptr, nullptr, nullptr, 0, 0);
        }
        std::vector<uint32_t> px; int off[6], w[6], h[6]; int cur = 0;
        for (int i = 0; i < 6; ++i) {
            const texture& t = e->skybox_faces[i];
            w[i]=t.width; h[i]=t.height; off[i]=cur;
            int n = t.width*t.height;
            for (int k = 0; k < n; ++k) px.push_back(t.data ? t.data[k] : 0u);
            cur += n;
        }
        ocl::set_sky(px.data(), (int)px.size(), off, w, h);
    }

    e->done_sem    = SDL_CreateSemaphore(0);
    e->worker_args = new thread_data[Game::NUM_WORKERS];
    for (int i = 0; i < Game::NUM_WORKERS; ++i) {
        e->start_sems[i]  = SDL_CreateSemaphore(0);
        e->worker_args[i] = thread_data{e, i};
        e->render_workers[i] = SDL_CreateThread(worker_thread, "RenderWorker", &e->worker_args[i]);
    }
}

void game_update(Game* e, float dt) {
    if (e->show_menu) return;

    const Uint8* keys = SDL_GetKeyboardState(NULL);
    int mdx = 0, mdy = 0;
    SDL_GetRelativeMouseState(&mdx, &mdy);

    e->yaw   += float(mdx) * kMouseSensitivity;
    e->pitch += -float(mdy) * kMouseSensitivity;
    e->pitch  = std::clamp(e->pitch, to_radians(-85.0f), to_radians(85.0f));

    vec3 inputDir(0.0f, 0.0f, 0.0f);
    if (keys[SDL_SCANCODE_W]) inputDir.z -= 1.0f;
    if (keys[SDL_SCANCODE_S]) inputDir.z += 1.0f;
    if (keys[SDL_SCANCODE_A]) inputDir.x -= 1.0f;
    if (keys[SDL_SCANCODE_D]) inputDir.x += 1.0f;
    if (e->fly_mode) {
        if (keys[SDL_SCANCODE_LSHIFT]) inputDir.y -= 1.0f;
        if (keys[SDL_SCANCODE_SPACE])  inputDir.y += 1.0f;
    }

    float s = sinf(e->yaw), c = cosf(e->yaw);
    vec3 moveDir;
    moveDir.x = (inputDir.x*c - inputDir.z*s) * moveSpeed;
    moveDir.z = (inputDir.x*s + inputDir.z*c) * moveSpeed;
    moveDir.y =  inputDir.y * moveSpeed;

    e->position = e->position + moveDir * dt;
    if (!e->fly_mode) e->position.y = 1.7f;

    float horizSpeed = std::sqrt(moveDir.x*moveDir.x + moveDir.z*moveDir.z);
    if (!e->fly_mode && horizSpeed > 0.001f) e->bob_phase += dt * horizSpeed * 2.0f;
    float bobAmt = e->fly_mode ? 0.0f : std::min(1.0f, horizSpeed / moveSpeed);
    float bob    = std::sin(e->bob_phase) * 0.05f * bobAmt;

    e->cam.setPosition(e->position + vec3(0.0f, bob, 0.0f));
    e->cam.setRotation(vec3(e->pitch, e->yaw, 0.0f));

    e->time += dt;
    const float lim = CITY_SPAN * 0.5f - 1.0f;
    for (auto& p : e->peds) {
        p.pos.z -= p.speed * dt;
        p.phase += std::fabs(p.speed) * dt * 4.0f;
        if      (p.pos.z < -lim) p.pos.z =  lim;
        else if (p.pos.z >  lim) p.pos.z = -lim;
    }
}

void game_render(Game* e, SDL_Texture* sdl_fb_texture, float dt) {
    uint64_t t = SDL_GetPerformanceCounter();
    double ms_build = 0, ms_core = 0;
    long node_visits = 0;

    e->fb.clear(0xFF000000);

    if (e->raytrace_mode) {
        build_scene_tris(e);
        ms_build = tick_ms(t);

        // Prime the camera's cached rotation matrix on the main thread before workers fire.
        mat4 R = e->cam.rotation();

        bool gpu = (e->backend == RenderBackend::OclGpu) && ocl::available();

        #ifdef WITH_EMBREE
                if (e->backend == RenderBackend::Embree) {
                    if (e->embree_static_dirty) {
                        e->embree_static_tris.clear();
                        for (size_t i = 0; i < e->static_bvh.triangle_count(); ++i)
                            e->embree_static_tris.push_back(e->static_bvh.tri((int)i));
                        e->embree_static_scene.build(e->embree_static_tris, e->build_strategy);
                        e->embree_static_dirty = false;
                    }
                    e->embree_dynamic_tris = e->rt_tris;
                    e->embree_dynamic_scene.build(e->embree_dynamic_tris, e->dynamic_build_strategy);
                }
        #endif

        if (gpu) {
            std::vector<float> nb, tf; std::vector<int> nl;
            e->dynamic_bvh.flatten(nb, nl, tf);
            ocl::set_dynamic(nb.data(), nl.data(), tf.data(),
                             (int)e->dynamic_bvh.node_count(), (int)e->dynamic_bvh.triangle_count());
            vec4 cx = R*vec4(1,0,0,0), cy = R*vec4(0,1,0,0), cz = R*vec4(0,0,1,0);
            float tb = tanf(to_radians(e->cam._fov)*0.5f), ta = tb/e->cam._aspectRatio;
            ocl::render(e->cam._position.x, e->cam._position.y, e->cam._position.z,
                        cx.x,cx.y,cx.z, cy.x,cy.y,cy.z, cz.x,cz.y,cz.z,
                        tb, ta, SUN_DIR.x, SUN_DIR.y, SUN_DIR.z,
                        e->fb.width, e->fb.height, e->fb.colorBuffer);
        }
        if (!gpu) {
            for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_SemPost(e->start_sems[i]);
            for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_SemWait(e->done_sem);
            for (int i = 0; i < Game::NUM_WORKERS; ++i) node_visits += e->worker_args[i].visits;
        }
        ms_core = tick_ms(t);
    } else {
        renderConfig cfg;
        render_skybox(e->fb, e->cam, e->skybox_faces);
        if (e->field_mesh) {
            cfg.baseColor = pack(vec3(0.55f, 0.55f, 0.58f));
            e->field_mesh->inverseFaces = false; render_mesh(e->fb, e->cam, *e->field_mesh, cfg);
            e->field_mesh->inverseFaces = true;  render_mesh(e->fb, e->cam, *e->field_mesh, cfg);
            e->field_mesh->inverseFaces = false;
        }
        for (auto& [name, mesh_ptr] : e->meshes) {
            cfg.baseColor = pack(color_from_hash(name));
            render_mesh(e->fb, e->cam, *mesh_ptr, cfg);
        }
        render_raster_shadows(e);
        ms_core = tick_ms(t);
    }

    render_gizmo(e->fb, e->cam, vec3{}, 1.0f);

    if (e->show_bvh) {
        if (!e->raytrace_mode) build_scene_tris(e);
        draw_bvh_debug(e);
    }
    if (e->show_normals) {
        if (!e->raytrace_mode) build_scene_tris(e);
        draw_normals_debug(e);
    }

    double ms_present = tick_ms(t);
    long   rays = (long)e->fb.width * e->fb.height;
    double nodes_per_ray = rays > 0 ? (double)node_visits / rays : 0.0;

    if (e->show_hud) {
        char backend[192];
        if (e->raytrace_mode && e->backend == RenderBackend::OclGpu && ocl::available()) {
            snprintf(backend, sizeof(backend), "%s: %s", backend_name(e->backend), ocl::device_name());
        } else if (e->raytrace_mode && e->backend == RenderBackend::OclGpu) {
            snprintf(backend, sizeof(backend), "%s (unavailable)", backend_name(e->backend));
        } else if (e->raytrace_mode && e->backend == RenderBackend::Cpu) {
            snprintf(backend, sizeof(backend), "%s (%d workers)", backend_name(e->backend), Game::NUM_WORKERS);
        } else {
            snprintf(backend, sizeof(backend), "%s", backend_name(e->backend));
        }

        if (e->hud_simple) {
            char line[192];
            snprintf(line, sizeof(line),
                "FPS: %.0f  |  %s  |  %s  |  BVH: %s  |  [H] expand",
                1.0f/dt, e->raytrace_mode ? "Raytrace" : "Raster",
                backend, e->use_bvh ? "On" : "Off");
            draw_text(e->fb, 12, 12, line, 0x88000000, 0x88000000);
            draw_text(e->fb, 10, 10, line, 0xFFFFFFFF);
        } else {
            char HUD[640] = {0};
            snprintf(HUD, sizeof(HUD),
                "=== SR-LEC (%s) ===\n"
                "Frametime: %.2fms   FPS: %.2f\n"
                "render: %.1fms  dyn-build: %.1fms  rest: %.1fms\n"
                "--- Static BVH (scene) ---\n"
                "  tris: %zu  nodes: %zu  build: %.2fms\n"
                "--- Dynamic BVH (objs+peds) ---\n"
                "  tris: %zu  nodes: %zu  nodes/ray: %.1f\n"
                "Accel: %s   Static: %s  Dyn: %s\n"
                "Backend: %s\n"
                "Move: %s\n"
                "[TAB] mode [B] BVH [V] vis \n[N] normals [M] menu [F1] bench\n[SPACE x2] walk/fly  [H] compact  [G] backend\n",
                e->raytrace_mode ? "RAYTRACE" : "RASTER",
                dt*1000.0f, 1.0f/dt, ms_core, ms_build, ms_present,
                e->static_bvh.triangle_count(), e->static_bvh.node_count(), e->static_build_ms,
                e->dynamic_bvh.triangle_count(), e->dynamic_bvh.node_count(), nodes_per_ray,
                e->use_bvh ? "BVH (fast)" : "BRUTE FORCE (slow)",
                e->build_strategy==bvh::SAH ? "SAH" : e->build_strategy==bvh::Median ? "Median" : "Morton",
                e->dynamic_build_strategy==bvh::SAH ? "SAH" : e->dynamic_build_strategy==bvh::Median ? "Median" : "Morton",
                backend, e->fly_mode ? "FLY" : "WALK");
            draw_text(e->fb, 22, 22, HUD, 0x88000000, 0x88000000);
            draw_text(e->fb, 20, 20, HUD, 0xFFFFFFFF);
        }
    }

    if (e->show_menu) draw_menu(e);

    SDL_UpdateTexture(sdl_fb_texture, NULL, e->fb.colorBuffer, e->fb.width * sizeof(uint32_t));
}

void game_handle_events(Game* e, SDL_Event& event, bool& running) {
    if (event.type == SDL_QUIT) { running = false; return; }
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
        if (e->show_menu) { e->show_menu = false; SDL_SetRelativeMouseMode(SDL_TRUE); }
        else running = false;
        return;
    }
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_m) {
        e->show_menu = !e->show_menu;
        SDL_SetRelativeMouseMode(e->show_menu ? SDL_FALSE : SDL_TRUE);
        return;
    }
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_F1) {
        run_benchmark(e); return;
    }
    if (e->show_menu && event.type == SDL_KEYDOWN) {
        SDL_Keycode k = event.key.keysym.sym;
        if      (k == SDLK_UP)                  { e->menu_cursor = (e->menu_cursor-1+11)%11; }
        else if (k == SDLK_DOWN)                 { e->menu_cursor = (e->menu_cursor+1)%11;   }
        else if (k == SDLK_LEFT)                 { menu_apply(e, -1); }
        else if (k == SDLK_RIGHT || k==SDLK_RETURN) { menu_apply(e, +1); }
        return;
    }
    if (event.type == SDL_KEYDOWN) {
        switch (event.key.keysym.sym) {
            case SDLK_TAB: e->raytrace_mode = !e->raytrace_mode; break;
            case SDLK_b:   e->use_bvh       = !e->use_bvh;       break;
            case SDLK_v:   e->show_bvh      = !e->show_bvh;      break;
            case SDLK_h:   e->hud_simple    = !e->hud_simple;    break;
            case SDLK_n:   e->show_normals  = !e->show_normals;  break;
            case SDLK_g:   cycle_backend(e, +1); break;
            default: break;
        }
    }
}

void game_shutdown(Game* e) {
    e->workers_running = false;
    for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_SemPost(e->start_sems[i]);
    for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_WaitThread(e->render_workers[i], NULL);
    for (int i = 0; i < Game::NUM_WORKERS; ++i) SDL_DestroySemaphore(e->start_sems[i]);
    SDL_DestroySemaphore(e->done_sem);
    delete[] e->worker_args;
    delete e->field_mesh;
}
