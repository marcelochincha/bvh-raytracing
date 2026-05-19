#include <game/sr_game.hpp>
#include <game/sr_ball.hpp>

#include <sr_config.hpp>
#include <unordered_map>
#include <renderer/sr_renderer.hpp>
#include <algorithm>
#include <string>
#include <cmath>
#include <vector>

const float kMouseSensitivity = 0.0025f;
const float moveSpeed = 6.0f;

// Table dimensions (X = long axis, Y = up, Z = width).
constexpr float TABLE_LENGTH = 2.74f; // X
constexpr float TABLE_HEIGHT = 0.0932f;
constexpr float TABLE_WIDTH  = 1.52f; // Z

struct Game
{
    framebuffer fb;
    camera cam;
    texture skybox_tex;
    std::array<texture, 6> skybox_faces;

    Ball* ball_entity;
    mesh* ball_outline;

    // Ball trail: ring buffer of recent positions + one reusable ghost mesh.
    static constexpr int TRAIL_LENGTH = 16;
    std::vector<vec3> trail_positions;
    mesh* trail_mesh = nullptr;

    float yaw = to_radians(90.0f), pitch = -to_radians(19.1266f);
    vec3 position = vec3(-6.34178f, 2.23645f, -0.233991f);

    // === Dynamic spherical camera (Rockstar-style) ===
    // Camera lives behind the +X player (theta_base = pi/2) and only moves
    // LEFT/RIGHT along the z-arc — phi is fixed. theta_desired is updated
    // only when a shot is taken (R-key or rally responder), using the new
    // target's z to apply the "lead-room" rule: camera moves OPPOSITE to
    // where the ball is heading, so the side the trajectory points into
    // gets more screen space.
    vec3  cam_orbit_center  = vec3(0.0f, 0.0f, 0.0f);
    float cam_r             = 6.0f;       // distance (zoom)
    float cam_phi           = 0.30f;      // elevation, rad (~17°) — fixed
    float cam_theta_base    = 1.5707963f; // pi/2 → camera on +X axis (behind player A)
    float cam_theta         = 1.5707963f; // current azimuth, damped
    float cam_theta_desired = 1.5707963f; // updated only on hit, persists otherwise
    float cam_theta_sens    = 0.60f;      // rad / m of target.z (lead-room amount)
    float cam_focus_weight  = 0.10f;      // lookAt = lerp(tableCenter, ball, weight)
    float cam_lambda        = 2.5f;       // exponential damping rate

    // Focus Y dynamics: when the ball climbs above a threshold (high lob),
    // raise the lookAt target. To avoid bouncy re-triggers on every arc,
    // the desired Y is LATCHED to its peak — it only resets when the ball
    // crosses the net plane (x sign flip). Then the latch goes back to 0
    // and the focus is free to follow a new peak.
    float cam_focus_y_current   = 0.0f;   // damped, applied to lookAt
    float cam_focus_y_latched   = 0.0f;   // peak-held desired value
    float cam_focus_y_last_x    = 0.0f;   // last ball.x for net-crossing detection
    float cam_focus_y_threshold = 1.2f;   // m above table before the focus starts rising
    float cam_focus_y_weight    = 1.0f;   // how much of the excess height to mirror
    float cam_focus_y_lambda    = 1.0f;   // damping rate (snappier than theta)

    // Dynamic FOV: when the ball drifts off the optical axis (cam can't
    // physically turn fast enough due to damping), widen the FOV so the
    // ball stays in frame. Narrows back to base when ball is centred.
    float cam_fov_base    = 46.7336f;     // resting horizontal FOV (deg)
    float cam_fov_current = 46.7336f;     // smoothed FOV
    float cam_fov_max     = 90.0f;        // hard cap
    float cam_fov_margin     = 1.5f;      // padding around the ball (1.0 = ball at edge)
    // Widening uses rate-based damping (snappy). Narrowing uses DURATION:
    // the FOV stays held wide for `hold_per_deg * excess_deg` seconds, then
    // linearly closes back to base over `return_duration`. Larger excess
    // → longer hold, so the player has time to track the ball instead of
    // seeing it for a single frame and feeling dizzy.
    float cam_fov_lambda_up       = 6.0f; // widening rate (unchanged)
    float cam_fov_hold_per_deg    = 0.10f;// seconds of hold per deg of excess over base
    float cam_fov_return_duration = 1.5f; // seconds to linearly close from max → base
    float cam_fov_hold_timer      = 0.0f; // accumulates time since last widening

    std::unordered_map<std::string, mesh*> meshes;

    Game(int width, int height) : fb(width, height) {}
};

// Pick a random point inside an axis-aligned rectangle on the XZ plane at y=0.
static vec3 random_point_xz(float x_min, float x_max, float z_min, float z_max) {
    float x = x_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (x_max - x_min)));
    float z = z_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (z_max - z_min)));
    return vec3(x, 0.0f, z);
}

Game *game_create(int width, int height)
{
    return new Game(width, height);
}

#include <sound/sr_sound.hpp>
void game_init(Game *e)
{
    SDL_SetRelativeMouseMode(SDL_TRUE);

    sound_init(global_config.audio_rate);
    sound_play_music("res/music/resurrection.ogg");
    sound_set_music_volume(1.5f);
    e->cam = camera(e->position, vec3(e->pitch, e->yaw, 0.0f), 46.7336f,
                    float(e->fb.width) / float(e->fb.height), 0.01f, 1000.0f);

    load_png_texture("res/textures/skybox3/null_plainsky512_rt.png", e->skybox_faces[0]);
    load_png_texture("res/textures/skybox3/null_plainsky512_bk.png", e->skybox_faces[1]);
    load_png_texture("res/textures/skybox3/null_plainsky512_lf.png", e->skybox_faces[3]);
    load_png_texture("res/textures/skybox3/null_plainsky512_ft.png", e->skybox_faces[2]);
    load_png_texture("res/textures/skybox3/null_plainsky512_dn.png", e->skybox_faces[5]);
    load_png_texture("res/textures/skybox3/null_plainsky512_up.png", e->skybox_faces[4]);

    // Ball
    Ball* ball_entity = new Ball;
    ball_init(ball_entity, vec3(0.0f, 1.0f, -1.5f));

    mesh* ball_mesh = new mesh;
    create_sphere(ball_mesh, ball_entity->radius, 8, 12);
    ball_entity->mesh = ball_mesh;
    e->ball_entity = ball_entity;

    // Ball outline (rendered as inside-out sphere for a cheap silhouette)
    mesh* outline_mesh = new mesh;
    create_sphere(outline_mesh, ball_entity->radius * 1.5f, 6, 10);
    outline_mesh->inverseFaces = true;
    e->ball_outline = outline_mesh;

    // Trail: one small sphere mesh reused for every ghost position.
    mesh* trail_m = new mesh;
    create_sphere(trail_m, ball_entity->radius * 0.85f, 5, 8);
    e->trail_mesh = trail_m;
    e->trail_positions.reserve(Game::TRAIL_LENGTH + 1);

    // Floor
    mesh* floor_mesh = new mesh;
    create_plane(floor_mesh, 32.0f, 32.0f);
    floor_mesh->setPosition(vec3(0.0f, -0.76f, 0.0f));
    e->meshes["est"] = floor_mesh;

    // Table: X is the long axis (2.74m), Z is the short axis (1.52m).
    mesh* table_mesh = new mesh;
    create_cube(table_mesh, 1.0f);
    table_mesh->setPosition(vec3(0.0f, -TABLE_HEIGHT * 0.5f, 0.0f));
    table_mesh->setScale(vec3(TABLE_LENGTH, TABLE_HEIGHT, TABLE_WIDTH));
    e->meshes["MAIN_TABLE"] = table_mesh;

    // Net (currently disabled — uncomment to enable). The net spans the
    // table's WIDTH (Z axis), so it's thin in X and wide in Z.
    mesh* net_mesh = new mesh;
    create_cube(net_mesh, 1.0f);
    net_mesh->setPosition(vec3(0.0f, 0.076f + 0.025f, 0.0f));
    net_mesh->setScale(vec3(0.02f, 0.1525f, TABLE_WIDTH + 0.305f));
    e->meshes["NET_TABLE"] = net_mesh;

    mesh* target = new mesh;
    create_cube(target, 0.1f);
    e->meshes["TARGET"] = target;



    mesh* player = new mesh;
    create_cube(player, 0.2f);
    player->setPosition(vec3(2.5f, 0.0f, 0.0f));
    player->setScale(vec3(0.25f, 1.0f, 0.25f) * 10);

    //e->meshes["PLAYER"] = player;
}

void game_update(Game *e, float dt)
{
    const Uint8 *keys = SDL_GetKeyboardState(NULL);

    // R: pick a random landing target on the OPPONENT'S side of the table
    // (negative-X half), pick a random launch position behind the player's
    // side, and use Solver #1 to compute the launch velocity.
    if (keys[SDL_SCANCODE_R]) {
        // Target zone: opponent half (x in [-1.37, -0.1]), full width in Z.
        const float x_min = -TABLE_LENGTH * 0.5f * 0.9f;
        const float x_max = -0.1f;
        const float z_min = -TABLE_WIDTH * 0.5f * 0.9f;
        const float z_max =  TABLE_WIDTH * 0.5f * 0.9f;

        vec3 random_target = random_point_xz(x_min, x_max, z_min, z_max) + vec3(0.0f, 0.0f, 0.0f);

        // Initial position: behind the player's side (positive X), above table height.
        float init_y = 0.1f + (rand() % 150) / 100.0f;
        vec3 random_init = random_point_xz(1.5f, 2.5f, -0.20f, 0.20f) + vec3(0.0f, init_y, 0.0f);

        ball_init(e->ball_entity, random_init);
        e->trail_positions.clear();

        // Search for the minimum launch speed that clears the net.
        BallLaunchSolution sol = solve_launch_velocity_min_speed(random_init, random_target);
        // `sol.vel` is the low arc if it clears, else the high arc.
        e->ball_entity->vel = sol.vel;

        e->meshes["TARGET"]->setPosition(random_target);

        // Camera reacts to the new target: turns AWAY from where the ball is
        // heading so the destination side gets more screen real-estate.
        e->cam_theta_desired = e->cam_theta_base + random_target.z * e->cam_theta_sens;
    }

    if (keys[SDL_SCANCODE_G]) dt *= 0.2f;

    ball_update(e->ball_entity, e->meshes, dt);

    // === Prototype rally: two responders, one per side. ===
    // Each fires when the ball passes its racket plane moving outward, and
    // returns the ball to a random point on the opposite half of the table.
    {
        constexpr float RACKET_X     = TABLE_LENGTH * 0.5f + 0.1f; // ~1.5 
        constexpr float RACKET_Y_MAX = 0.15f; // only respond if ball is below this height (lets high lobs pass without triggering a hit, for more interesting rallies and less camera spazz)
        Ball* ball = e->ball_entity;

        auto return_to_opposite = [&](float tx_min, float tx_max) {
            const float tz_min = -TABLE_WIDTH * 0.5f * 0.9f;
            const float tz_max =  TABLE_WIDTH * 0.5f * 0.9f;
            vec3 target = random_point_xz(tx_min, tx_max, tz_min, tz_max);
            // Bisect for minimum launch speed that clears the net.
            BallLaunchSolution sol = solve_launch_velocity_min_speed(ball->pos, target);
            ball->vel = sol.vel;
            e->meshes["TARGET"]->setPosition(target);
            // Camera reacts: turn opposite to target.z for lead-room framing.
            e->cam_theta_desired = e->cam_theta_base + target.z * e->cam_theta_sens;
        };

        // Player A (+X side): when ball reaches racket moving outward (+X), return to -X half.
        if (ball->pos.x > +RACKET_X && ball->vel.x > 0.0f && ball->pos.y <= RACKET_Y_MAX) {
            return_to_opposite(-TABLE_LENGTH * 0.5f * 0.9f, -0.1f);
        }
        // Player B (-X side): when ball reaches racket moving outward (-X), return to +X half.
        if (ball->pos.x < -RACKET_X && ball->vel.x < 0.0f  && ball->pos.y <= RACKET_Y_MAX) {
            return_to_opposite(+0.1f, +TABLE_LENGTH * 0.5f * 0.9f);
        }
    }

    e->ball_outline->setPosition(e->ball_entity->pos);

    // Trail sampling: push current ball position, drop oldest if full.
    e->trail_positions.push_back(e->ball_entity->pos);
    if ((int)e->trail_positions.size() > Game::TRAIL_LENGTH)
        e->trail_positions.erase(e->trail_positions.begin());

    #ifndef DEBUG_CAMERA
    // === Dynamic spherical camera (per-frame: damp + place + look) ===
    // theta_desired is set externally (on hit). Here we only interpolate
    // toward it, convert spherical->cartesian, and aim at the focus point.

    // 1. Exponential damping (theta never teleports — always interpolates).
    e->cam_theta += (e->cam_theta_desired - e->cam_theta) * e->cam_lambda * dt;

    // 2. Spherical -> cartesian around the orbit center. phi is fixed
    //    so the camera moves only on the horizontal arc (left/right).
    float cos_phi = std::cos(e->cam_phi);
    vec3 cam_pos = e->cam_orbit_center + vec3(
        e->cam_r * std::sin(e->cam_theta) * cos_phi,
        e->cam_r * std::sin(e->cam_phi),
        e->cam_r * std::cos(e->cam_theta) * cos_phi
    );

    // 3. Focus point: blend between table center and ball.x — lets the
    //    lookAt orient the cam toward the ongoing action even when the
    //    cam position itself stays put between hits. Y rises (damped)
    //    once the ball climbs past `cam_focus_y_threshold` so high lobs
    //    don't disappear above the frame.
    float ball_y = e->ball_entity->pos.y;
    float ball_x = e->ball_entity->pos.x;

    // Net-crossing detection: sign flip of ball.x means the ball just went
    // through the net plane → release the latch so it can re-arm.
    bool crossed_net = (e->cam_focus_y_last_x * ball_x) < 0.0f;
    e->cam_focus_y_last_x = ball_x;
    if (crossed_net) e->cam_focus_y_latched = 0.0f;

    // Arm/latch the elevated focus when the ball goes over threshold.
    if (ball_y >= e->cam_focus_y_threshold)
        e->cam_focus_y_latched = e->cam_focus_y_threshold * e->cam_focus_y_weight;

    e->cam_focus_y_current += (e->cam_focus_y_latched - e->cam_focus_y_current) * e->cam_focus_y_lambda * dt;

    vec3 focus_point = vec3(
        e->ball_entity->pos.x * e->cam_focus_weight,
        e->cam_focus_y_current,
        0.0f
    );

    e->cam.setPosition(cam_pos);
    e->cam.lookAt(focus_point);

    // 4. Dynamic FOV: if the ball is angularly farther from the camera's
    //    optical axis (= focus direction) than the current half-FOV, widen
    //    the FOV until it fits with `cam_fov_margin` padding. This rescues
    //    frames where the camera is still damping toward the new theta and
    //    the ball would otherwise be off-screen.
    {
        vec3 forward = focus_point - cam_pos;
        vec3 to_ball = e->ball_entity->pos - cam_pos;
        float fl = magnitude(forward);
        float bl = magnitude(to_ball);
        if (fl > 1e-4f && bl > 1e-4f) {
            float cos_a = dot(forward, to_ball) / (fl * bl);
            if (cos_a >  1.0f) cos_a =  1.0f;
            if (cos_a < -1.0f) cos_a = -1.0f;
            float ang_deg = std::acos(cos_a) * (180.0f / 3.14159265f);

            // FOV needed to fit the ball with margin (margin>1 = padding).
            float fov_needed  = 2.0f * ang_deg * e->cam_fov_margin;
            float fov_desired = std::max(e->cam_fov_base, std::min(e->cam_fov_max, fov_needed));
            float fov_delta   = fov_desired - e->cam_fov_current;

            if (fov_delta > 0.0f) {
                // Widening: snappy rate-based damping, reset hold timer.
                e->cam_fov_current += fov_delta * e->cam_fov_lambda_up * dt;
                e->cam_fov_hold_timer = 0.0f;
            } else {
                // Narrowing path: HOLD the wide FOV for a duration that
                // scales with current excess, then close linearly.
                e->cam_fov_hold_timer += dt;
                float excess         = e->cam_fov_current - e->cam_fov_base;
                float hold_duration  = excess * e->cam_fov_hold_per_deg;
                if (e->cam_fov_hold_timer >= hold_duration) {
                    float rate = (e->cam_fov_max - e->cam_fov_base) / e->cam_fov_return_duration;
                    e->cam_fov_current = std::max(fov_desired,
                                                  e->cam_fov_current - rate * dt);
                }
            }
            e->cam.setFov(e->cam_fov_current);
        }
    }
    #endif

    #ifdef DEBUG_CAMERA
        int mdx = 0, mdy = 0;
        SDL_GetRelativeMouseState(&mdx, &mdy);

        float d_yaw   =  float(mdx) * kMouseSensitivity;
        float d_pitch = -float(mdy) * kMouseSensitivity;

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
        e->yaw   += d_yaw;
        e->pitch += d_pitch;
        e->pitch = std::clamp(e->pitch, to_radians(-85.0f), to_radians(85.0f));
        e->cam.setPosition(e->position);
        e->cam.setRotation(vec3(e->pitch, e->yaw, 0.0f));
    #endif
}

void game_render(Game *e, SDL_Texture *sdl_fb_texture, float dt)
{
    e->fb.clear(0xFF222222);

    std::hash<std::string> r;

    renderConfig cfg;
    for (auto &[name, mesh_ptr] : e->meshes)
    {
        cfg.baseColor = (uint32_t)r(name);
        render_mesh(e->fb, e->cam, *mesh_ptr, cfg);
    }

    // Ball trail: render past positions as ghost balls (fading shade = fake
    // transparency, since the renderer doesn't blend), then connect each
    // consecutive pair with a depth-aware pixel-by-pixel 3D segment so the
    // trail reads as a single continuous 3D object.
    auto trail_shade = [&](int i, int n) -> uint32_t {
        float t = (float)(i + 1) / (float)n;        // 1 = newest, ~0 = oldest
        uint8_t a = (uint8_t)(t * 0xC0);            // alpha (engine-ignored for now)
        uint8_t s = (uint8_t)(t * 0xFF);            // RGB shade — fake fade
        return ((uint32_t)a << 24) | ((uint32_t)s << 16) | ((uint32_t)s << 8) | (uint32_t)s;
    };

    renderConfig trail_cfg;
    trail_cfg.ignoreLight = true;
    int trail_n = (int)e->trail_positions.size();
    for (int i = 0; i < trail_n; ++i) {
        trail_cfg.baseColor = trail_shade(i, trail_n);
        e->trail_mesh->setPosition(e->trail_positions[i]);
        render_mesh(e->fb, e->cam, *e->trail_mesh, trail_cfg);
    }
    // Pixel-by-pixel 3D segments between consecutive ghosts.
    for (int i = 0; i + 1 < trail_n; ++i) {
        draw_segment_3d(e->fb, e->cam,
                        e->trail_positions[i], e->trail_positions[i + 1],
                        trail_shade(i, trail_n), trail_shade(i + 1, trail_n));
    }

    renderConfig ball_cfg;
    ball_cfg.baseColor = 0xFFFFFFFF;
    ball_cfg.ignoreLight = true;
    render_mesh(e->fb, e->cam, *e->ball_entity->mesh, ball_cfg);

    renderConfig outline_cfg;
    outline_cfg.baseColor = 0x88000000;
    outline_cfg.ignoreLight = true;
    render_mesh(e->fb, e->cam, *e->ball_outline, outline_cfg);

    render_skybox(e->fb, e->cam, e->skybox_faces);
    render_gizmo(e->fb, e->cam, vec3{}, 1.0f);

    char HUD[512] = {0};
    snprintf(HUD, sizeof(HUD),
        "=== SR-LEC ===\n"
        "Frametime: %.2fms\n"
        "CAMROT: %.2f %.2f %.2f\n"
        "CAMPOS %.2f %.2f %.2f\n"
        "BALL POS: %+.2f %+.2f %+.2f\n"
        "BALL VEL: %+.2f %+.2f %+.2f\n"
        "BALL OMG: %+.2f %+.2f %+.2f\n"
        "BALL SPEED: %.4f\n",
        dt * 1000.0f,
        e->cam._rotation.x, e->cam._rotation.y, e->cam._rotation.z,
        e->cam._position.x, e->cam._position.y, e->cam._position.z,
        e->ball_entity->pos.x, e->ball_entity->pos.y, e->ball_entity->pos.z,
        e->ball_entity->vel.x, e->ball_entity->vel.y, e->ball_entity->vel.z,
        e->ball_entity->angular_vel.x, e->ball_entity->angular_vel.y, e->ball_entity->angular_vel.z,
        magnitude(e->ball_entity->vel)
    );

    draw_text(e->fb, 22, 22, HUD, 0x88000000, 0x88000000);
    draw_text(e->fb, 20, 20, HUD, 0xFFFFFFFF);
    SDL_UpdateTexture(sdl_fb_texture, NULL, e->fb.colorBuffer, e->fb.width * sizeof(uint32_t));
}

void game_handle_events(Game *e, SDL_Event &event, bool &running)
{
    if (event.type == SDL_QUIT) running = false;
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) running = false;
}
