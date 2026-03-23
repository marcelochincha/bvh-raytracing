#include <engine/sr_engine.hpp>
#ifdef TYPE_A
#include <renderer/sr_renderer.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

// --- AJUSTES DE JUEGO ---
static constexpr float kMoveSpeed = 7.0f;
static constexpr float kGravity = -18.0f; // Gravedad más fuerte para control preciso
static constexpr float kJumpForce = 8.0f; // Fuerza del salto
static constexpr float kMouseSensitivity = 0.0025f;
static constexpr float kDeathY = -400.0f;
static const vec3 kPlayerExtents(0.4f, 1.0f, 0.4f); // Medias extensiones de la caja del jugador

struct AABB
{
    vec3 center;
    vec3 half;
};

struct Platform
{
    vec3 position;
    vec3 size;
    mesh meshObj;
    AABB aabb;
    uint32_t color;
    bool isGoal;
    bool isDeath;
};

static float axis_get(const vec3 &v, int axis)
{
    return axis == 0 ? v.x : (axis == 1 ? v.y : v.z);
}

static void axis_set(vec3 &v, int axis, float value)
{
    if (axis == 0)
        v.x = value;
    else if (axis == 1)
        v.y = value;
    else
        v.z = value;
}

static bool aabb_overlap(const AABB &a, const AABB &b)
{
    float dx = fabsf(a.center.x - b.center.x);
    float dy = fabsf(a.center.y - b.center.y);
    float dz = fabsf(a.center.z - b.center.z);
    return dx <= (a.half.x + b.half.x) && dy <= (a.half.y + b.half.y) && dz <= (a.half.z + b.half.z);
}

struct Engine
{
    framebuffer fb;
    camera cam;
    texture skybox_tex;
    mesh skybox;
    mesh map;
    mesh spray;

    texture spray_tex;

    struct Player
    {
        vec3 size;
        vec3 position;
        float yaw;
        float pitch;
        vec3 velocity;
        bool onGround; // IMPORTANTE PARA EL SALTO
    } player;

    std::vector<Platform> level;
    vec3 spawnPoint = vec3(0.0f, 10.0f, 0.0f);
    bool gameWon = false;

    Engine(int width, int height) : fb(width, height) {}

    void resetPlayer()
    {
        printf("Resetting player to spawn point\n");
        player.position = spawnPoint;
        player.size = kPlayerExtents * 2;
        player.velocity = vec3(0.0f, 0.0f, 0.0f);
        player.onGround = false;
        gameWon = false;
        if(player.position.x == spawnPoint.x && player.position.y == spawnPoint.y && player.position.z == spawnPoint.z)
            printf("Player already at spawn point, no reset needed\n");
        printf("Player reset to: %.2f %.2f %.2f\n", player.position.x, player.position.y, player.position.z);
    }

    void addPlatform(vec3 pos, vec3 scale, uint32_t col, bool goal = false, bool death = false)
    {
        mesh m = create_cube(1.0f);
        AABB aabb;
        aabb.center = pos;
        aabb.half = scale * 0.5f;
        m.setScale(scale);
        m.setPosition(pos);
        level.push_back((Platform){pos, scale, m, aabb, col, goal, death});
    }

    std::array<texture, 6> skybox_faces;
};

Engine *engine_create(int width, int height)
{
    return new Engine(width, height);
}

void engine_init(Engine *e)
{
    SDL_SetRelativeMouseMode(SDL_TRUE);

    e->cam = camera(vec3(0.0f, 1.5f, 0.0f), vec3(0.0f, 0.0f, 0.0f), 120.0f, float(e->fb.width) / float(e->fb.height), 0.01f, 1000.0f);

    load_png_texture("res/textures/skybox3/null_plainsky512_rt.png", e->skybox_faces[0]);
    load_png_texture("res/textures/skybox3/null_plainsky512_bk.png", e->skybox_faces[1]);
    load_png_texture("res/textures/skybox3/null_plainsky512_lf.png", e->skybox_faces[3]);
    load_png_texture("res/textures/skybox3/null_plainsky512_ft.png", e->skybox_faces[2]);
    load_png_texture("res/textures/skybox3/null_plainsky512_dn.png", e->skybox_faces[5]);
    load_png_texture("res/textures/skybox3/null_plainsky512_up.png", e->skybox_faces[4]);
    
    // e->map = load_ply_ascii("res/models/crossroads.ply");
    // e->map.setScale(vec3(0.5f, 0.5f, 0.5f));
    // e->map.setPosition(vec3(0.0f, 0.0f, 0.0f));
    // e->map.setRotation(vec3(0, 0.0f, 0.0f));
    // e->map.inverseFaces = true;


    e->spray = create_plane(1.0f, .75f);
    e->spray.setScale(vec3(1.0f, 1.0f, 1.0f) * 75);
    e->spray.setPosition(vec3(-12.0f, 3.0f, -2.0f));
    e->spray.setRotation(vec3(SR_PI / 2.0f, SR_PI / 2.0f, 0.0f));
    load_png_texture("res/textures/perro.png", e->spray_tex);

    // --- SITIO DE PRUEBAS PARA EL OBBY ---
    e->level.clear();

    // Suelo seguro y zona de spawn
    e->addPlatform(vec3(0, 0, 0), vec3(10, 1, 10), 0xFF444444);

    // Piscina de muerte debajo para probar reinicios
    e->addPlatform(vec3(0, -3.0f, -12.0f), vec3(30, 1, 30), 0xFFAA2200, false, true);

    // Saltos de precisión
    e->addPlatform(vec3(0, 1.2f, -8), vec3(3, 1, 3), 0xFF0AAAF0);
    e->addPlatform(vec3(4, 2.0f, -16), vec3(3, 1, 3), 0xFF0AAAF0);
    e->addPlatform(vec3(0, 2.6f, -24), vec3(3, 1, 3), 0xFFFFCC00);

    // Pasarela estrecha para probar colisión lateral
    e->addPlatform(vec3(0, 2.0f, -32), vec3(1.0f, 1, 12.0f), 0xFF66FF66);

    // Plataforma final de victoria
    e->addPlatform(vec3(0, 3.5f, -42), vec3(8, 1, 8), 0xFF00FF00, true);

    e->spawnPoint = vec3(0.0f, 3.0f, 4.0f);
    e->resetPlayer();
}

void engine_update(Engine *e, float dt)
{
    // 1. Mirar con el Mouse
    int mdx = 0, mdy = 0;
    SDL_GetRelativeMouseState(&mdx, &mdy);
    e->player.yaw += float(mdx) * kMouseSensitivity;
    e->player.pitch += float(mdy) * kMouseSensitivity;
    e->player.pitch = std::clamp(e->player.pitch, to_radians(-85.0f), to_radians(85.0f));

    // 2. Movimiento Horizontal (Teclado)
    const Uint8 *keys = SDL_GetKeyboardState(NULL);
    vec3 inputDir(0.0f, 0.0f, 0.0f);
    if (keys[SDL_SCANCODE_W])
        inputDir.z -= 1.0f;
    if (keys[SDL_SCANCODE_S])
        inputDir.z += 1.0f;
    if (keys[SDL_SCANCODE_A])
        inputDir.x -= 1.0f;
    if (keys[SDL_SCANCODE_D])
        inputDir.x += 1.0f;

    ////ADD fly mode
    // if (keys[SDL_SCANCODE_LSHIFT]) inputDir.y -= 1.0f;
    // if (keys[SDL_SCANCODE_SPACE]) inputDir.y += 1.0;

    float s = sinf(e->player.yaw);
    float c = cosf(e->player.yaw);
    vec3 moveDir;
    moveDir.x = (inputDir.x * c - inputDir.z * s) * kMoveSpeed;
    moveDir.z = (inputDir.x * s + inputDir.z * c) * kMoveSpeed;
    moveDir.y = inputDir.y * kMoveSpeed;

    // 3. LOGICA DE SALTO
    if (keys[SDL_SCANCODE_SPACE] && e->player.onGround)
    {
        e->player.velocity.y = kJumpForce;
        e->player.onGround = false; // Despegar
    }

    // 4. Gravedad
    e->player.velocity.y += kGravity * dt;

    // 5. Movimiento propuesto
    vec3 nextPos = e->player.position;
    nextPos.x += moveDir.x * dt;
    nextPos.z += moveDir.z * dt;
    nextPos.y += moveDir.y * dt + e->player.velocity.y * dt;

    // 6. Colisión AABB 3D simple
    e->player.onGround = false;
    for (const auto &plat : e->level)
    {
        AABB playerBox{nextPos, kPlayerExtents};
        if (!aabb_overlap(playerBox, plat.aabb))
            continue;

        if (plat.isDeath)
        {
            printf("Player died by touching a death platform\n");
            e->resetPlayer();
            return;
        }

        vec3 delta = nextPos - plat.aabb.center;
        float px = (playerBox.half.x + plat.aabb.half.x) - fabsf(delta.x);
        float py = (playerBox.half.y + plat.aabb.half.y) - fabsf(delta.y);
        float pz = (playerBox.half.z + plat.aabb.half.z) - fabsf(delta.z);

        // Resolver en el eje de menor penetración
        if (px <= py && px <= pz)
        {
            nextPos.x += (delta.x > 0.0f ? px : -px);
        }
        else if (py <= px && py <= pz)
        {
            nextPos.y += (delta.y > 0.0f ? py : -py);
            e->player.velocity.y = 0.0f;
            if (delta.y > 0.0f)
                e->player.onGround = true;
        }
        else
        {
            nextPos.z += (delta.z > 0.0f ? pz : -pz);
        }

        // Recalcular AABB tras movernos
        playerBox.center = nextPos;
        if (plat.isGoal)
        {
            e->gameWon = true;
            printf("Player reached the goal!\n");
        }
    }

    // 7. Muerte por caída o reinicio manual
    if (nextPos.y < kDeathY || keys[SDL_SCANCODE_R])
    {
        printf("Player died by falling or manual reset\n");
        e->resetPlayer();
    }
    else
    {
        e->player.position = nextPos;
    }

    // Actualizar Cámara
    e->cam.setPosition(e->player.position);
    e->cam.setRotation(vec3(e->player.pitch, e->player.yaw, 0.0f));
}

void engine_render(Engine *e, SDL_Texture *sdl_fb_texture, float dt)
{
    e->fb.clear(0xFF222222);

    renderConfig plat_shader;
    for (auto &plat : e->level)
    {
        plat_shader.baseColor = plat.color;
        render_mesh(e->fb, e->cam, plat.meshObj, plat_shader);
    }

    render_skybox(e->fb, e->cam, e->skybox_faces);
    render_gizmo(e->fb, e->cam, vec3{}, 10.0f);

    // UI
    char HUD[256];
    snprintf(HUD, sizeof(HUD), "SR-LEC DEBUG\nFrametime : %.4f\nPos: %.2f %.2f %.2f", dt, e->player.position.x, e->player.position.y, e->player.position.z);
    draw_text(e->fb, 22, 22, HUD, 0x88888888, 0x88888888); // Sombra
    draw_text(e->fb, 20, 20, HUD, 0xFFFFFFFF);

    if (e->gameWon)
    {
        draw_text(e->fb, e->fb.width / 2 - 60, e->fb.height / 2, "GANASTE EL OBBY ! :)", 0xFFFF0000);
    }

    SDL_UpdateTexture(sdl_fb_texture, NULL, e->fb.colorBuffer, e->fb.width * sizeof(uint32_t));
}

void engine_handle_events(Engine *e, SDL_Event &event, bool &running)
{
    if (event.type == SDL_QUIT)
        running = false;
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_DELETE)
        running = false;
}

#endif