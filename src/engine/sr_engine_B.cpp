#include <engine/sr_engine.hpp>
#ifdef TYPE_B 

#include <unordered_map>
#include <renderer/sr_renderer.hpp>
#include <algorithm>
#include <string>
#include <cmath>
#include <vector>

const float kMouseSensitivity = 0.0025f;
const float moveSpeed = 6.0f;

struct ball {
    vec3 pos;           
    vec3 vel;           
    vec3 angular_vel;    
    mesh* mesh;         
    float radius;       
    float bounce;       
    float friction;     
};

// Main game engine structure
struct Engine
{
    framebuffer fb;
    camera cam;
    texture skybox_tex;
    std::array<texture, 6> skybox_faces;

    ball* ball_entity;
    mesh* ball_outline;

    float yaw = 0, pitch = 0;
    vec3 position = vec3(0.0f, 1.5f, 5.0f);
    vec3 target = vec3(0.0f, 1.0f, 0.0f);

    std::unordered_map<std::string,mesh*> meshes;

    // Must declare constructor to initialize framebuffer
    Engine(int width, int height) : fb(width, height) {}
};

//
// BALL PHYSICS FUNCTIONS
//

struct AABB
{
    vec3 min;
    vec3 max;
};

// Compute the AABB of a mesh in world space (taking into account its model matrix)
AABB calculate_mesh_aabb(const mesh* m) {
    AABB aabb;
    
    if (m->vertices.empty()) {
        aabb.min = aabb.max = vec3(0.0f, 0.0f, 0.0f);
        return aabb;
    }
    
    mat4 model = m->modelMatrix();
    vec3 min_bounds(1e10f, 1e10f, 1e10f);
    vec3 max_bounds(-1e10f, -1e10f, -1e10f);
    
    for (const vertex& v : m->vertices) {
        vec4 world_pos = model * vec4(v.p.x, v.p.y, v.p.z, 1.0f);
        min_bounds.x = std::min(min_bounds.x, world_pos.x);
        min_bounds.y = std::min(min_bounds.y, world_pos.y);
        min_bounds.z = std::min(min_bounds.z, world_pos.z);
        
        max_bounds.x = std::max(max_bounds.x, world_pos.x);
        max_bounds.y = std::max(max_bounds.y, world_pos.y);
        max_bounds.z = std::max(max_bounds.z, world_pos.z);
    }
    
    aabb.min = min_bounds;
    aabb.max = max_bounds;
    return aabb;
}

vec3 closest_point_on_aabb(const AABB& aabb, const vec3& point) {
    vec3 closest;
    //this clamps de point inside the AABB for each axis
    closest.x = std::max(aabb.min.x, std::min(point.x, aabb.max.x));
    closest.y = std::max(aabb.min.y, std::min(point.y, aabb.max.y));
    closest.z = std::max(aabb.min.z, std::min(point.z, aabb.max.z));
    return closest;
}

// Check for collision between a sphere and an AABB, and resolve it if they collide
bool collide_sphere_aabb(ball* b, const AABB& aabb, vec3& out_normal) {
    vec3 closest = closest_point_on_aabb(aabb, b->pos);
    vec3 diff = b->pos - closest;
    
    float dist_squared = dot(diff, diff);
    float penetration = -1.0f;
    //Compare distance, avoid sqrt if possible for performance and numerical stability
    if (dist_squared < b->radius * b->radius) {
        float dist = sqrtf(dist_squared);
        // If the distance is very small, we are likely colliding with a face directly, so we can use the normal of the face
        
        if (dist < 0.0001f) {
            float dist_to_faces[6] = {
                fabsf(b->pos.x - aabb.min.x), // Left (-X)
                fabsf(b->pos.x - aabb.max.x), // Right (+X)
                fabsf(b->pos.y - aabb.min.y), // Bottom (-Y)
                fabsf(b->pos.y - aabb.max.y), // Top (+Y)
                fabsf(b->pos.z - aabb.min.z), // Back (-Z)
                fabsf(b->pos.z - aabb.max.z)  // Front (+Z)
            };
            
            int closest_face = 0;
            float min_dist = dist_to_faces[0];
            for (int i = 1; i < 6; i++) {
                if (dist_to_faces[i] < min_dist) {
                    min_dist = dist_to_faces[i];
                    closest_face = i;
                }
            }

            vec3 normals[6] = {
                {-1.0f, 0.0f, 0.0f},  // Left
                {1.0f, 0.0f, 0.0f},   // Right
                {0.0f, -1.0f, 0.0f},  // Bottom
                {0.0f, 1.0f, 0.0f},   // Top
                {0.0f, 0.0f, -1.0f},  // Back
                {0.0f, 0.0f, 1.0f}    // Front
            };
            
            out_normal = normals[closest_face];
            penetration = b->radius + min_dist;
  
        }
        else { 
            out_normal = diff / dist; // Use aleready computed sqrt for normalization
            penetration = b->radius - dist;
        }
        

        //float dist = sqrtf(dist_squared);
        //out_normal = diff / dist; // Use already computed sqrt for normalization
        penetration = b->radius - dist;
        b->pos = b->pos + out_normal * penetration; // Push the ball out of the collision
        return true;
    }
    
    return false;
}

// Resolve bounce with spin on a surface, normal must be normalized
// Resuelve el rebote con spin sobre una superficie (la normal debe estar normalizada)
void resolve_spin_bounce(ball* b, vec3 normal) {
    // 1. ¿Con qué fuerza choca contra la superficie?
    float vel_normal_mag = dot(b->vel, normal);
    // Si es mayor o igual a 0, la pelota ya está rebotando o alejándose. Salimos.
    if (vel_normal_mag >= 0.0f) return;
    // --- SECCIÓN 1: TRASLACIÓN (Movimiento Lineal) ---
    // 2. Separar la velocidad actual en sus dos componentes
    vec3 vel_normal = normal * vel_normal_mag; // La que "choca"
    vec3 vel_tangent = b->vel - vel_normal;    // La que "raspa" paralela a la mesa

    // 3. Calcular la velocidad de la superficie de la goma por el giro
    // El vector que va desde el centro de la bola al punto de contacto es la normal hacia abajo
    vec3 r_contact = -normal * b->radius; 
    // Producto cruz: Giro x Vector al suelo
    vec3 contact_vel_spin = cross(b->angular_vel, r_contact);

    // 4. Velocidad total de deslizamiento (Slip Velocity)
    // Es la suma de cómo avanza el centro + cómo raspa la goma
    vec3 slip_vel = vel_tangent + contact_vel_spin;

    // 5. Aplicar la fricción de la mesa
    // La fricción actúa en dirección opuesta al deslizamiento para intentar frenarlo
    // b->friction es un valor entre 0.0 (hielo) y 1.0 (súper pegajoso)
    vec3 delta_vel_tangent = -slip_vel * b->friction;

    // 6. Actualizar las velocidades lineales
    vel_tangent = vel_tangent + delta_vel_tangent; // Se frena el avance tangencial
    vel_normal = -vel_normal * b->bounce;          // Se invierte y reduce el rebote normal (Restitución)
    
    // Aplicamos la nueva velocidad lineal a la pelota
    b->vel = vel_tangent + vel_normal; 

    // --- SECCIÓN 2: ROTACIÓN (El Spin) ---

    // 7. Transferir el impacto de la fricción al giro (Conservación de Energía)
    // Para una pelota de ping pong (esfera hueca), el factor de inercia inverso es 1.5
    // (Si fuera una bola de billar maciza, sería 2.5)
    float inertia_factor = 1.5f; 
    
    // El cambio en el giro es perpendicular a la fuerza de fricción y a la normal de la mesa.
    // Al dividir por el radio, aseguramos que la energía cuadre perfectamente.
    vec3 delta_omega = cross(delta_vel_tangent, normal) * (inertia_factor / b->radius);
    
    // Aplicamos el cambio al giro actual
    b->angular_vel = b->angular_vel + delta_omega;
}

// BALL PHYSICS - Ping Pong Ball Simulation

//Ball settings
const float GRAVITY = 9.81f;    
const float TRANSLATIONAL_DRAG = 0.1f; 
const float ANGULAR_DRAG_COEFF = 0.98f;
const float MAGNUS_STRENGTH = 0.05f;   
const float MAX_VELOCITY = 20.0f;
const float BOUNCE_DAMPING = 0.85f;
const float FRICTION = 0.4f;
const float RADIUS = 1.0f * 0.05f;

// Physics parameters
const float MAX_TIME_STEP = 0.005f;
const int MAX_SUBSTEPS = 10;    

void init_ball(ball* b, vec3 initial_pos = vec3(0.0f, 0.0f, 0.0f), vec3 initial_vel = vec3(0.0f, 0.0f, 0.0f), vec3 initial_angular_vel = vec3(0.0f, 0.0f, 0.0f)) {
    b->pos = initial_pos;
    b->vel = initial_vel;
    b->angular_vel = initial_angular_vel;
    b->radius = RADIUS;
    b->bounce =BOUNCE_DAMPING;
    b->friction = FRICTION;
    //b->mesh = nullptr;
}

vec3 get_translational_acceleration(vec3 v, vec3 omega) {
    vec3 gravity = {0.0f, -GRAVITY, 0.0f}; // Gravity
    float speed = magnitude(v);
    vec3 drag = v * (-TRANSLATIONAL_DRAG * speed); // Air resistance (quadratic drag)
    vec3 magnus = cross(omega, v) * MAGNUS_STRENGTH; // Magnus effect (lift force due to spin)
    return gravity + drag + magnus; // Total acceleration is the sum of all forces
}

vec3 get_rotation_acceleration(vec3 v, vec3 omega) {
    // Simple model: angular drag proportional to angular velocity
    return omega * (-ANGULAR_DRAG_COEFF);
}

void physics_substep(ball* b, const std::unordered_map<std::string, mesh*>& meshes, float dt) {

    // TRANSLATION INTEGRATION
    vec3 a_current = get_translational_acceleration(b->vel, b->angular_vel);
    b->pos = b->pos + (b->vel * dt) + (a_current * (0.5f * dt * dt));
    vec3 v_half = b->vel + a_current * dt;
    vec3 a_next = get_translational_acceleration(v_half, b->angular_vel);
    b->vel = b->vel + (a_current + a_next) * (0.5f * dt);

    // ROTATION INTEGRATION
    vec3 a_rot_curent = get_rotation_acceleration(b->vel, b->angular_vel);
    vec3 omega_half = b->angular_vel + a_rot_curent * dt;
    vec3 alpha_next = get_rotation_acceleration(b->vel, omega_half);
    b->angular_vel = b->angular_vel + (a_rot_curent + alpha_next) * (0.5f * dt);

    float translational_speed = magnitude(b->vel);
    if (translational_speed > MAX_VELOCITY) {
        b->vel = b->vel * (MAX_VELOCITY / translational_speed);
    }

    float angular_speed = magnitude(b->angular_vel);
    if (angular_speed > MAX_VELOCITY) {
        b->angular_vel = b->angular_vel * (MAX_VELOCITY / angular_speed);
    }

    // Finally Solve collisions with ANY mesh in the meshes map (using AABB for broad phase and then sphere-AABB for narrow phase)
    for (const auto& [name, mesh_ptr] : meshes) {
        AABB mesh_aabb = calculate_mesh_aabb(mesh_ptr);
        vec3 collision_normal;
        if (collide_sphere_aabb(b, mesh_aabb, collision_normal)) {
            resolve_spin_bounce(b, collision_normal);
        }
    }
}

void update_physics(ball* b, const std::unordered_map<std::string, mesh*>& meshes, float dt) {
    if (dt > 0.1f) dt = 0.1f;
    
    if (std::isnan(b->pos.x) || std::isnan(b->pos.y) || std::isnan(b->pos.z) ||
        std::isnan(b->vel.x) || std::isnan(b->vel.y) || std::isnan(b->vel.z)) {
        init_ball(b);
        return;
    }
    
    // Sub-stepping for numerical stability: divide the time step into smaller steps if the ball is moving fast or if dt is large
    float speed = magnitude(b->vel);
    int substeps = (int)(dt / MAX_TIME_STEP) + 1;
    float distance_per_frame = speed * dt;
    if (distance_per_frame > b->radius) {
        int velocity_substeps = (int)(distance_per_frame / b->radius) + 1;
        substeps = std::max(substeps, velocity_substeps);
    }
    substeps = std::min(substeps, MAX_SUBSTEPS);
    float sub_dt = dt / (float)substeps;
    
    for (int i = 0; i < substeps; i++) {
        physics_substep(b, meshes, sub_dt);
        if (std::isnan(b->pos.x) || std::isnan(b->pos.y) || std::isnan(b->pos.z) ||
            std::isnan(b->vel.x) || std::isnan(b->vel.y) || std::isnan(b->vel.z)) {
            init_ball(b);
            return;
        }
    }

    if(b->mesh) {
        b->mesh->setPosition(b->pos);
        b->mesh->updateRotation(b->angular_vel * dt); // Update rotation based on angular velocity
    }
}

// TRAYECTORY PREDICTION AND RALLY DIRECTION

vec3 get_random_point_in_zone(float x_min, float x_max, float z_min, float z_max) {
    float x = x_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (x_max - x_min)));
    float z = z_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (z_max - z_min)));
    return vec3(x, 0.0f, z);
}

struct interframe_result {
    vec3 p;
    float d;
};

// Continous check to find the distance between iterations of the ball trajectory.
inline interframe_result get_closest_dist_sq_in_segment(vec3 p_old, vec3 p_new, vec3 target) {
    vec3 line = p_new - p_old;
    float len_sq = dot(line, line);
    if (len_sq < 0.00001f) 
        return {
            p_new,
            dot(p_new, target)
        };
    // Proyección del target en el segmento [0, 1]
    float t = dot(target - p_old, line) / len_sq;
    t = std::max(0.0f, std::min(1.0f, t));
    
    vec3 closest_on_segment = p_old + line * t;
    vec3 diff = target - closest_on_segment;
    return {
        closest_on_segment,
        dot(diff, diff)
    };
}

// MAIN ENGINE FUNCTIONS
Engine *engine_create(int width, int height)
{
    return new Engine(width, height);
}

void engine_init(Engine *e)
{
    SDL_SetRelativeMouseMode(SDL_TRUE); 

    e->cam = camera(vec3(0.0f, 4.5f, -2.0f), vec3(0.0f, 0.0f, 0.0f), 70.0f, float(e->fb.width) / float(e->fb.height), 0.01f, 1000.0f);

    load_png_texture("res/textures/skybox3/null_plainsky512_rt.png", e->skybox_faces[0]);
    load_png_texture("res/textures/skybox3/null_plainsky512_bk.png", e->skybox_faces[1]);
    load_png_texture("res/textures/skybox3/null_plainsky512_lf.png", e->skybox_faces[3]);
    load_png_texture("res/textures/skybox3/null_plainsky512_ft.png", e->skybox_faces[2]);
    load_png_texture("res/textures/skybox3/null_plainsky512_dn.png", e->skybox_faces[5]);
    load_png_texture("res/textures/skybox3/null_plainsky512_up.png", e->skybox_faces[4]);

    //add a simple cube mesh to the engine's mesh map for testing
    ball* ball_entity = new ball;
    mesh* ball_mesh = new mesh;

    init_ball(ball_entity,vec3(0.0f,1.0f,-1.5f));
    create_sphere(ball_mesh, ball_entity->radius, 8, 12);

    //Quirky ahh thing, this should be changed later, yes the ball enity should be the only one to tsotre the mesh
    ball_entity->mesh = ball_mesh;
    e->ball_entity = ball_entity; 

    mesh* outline_mesh = new mesh;
    create_sphere(outline_mesh, ball_entity->radius * 1.5f, 6, 10);
    outline_mesh->inverseFaces = true; // Invertir caras para renderizar el contorno desde adentro
    e->ball_outline = outline_mesh;


    mesh* floor_mesh = new mesh;
    create_plane(floor_mesh, 16.0f, 16.0f);
    e->meshes["est"] = floor_mesh;

    //Create table?
    mesh* table_mesh = new mesh;
    create_cube(table_mesh, 1.0f);
    table_mesh->setPosition(vec3(0.0f, 0.76f, 0.0f));
    table_mesh->setScale(vec3(0.76f, 0.025f, 1.37f)* 2.0f);
    e->meshes["MAIN_TABLE"] = table_mesh;

    //Create net
    mesh* net_mesh = new mesh;
    create_cube(net_mesh, 1.0f);
    net_mesh->setPosition(vec3(0.0f, 0.76 + 0.076f + 0.025f, 0.0f));
    net_mesh->setScale(vec3(0.912f, 0.076f, 0.02f) * 2.0f);
    e->meshes["NET_TABLE"] = net_mesh;


    mesh* target = new mesh;
    create_cube(target, 0.1f);
    e->meshes["TARGET"] = target;
}

void engine_update(Engine *e, float dt)
{
    const Uint8 *keys = SDL_GetKeyboardState(NULL);
    if(keys[SDL_SCANCODE_R]) {
        //get the range of the table to generate a random point for the ball to target
        float x_min = -0.76f * 0.9;
        float x_max = 0.76f * 0.9;
        float z_min = -1.37f * 0.9;
        float z_max = -0.1f;

        vec3 random_target = get_random_point_in_zone(x_min, x_max, z_min, z_max) + vec3(0,0.77,0); // Add table height to target
        vec3 random_init = get_random_point_in_zone(-0.20f, 0.20f, 1.5f, 2.5f) + vec3(0.0f,(rand() % 150) / 100.0f + 0.1,0.0f); // Random initial position for the ball to add variety
        init_ball(e->ball_entity, random_init);

        vec3 rally_direction = find_rally_direction(e->ball_entity, random_target, 10.0f, e->meshes);
        e->ball_entity->vel = rally_direction * 10.0f; // Set
        e->meshes["TARGET"]->setPosition(random_target);
    }
    
    if(keys[SDL_SCANCODE_G]) {
        dt *= 0.2f;
    }
    
    
    update_physics(e->ball_entity, e->meshes, dt);
    e->ball_outline->setPosition(e->ball_entity->pos);

    // == CAMERA ONLY ==
    #ifdef DEBUG_CAMERA 
        int mdx = 0, mdy = 0;
        SDL_GetRelativeMouseState(&mdx, &mdy);
        
        float d_yaw = float(mdx) * kMouseSensitivity;
        float d_pitch = float(mdy) * kMouseSensitivity;


        vec3 inputDir(0.0f, 0.0f, 0.0f);
        if (keys[SDL_SCANCODE_W])
            inputDir.z -= 1.0f;
        if (keys[SDL_SCANCODE_S])
            inputDir.z += 1.0f;
        if (keys[SDL_SCANCODE_A])
            inputDir.x -= 1.0f;
        if (keys[SDL_SCANCODE_D])
            inputDir.x += 1.0f;
        if (keys[SDL_SCANCODE_LSHIFT])
            inputDir.y -= 1.0f;
        if (keys[SDL_SCANCODE_SPACE])
            inputDir.y += 1.0f;
        float s = sinf(e->yaw);
        float c = cosf(e->yaw);


        //use arrow kkeys and + - to control the target position of the camera, then lerp the actual camera position to it for a smooth movement
        vec3 inputDirTarget(0.0f, 0.0f, 0.0f);
        if (keys[SDL_SCANCODE_UP])
            inputDirTarget.z -= 1.0f;
        if (keys[SDL_SCANCODE_DOWN])
            inputDirTarget.z += 1.0f;
        if (keys[SDL_SCANCODE_LEFT])
            inputDirTarget.x -= 1.0f;
        if (keys[SDL_SCANCODE_RIGHT])
            inputDirTarget.x += 1.0f;
        if (keys[SDL_SCANCODE_KP_PLUS])
            inputDirTarget.y += 1.0f;
        if (keys[SDL_SCANCODE_KP_MINUS])
            inputDirTarget.y -= 1.0f;

        //apply to target with a speed of 1 units per second
        e->target = e->target + inputDirTarget * dt * 1.0f;

        vec3 moveDir;
        moveDir.x = (inputDir.x * c - inputDir.z * s) * moveSpeed;
        moveDir.z = (inputDir.x * s + inputDir.z * c) * moveSpeed;
        moveDir.y = inputDir.y * moveSpeed;
        e->position = e->position + moveDir * dt;
        e->yaw += d_yaw;
        e->pitch += d_pitch;
        e->pitch = std::clamp(e->pitch, to_radians(-85.0f), to_radians(85.0f));
        e->cam.setPosition(e->position);
        e->cam.setRotation(vec3(e->pitch, e->yaw, 0.0f));
       //e->cam.lookAt(e->target); // Mirar al centro de la mesa
    #endif
}

void engine_render(Engine *e, SDL_Texture *sdl_fb_texture, float dt)
{
    e->fb.clear(0xFF222222);

    std::hash<std::string> r;

    //render all the meshes in the engine's mesh map with a simple shader config
    renderConfig cfg;
    for (auto &[name, mesh_ptr] : e->meshes)
    {
        cfg.baseColor = (uint32_t)r(name); //hash the name to get a color
        render_mesh(e->fb, e->cam, *mesh_ptr, cfg);
    }

    //Draw the ball mesh
    renderConfig ball_cfg;
    ball_cfg.baseColor = 0xFFFFFFFF; // White color for the ball
    ball_cfg.ignoreLight = true; // Make the ball unaffected by lighting for better visibility
    render_mesh(e->fb, e->cam, *e->ball_entity->mesh, ball_cfg);

    renderConfig outline_cfg;
    outline_cfg.baseColor = 0x88000000; // Cyan color for the outline
    outline_cfg.ignoreLight = true;
    render_mesh(e->fb, e->cam, *e->ball_outline, outline_cfg);


    // render the skybox at the end with depth test disabled for a simple implementation
    render_skybox(e->fb, e->cam, e->skybox_faces);
    render_gizmo(e->fb, e->cam, vec3{}, 1.0f);
    
    char HUD[512] = {0};
    snprintf(HUD, sizeof(HUD), 
        "=== SIRVE ===\n"
        "Frametime: %.2fms\n"
        "CAMROT: %.2f %.2f %.2f\n"
        "CAMPOS %.2f %.2f %.2f\n"
        "BALL_VEL POS: %+.2f %+.2f %+.2f\n"
        "BALL VEL ROT: %+.2f %+.2f %+.2f\n"
        "BALL SPEED: %.4f\n",
        dt * 1000.0f,
        e->cam._rotation.x, e->cam._rotation.y, e->cam._rotation.z,
        e->cam._position.x, e->cam._position.y, e->cam._position.z,
        e->ball_entity->vel.x, e->ball_entity->vel.y, e->ball_entity->vel.z,
        e->ball_entity->angular_vel.x, e->ball_entity->angular_vel.y, e->ball_entity->angular_vel.z,
        magnitude(e->ball_entity->vel)
    );    
    
    draw_text(e->fb, 22, 22, HUD, 0x88000000, 0x88000000); // Sombra
    draw_text(e->fb, 20, 20, HUD, 0xFFFFFFFF);
    SDL_UpdateTexture(sdl_fb_texture, NULL, e->fb.colorBuffer, e->fb.width * sizeof(uint32_t));
}

void engine_handle_events(Engine *e, SDL_Event &event, bool &running)
{
    if (event.type == SDL_QUIT)
        running = false;
    if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)
        running = false;
}

#endif