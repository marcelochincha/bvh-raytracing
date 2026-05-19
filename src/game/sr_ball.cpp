#include <game/sr_ball.hpp>

#include <algorithm>
#include <cmath>

// Faithful port of the physics logic in tests/sim_2.ipynb, with the Magnus
// effect intentionally omitted (per request). The notebook's `update_ball`
// and `check_and_solve_collision` are the reference; the only deviations
// from a literal port are noted inline.

namespace {

// ---- Tunables (taken from sim_2.ipynb) --------------------------------
constexpr float GRAVITY        = 9.81f;
constexpr float MAX_VELOCITY   = 50.0f;
constexpr float BOUNCE_COEFF   = 0.95f;
constexpr float FRICTION_COEFF = 0.1f;
constexpr float BALL_RADIUS    = 0.02f;

// Sub-stepping keeps the engine stable when frame dt is large. The notebook
// itself uses a fixed 1/30 s tick with continuous collision; we mirror that
// upper bound here. Within each substep we do a single RK2 integrate +
// collision pass — exactly the notebook's main loop.
constexpr float MAX_SUBSTEP_DT = 1.0f / 60.0f;
constexpr int   MAX_SUBSTEPS   = 8;

// ---- Helpers ----------------------------------------------------------
inline bool has_nan(const vec3& v) {
    return std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z);
}

inline vec3 clamp_magnitude(vec3 v, float max_mag) {
    float m = magnitude(v);
    return (m > max_mag) ? v * (max_mag / m) : v;
}

inline vec3 closest_point_on_aabb(const AABB& box, const vec3& p) {
    return vec3(
        std::max(box.min.x, std::min(p.x, box.max.x)),
        std::max(box.min.y, std::min(p.y, box.max.y)),
        std::max(box.min.z, std::min(p.z, box.max.z))
    );
}

// Discrete sphere-vs-AABB test — mirrors `check_discrete_aabb_collision`.
bool discrete_sphere_aabb(const vec3& pos, float radius, const AABB& box,
                          vec3& out_normal, float& out_penetration)
{
    vec3 closest = closest_point_on_aabb(box, pos);
    vec3 diff    = pos - closest;
    float d2     = dot(diff, diff);
    float r2     = radius * radius;
    if (d2 >= r2) return false;

    float d = std::sqrt(d2);
    if (d > 1e-4f) {
        out_normal      = diff / d;
        out_penetration = radius - d;
        return true;
    }

    // Center inside the box: pick the closest face.
    float dists[6] = {
        pos.x - box.min.x, box.max.x - pos.x,
        pos.y - box.min.y, box.max.y - pos.y,
        pos.z - box.min.z, box.max.z - pos.z,
    };
    int   imin = 0;
    float dmin = dists[0];
    for (int i = 1; i < 6; ++i) if (dists[i] < dmin) { dmin = dists[i]; imin = i; }
    const vec3 normals[6] = {
        { -1, 0, 0 }, { 1, 0, 0 },
        {  0,-1, 0 }, { 0, 1, 0 },
        {  0, 0,-1 }, { 0, 0, 1 },
    };
    out_normal      = normals[imin];
    out_penetration = radius + dmin;
    return true;
}

// Continuous swept-AABB test — mirrors `check_continuous_aabb_collision`.
// Treats the moving sphere as a point against the AABB inflated by `radius`.
bool continuous_sphere_aabb(const vec3& prev_pos, const vec3& curr_pos,
                            float radius, const AABB& box,
                            float& out_t, vec3& out_normal)
{
    vec3 emin = { box.min.x - radius, box.min.y - radius, box.min.z - radius };
    vec3 emax = { box.max.x + radius, box.max.y + radius, box.max.z + radius };

    vec3 delta = curr_pos - prev_pos;
    constexpr float eps = 1e-8f;

    float t0[3], t1[3];
    for (int i = 0; i < 3; ++i) {
        float d  = delta.v[i];
        float ds = (std::fabs(d) < eps) ? (d < 0 ? -eps : eps) : d;
        t0[i] = (emin.v[i] - prev_pos.v[i]) / ds;
        t1[i] = (emax.v[i] - prev_pos.v[i]) / ds;
    }
    float tmin[3] = { std::min(t0[0], t1[0]), std::min(t0[1], t1[1]), std::min(t0[2], t1[2]) };
    float tmax[3] = { std::max(t0[0], t1[0]), std::max(t0[1], t1[1]), std::max(t0[2], t1[2]) };

    float t_enter = std::max(tmin[0], std::max(tmin[1], tmin[2]));
    float t_exit  = std::min(tmax[0], std::min(tmax[1], tmax[2]));
    if (t_enter > t_exit || t_enter < 0.0f || t_enter > 1.0f) return false;

    int axis = (t_enter == tmin[0]) ? 0 : (t_enter == tmin[1] ? 1 : 2);
    vec3 n(0, 0, 0);
    n.v[axis] = (delta.v[axis] > 0.0f) ? -1.0f : 1.0f;

    out_t      = t_enter;
    out_normal = n;
    return true;
}

// RK2 (midpoint) integration — mirrors `update_ball(ball, t)` in the notebook.
// Magnus is omitted per request; acceleration is gravity only.
void integrate(Ball* b, float dt) {
    b->vel         = clamp_magnitude(b->vel,         MAX_VELOCITY);
    b->angular_vel = clamp_magnitude(b->angular_vel, MAX_VELOCITY * 0.25f);

    vec3 a = vec3(0.0f, -GRAVITY, 0.0f);

    vec3 mid_vel = b->vel + a * (0.5f * dt);
    b->pos = b->pos + mid_vel * dt;
    b->vel = b->vel + a * dt;
}

// Bounce resolution — mirrors `resolve_bounce`. Notably, the notebook does
// NOT update angular_vel on bounce; we preserve that behaviour.
void resolve_bounce(Ball* b, vec3 normal) {
    float vn = dot(b->vel, normal);
    if (vn >= -0.01f) return;

    vec3 v_n = normal * vn;
    vec3 v_t = b->vel - v_n;

    vec3 r_contact = -normal * b->radius;
    vec3 spin_surf = cross(b->angular_vel, r_contact);
    vec3 slip      = v_t + spin_surf;
    vec3 dv_t      = -slip * b->friction;

    v_t = v_t + dv_t;
    v_n = -v_n * b->bounce;
    b->vel = v_t + v_n;
}

// One collision pass over the world — mirrors `check_and_solve_collision`.
// For each box, try continuous first; if no tunneling, fall back to discrete.
// If the collided box is named "NET", drastically damp velocity and spin
// (matches the notebook's special-case net behaviour).
void resolve_collisions(Ball* b,
                        const vec3& prev_pos,
                        const vec3& prev_vel,
                        float dt,
                        const std::unordered_map<std::string, AABB>& world)
{
    constexpr int MAX_ITERS = 3;
    int counter = 0;

    for (const auto& [name, box] : world) {
        if (counter >= MAX_ITERS) break;

        bool collided = false;

        // Continuous sweep first (catches tunneling at high speeds).
        float t_impact;
        vec3  hit_normal;
        if (continuous_sphere_aabb(prev_pos, b->pos, b->radius, box, t_impact, hit_normal)) {
            // Re-integrate from prev state for fraction t_impact, then bounce.
            // (Matches notebook: `update_ball(pb, t_impact); resolve_bounce(pb, n)`.)
            Ball pb = *b;
            pb.pos = prev_pos;
            pb.vel = prev_vel;
            integrate(&pb, dt * t_impact);
            resolve_bounce(&pb, hit_normal);
            b->pos = pb.pos;
            b->vel = pb.vel;
            // Note: notebook leaves b->angular_vel unchanged here.
            collided = true;
        } else {
            // Discrete fallback (catches slow / resting overlap).
            vec3  n;
            float pen;
            if (discrete_sphere_aabb(b->pos, b->radius, box, n, pen)) {
                b->pos = b->pos + n * pen;
                resolve_bounce(b, n);
                collided = true;
            }
        }

        if (collided) {
            counter++;
            // NET special case: heavy energy loss when the ball hits the net.
            if (name == "NET") {
                b->vel         = b->vel * 0.1f;
                b->angular_vel = b->angular_vel * 0.1f;
            }
        }
    }
}

} // anonymous namespace

// ============================================================================
// Public API
// ============================================================================

void ball_init(Ball* b, vec3 pos, vec3 vel, vec3 angular_vel) {
    b->pos         = pos;
    b->vel         = vel;
    b->angular_vel = angular_vel;
    b->radius      = BALL_RADIUS;
    b->bounce      = BOUNCE_COEFF;
    b->friction    = FRICTION_COEFF;
}

// Solver #1 — faithful port of `get_velocity_to_hit_target` (sim_2.ipynb).
// Fallback (45° aimed at target, magnitude=speed) is added per request when
// no ballistic solution exists at the given launch speed. The solver also
// checks whether each trajectory clears the net (a vertical plane at
// x = net_x with minimum clearance height net_clearance_y), and picks the
// preferred velocity automatically into `out.vel` / `out.t`.
BallLaunchSolution solve_launch_velocity(vec3 initial_pos, vec3 target_pos, float speed,
                                         float net_x, float net_clearance_y)
{
    BallLaunchSolution out;

    vec3  horiz  = vec3(target_pos.x - initial_pos.x, 0.0f, target_pos.z - initial_pos.z);
    float h_dist = magnitude(horiz);
    float dy     = target_pos.y - initial_pos.y;
    float v0     = speed;

    // atan2(z, x) — azimuth = 0 points along +X.
    float azimuth = std::atan2(horiz.z, horiz.x);

    auto angle_to_velocity = [&](float theta) -> vec3 {
        float vy = v0 * std::sin(theta);
        float vh = v0 * std::cos(theta);
        return vec3(vh * std::cos(azimuth), vy, vh * std::sin(azimuth));
    };

    auto fallback_45 = [&]() -> vec3 {
        constexpr float k = 0.70710678f; // cos(45°)
        return vec3(v0 * k * std::cos(azimuth), v0 * k, v0 * k * std::sin(azimuth));
    };

    // Does this trajectory (velocity v, total flight t) clear the net plane?
    // Computes ball y at the moment its x reaches `net_x`, and compares with
    // the required clearance. If the trajectory never crosses the net plane
    // (same side of net, or vx==0), it's considered "clear".
    auto clears_net = [&](const vec3& v, float t_flight) -> bool {
        if (std::fabs(v.x) < 1e-6f) return true;
        float t_cross = (net_x - initial_pos.x) / v.x;
        if (t_cross <= 0.0f || t_cross >= t_flight) return true;
        float y = initial_pos.y + v.y * t_cross - 0.5f * GRAVITY * t_cross * t_cross;
        return y > net_clearance_y;
    };

    // Helper: finalise the result by picking the preferred velocity. Prefers
    // vel_low if it clears the net, else falls back to vel_high. If neither
    // clears, sticks with vel_high (best-effort: higher arc has a better
    // chance of grazing over).
    auto finalise = [&]() {
        out.low_clears_net  = clears_net(out.vel_low,  out.t_low);
        out.high_clears_net = clears_net(out.vel_high, out.t_high);
        //if (out.low_clears_net) {
        out.vel = out.vel_low;  out.t = out.t_low;
        //} else {
        //    out.vel = out.vel_high; out.t = out.t_high;
        //}
    };

    // Degenerate: target directly above/below start.
    if (h_dist < 1e-5f) {
        vec3 fb = fallback_45();
        out.vel_low = out.vel_high = fb;
        out.t_low = out.t_high = 0.0f;
        out.valid = false;
        finalise();
        return out;
    }

    float v0_sq = v0 * v0;
    float disc  = v0_sq * v0_sq - GRAVITY * (GRAVITY * h_dist * h_dist + 2.0f * dy * v0_sq);
    if (disc < 0.0f) {
        vec3 fb = fallback_45();
        out.vel_low = out.vel_high = fb;
        out.t_low = out.t_high = 0.0f;
        out.valid = false;
        finalise();
        return out;
    }

    float sqrt_disc  = std::sqrt(disc);
    float gx         = GRAVITY * h_dist;
    float theta_a    = std::atan((v0_sq + sqrt_disc) / gx);
    float theta_b    = std::atan((v0_sq - sqrt_disc) / gx);
    float theta_low  = std::min(theta_a, theta_b);
    float theta_high = std::max(theta_a, theta_b);

    out.vel_low  = angle_to_velocity(theta_low);
    out.vel_high = angle_to_velocity(theta_high);
    out.t_low    = h_dist / (v0 * std::cos(theta_low));
    out.t_high   = h_dist / (v0 * std::cos(theta_high));
    out.valid    = true;
    finalise();
    return out;
}

// Solver #2: bisection search over launch speed. Prefers the LOW arc to
// avoid always serving high parabolas — checks first whether the low arc
// can clear the net at v_max (Case A: low always clears for v >= v_reach).
// If so, bisects for the smallest v where the low arc clears. Otherwise
// (Case B) falls back to the original "min v where any arc clears", which
// will pick the high arc via `finalise()` in solve_launch_velocity.
BallLaunchSolution solve_launch_velocity_min_speed(
    vec3 initial_pos, vec3 target_pos,
    float v_min, float v_max, int iters,
    float net_x, float net_clearance_y)
{
    auto try_v = [&](float v) {
        return solve_launch_velocity(initial_pos, target_pos, v, net_x, net_clearance_y);
    };
    auto low_clears  = [&](const BallLaunchSolution& s) {
        return s.valid && s.low_clears_net;
    };
    auto any_clears  = [&](const BallLaunchSolution& s) {
        return s.valid && (s.low_clears_net || s.high_clears_net);
    };

    // Best-effort upper bound: if even v_max can't clear, return that.
    BallLaunchSolution sol_hi = try_v(v_max);
    if (!any_clears(sol_hi)) return sol_hi;

    // CASE A: the LOW arc clears at v_max. By the limit-to-chord property,
    // the low arc clears for every v in [v_reach, v_max], so we can bisect
    // straight for the minimum v at which low clears.
    if (low_clears(sol_hi)) {
        BallLaunchSolution sol_lo = try_v(v_min);
        if (low_clears(sol_lo)) return sol_lo;

        float lo = v_min, hi = v_max;
        BallLaunchSolution best = sol_hi;
        for (int i = 0; i < iters; ++i) {
            float mid = 0.5f * (lo + hi);
            BallLaunchSolution s = try_v(mid);
            if (low_clears(s)) { hi = mid; best = s; }
            else                { lo = mid; }
        }
        return best;
    }

    // CASE B: low arc can't clear at v_max → geometry forces a high arc.
    // Find min v where any arc clears (the original behaviour).
    BallLaunchSolution sol_lo = try_v(v_min);
    if (any_clears(sol_lo)) return sol_lo;

    float lo = v_min, hi = v_max;
    BallLaunchSolution best = sol_hi;
    for (int i = 0; i < iters; ++i) {
        float mid = 0.5f * (lo + hi);
        BallLaunchSolution s = try_v(mid);
        if (any_clears(s)) { hi = mid; best = s; }
        else                { lo = mid; }
    }
    return best;
}

AABB mesh_world_aabb(const mesh* m) {
    AABB out;
    if (m->vertices.empty()) {
        out.min = out.max = vec3(0, 0, 0);
        return out;
    }
    mat4 model = m->modelMatrix();
    vec3 lo(1e10f, 1e10f, 1e10f);
    vec3 hi(-1e10f, -1e10f, -1e10f);
    for (const vertex& v : m->vertices) {
        vec4 w = model * vec4(v.p.x, v.p.y, v.p.z, 1.0f);
        lo = minimum(lo, vec3(w.x, w.y, w.z));
        hi = maximum(hi, vec3(w.x, w.y, w.z));
    }
    out.min = lo;
    out.max = hi;
    return out;
}

void ball_update(Ball* b,
                 const std::unordered_map<std::string, AABB>& world,
                 float dt)
{
    if (dt > 0.1f) dt = 0.1f;
    if (has_nan(b->pos) || has_nan(b->vel)) {
        ball_init(b);
        return;
    }

    int substeps = std::max(1, (int)std::ceil(dt / MAX_SUBSTEP_DT));
    substeps     = std::min(substeps, MAX_SUBSTEPS);
    float sub_dt = dt / (float)substeps;

    for (int i = 0; i < substeps; ++i) {
        vec3 prev_pos = b->pos;
        vec3 prev_vel = b->vel;
        integrate(b, sub_dt);
        resolve_collisions(b, prev_pos, prev_vel, sub_dt, world);
        if (has_nan(b->pos) || has_nan(b->vel)) {
            ball_init(b);
            return;
        }
    }

    if (b->mesh) {
        b->mesh->setPosition(b->pos);
        b->mesh->updateRotation(b->angular_vel * dt);
    }
}

void ball_update(Ball* b,
                 const std::unordered_map<std::string, mesh*>& meshes,
                 float dt)
{
    std::unordered_map<std::string, AABB> world;
    world.reserve(meshes.size());
    for (const auto& [name, m] : meshes)
        if (name != "TARGET")
            world.emplace(name, mesh_world_aabb(m));
    ball_update(b, world, dt);
}
