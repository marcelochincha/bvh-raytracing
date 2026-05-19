#pragma once

#include <math/sr_math.hpp>
#include <renderer/sr_geometry.hpp>
#include <unordered_map>
#include <string>

// Axis convention (matches the rendered table in sr_game.cpp):
//   X = long axis  (table length, 2.74 m)
//   Y = up
//   Z = width      (table width,  1.52 m)
// Table top surface sits at Y = 0.

struct Ball {
    vec3 pos;
    vec3 vel;
    vec3 angular_vel;
    mesh* mesh = nullptr;
    float radius;
    float bounce;
    float friction;
};

struct AABB {
    vec3 min;
    vec3 max;
};

// Result of the launch-velocity solver (Solver #1 from tests/sim_2.ipynb).
// Given a fixed launch speed, there are up to two ballistic angles that hit
// the target: a "low/fast" trajectory and a "high/slow" trajectory.
// If no ballistic solution exists at the given speed, `valid` is false and
// both vel_low and vel_high are set to the 45°-elevation fallback aimed at
// the target (with magnitude = speed).
//
// Net clearance: the solver checks whether each trajectory's height at the
// net plane (x = net_x) clears `net_clearance_y`. `vel` is automatically
// set to the preferred velocity — vel_low if it clears the net, otherwise
// vel_high. Callers can use `vel` and ignore the rest.
struct BallLaunchSolution {
    vec3  vel_low;
    vec3  vel_high;
    float t_low;
    float t_high;
    bool  valid;
    bool  low_clears_net;
    bool  high_clears_net;
    vec3  vel;   // preferred: low if it clears the net, else high
    float t;     // matching flight time for `vel`
};

// Solver #1: pure ballistic (no Magnus, no drag) launch-velocity solver.
//   initial_pos     : ball start position
//   target_pos      : point we want to hit
//   speed           : launch speed magnitude
//   net_x           : x plane of the net (default 0 = table centre)
//   net_clearance_y : minimum y the trajectory must reach at x=net_x
//                     (= net top + ball radius). Default ≈ 0.20 m.
// Returns both feasible velocities, with `vel` picking the one that clears
// the net, or a 45° fallback if no ballistic solution exists.
BallLaunchSolution solve_launch_velocity(vec3 initial_pos, vec3 target_pos, float speed,
                                         float net_x = 0.0f,
                                         float net_clearance_y = 0.20f);

// Solver #2: finds the SMALLEST launch speed in [v_min, v_max] at which at
// least one trajectory clears the net, by bisection. The returned solution
// has `vel` set to the low arc if it clears (preferred for speed), else
// the high arc. Useful when the fixed-speed Solver #1 can't pass the net.
//
// If even v_max can't reach or clear, returns the v_max attempt as a best
// effort. If v_min already works, returns immediately without bisecting.
BallLaunchSolution solve_launch_velocity_min_speed(
    vec3 initial_pos, vec3 target_pos,
    float v_min = 4.0f,
    float v_max = 6.0f,
    int   iters = 14,
    float net_x = 0.0f,
    float net_clearance_y = 0.20f);

// Initialise a ball with sensible ping-pong defaults.
void ball_init(Ball* b,
               vec3 pos        = vec3(0.0f, 0.0f, 0.0f),
               vec3 vel        = vec3(0.0f, 0.0f, 0.0f),
               vec3 angular_vel = vec3(0.0f, 0.0f, 0.0f));

// Advance the ball by `dt` seconds. Applies gravity + light translational drag,
// substeps for numerical stability, and resolves collisions against every AABB
// in `world` (continuous sweep first, discrete fallback). Magnus effect is OFF.
void ball_update(Ball* b,
                 const std::unordered_map<std::string, AABB>& world,
                 float dt);

// Convenience overload: derive AABBs from the rendering meshes map on the fly.
void ball_update(Ball* b,
                 const std::unordered_map<std::string, mesh*>& meshes,
                 float dt);

// Compute world-space AABB of a mesh (taking its model matrix into account).
AABB mesh_world_aabb(const mesh* m);
