import numpy as np
import numpy.typing as npt

#
# PHYSICS SECTION
#
TABLE_LENGTH = 2.74 #m
TABLE_WIDTH  = 1.525 #m
GRAVITY = 9.81
TRANSLATIONAL_DRAG = 0.05
MAGNUS_STRENGTH = 0.08
ANGULAR_DRAG_COEFF = 0.01
MAX_VELOCITY = 120.0
BALL_RADIUS = 0.02
NET_HEIGHT = 0.1525
MARGIN_BALL = 0.01 # Margin to consider the ball has hit the net, to avoid numerical issues
FRICTION_COEFF = 0.2 # Coefficient of friction for the bounce, this is a simplified model and can be adjusted for realism
BOUNCE_COEFF = 0.9 # Coefficient of restitution for the bounce, this is a simplified model and can be adjusted for realism
INERTIA_FACTOR = 1.1

#
# MATH SECTION
#
vec3 = npt.NDArray[np.float32]

def vec3_create(x : float = 0, y : float = 0, z : float = 0) -> vec3:
    return np.array([x, y, z], dtype=np.float32)

def dot(a : vec3, b : vec3) -> float:
    return np.dot(a, b)

def cross(a : vec3, b : vec3) -> vec3:
    return np.cross(a, b)

def magnitude(v : vec3) -> float:
    return np.linalg.norm(v)

#STRUCTURES
class Ball: #SIMPLE STRUCTURE TO HOLD BALL STATE
    def __init__(self, pos : vec3, vel : vec3, angular_vel : vec3):
        self.pos = pos
        self.vel = vel
        self.angular_vel = angular_vel
        
class AABB: #AXIS ALIGNED BOUNDING BOX, FOR COLLISION DETECTION
    def __init__(self, min_bound : vec3, max_bound : vec3):
        self.min_bound = min_bound
        self.max_bound = max_bound
        

def check_discrete_aabb_collision(ball : Ball, box : AABB) -> bool:
    closest_point = np.maximum(box.min_bound, np.minimum(ball.pos, box.max_bound))
    distance = dot(closest_point - ball.pos)
    return distance < BALL_RADIUS * BALL_RADIUS

def check_continuous_aabb_collision(prevBall, currentBall, box):
    radius = BALL_RADIUS
    expanded_min = box.min_bound - radius
    expanded_max = box.max_bound + radius

    origin = prevBall.pos
    delta = currentBall.pos - prevBall.pos # Vector direccional del movimiento

    epsilon = 1e-8
    dir_safe = np.where(np.abs(delta) < epsilon, epsilon, delta)
    
    t0 = (expanded_min - origin) / dir_safe
    t1 = (expanded_max - origin) / dir_safe
    
    tmin_axis = np.minimum(t0, t1)
    tmax_axis = np.maximum(t0, t1)
    
    t_enter = np.max(tmin_axis)
    t_exit = np.min(tmax_axis)
    
    if t_enter <= t_exit and t_enter >= 0.0 and t_enter <= 1.0:
        hit_normal = np.array([0.0, 0.0, 0.0])
        if t_enter == tmin_axis[0]: # Chocó en el eje X (Pared izquierda o derecha)
            hit_normal[0] = -1.0 if delta[0] > 0 else 1.0
        elif t_enter == tmin_axis[1]: # Chocó en el eje Y (Piso o techo)
            hit_normal[1] = -1.0 if delta[1] > 0 else 1.0
        else: # Chocó en el eje Z (Pared frontal o trasera)
            hit_normal[2] = -1.0 if delta[2] > 0 else 1.0
        return True, t_enter, hit_normal
    return False, None, None

def check_discrete_aabb_collision(ball: Ball, box: AABB):
    closest_point = np.maximum(box.min_bound, np.minimum(ball.pos, box.max_bound))
    diff = ball.pos - closest_point
    dist_squared = np.dot(diff, diff)

    if dist_squared >= (BALL_RADIUS * BALL_RADIUS):
        return False, None, 0.0

    distance = np.sqrt(dist_squared)

    if distance > 0.0001: 
        # Normalizamos el vector para obtener la dirección
        normal = diff / distance 
        penetration = BALL_RADIUS - distance
        return True, normal, penetration

    distances = [
        ball.pos[0] - box.min_bound[0], # 0: Distancia a pared X Izquierda
        box.max_bound[0] - ball.pos[0], # 1: Distancia a pared X Derecha
        ball.pos[1] - box.min_bound[1], # 2: Distancia a pared Y Abajo (Suelo)
        box.max_bound[1] - ball.pos[1], # 3: Distancia a pared Y Arriba (Techo)
        ball.pos[2] - box.min_bound[2], # 4: Distancia a pared Z Atrás
        box.max_bound[2] - ball.pos[2]  # 5: Distancia a pared Z Adelante
    ]

    min_dist = min(distances)
    min_index = distances.index(min_dist)

    normal = np.array([0.0, 0.0, 0.0])
    if min_index == 0:   normal[0] = -1.0
    elif min_index == 1: normal[0] = 1.0
    elif min_index == 2: normal[1] = -1.0
    elif min_index == 3: normal[1] = 1.0
    elif min_index == 4: normal[2] = -1.0
    elif min_index == 5: normal[2] = 1.0

    # La penetración total es el radio de la pelota MÁS lo que ya se hundió el centro
    penetration = BALL_RADIUS + min_dist
    return True, normal, penetration

def get_translational_acceleration(v : vec3, omega : vec3): 
    gravity = vec3_create(0.0, -GRAVITY, 0.0) # Gravity
    speed = magnitude(v)
    drag = v * (-TRANSLATIONAL_DRAG * speed) # Air resistance (quadratic drag)
    magnus = cross(omega, v) * MAGNUS_STRENGTH # Magnus effect (lift force due to spin)
    return gravity + drag + magnus # Total acceleration is the sum of all forces

def get_rotation_acceleration(v : vec3, omega : vec3):
    # Simple model: angular drag proportional to angular velocity
    return omega * (-ANGULAR_DRAG_COEFF)

def resolve_spin_bounce(ball, normal):
    vel_normal_mag = dot(ball.vel, normal)
    print(f"Resolving bounce: vel_normal_mag = {vel_normal_mag}")
    if vel_normal_mag >= -0.01:
        return
    vel_normal = normal * vel_normal_mag
    vel_tangent = ball.vel - vel_normal  

    r_contact = -normal * BALL_RADIUS
    contact_vel_spin = cross(ball.angular_vel, r_contact)

    slip_vel = vel_tangent + contact_vel_spin
    delta_vel_tangent = -slip_vel * FRICTION_COEFF

    vel_tangent = vel_tangent + delta_vel_tangent
    vel_normal = -vel_normal * BOUNCE_COEFF
    
    ball.vel = vel_tangent + vel_normal
    delta_omega = cross(delta_vel_tangent, normal) * (INERTIA_FACTOR)
    ball.angular_vel = ball.angular_vel + delta_omega

def resolve_position_completely(ball, normal, penetration):
    ball.pos = ball.pos + normal * penetration

#MAIN PHYSICS SUBSTEP
def integrate_physics(b, dt):
    # TRANSLATION INTEGRATION
    a_current = get_translational_acceleration(b.vel, b.angular_vel)
    b.pos = b.pos + (b.vel * dt) + (a_current * (0.5 * dt * dt))
    v_half = b.vel + a_current * dt
    a_next = get_translational_acceleration(v_half, b.angular_vel)
    b.vel = b.vel + (a_current + a_next) * (0.5 * dt)

    # ROTATION INTEGRATION
    a_rot_curent = get_rotation_acceleration(b.vel, b.angular_vel)
    omega_half = b.angular_vel + a_rot_curent * dt
    alpha_next = get_rotation_acceleration(b.vel, omega_half)
    b.angular_vel = b.angular_vel + (a_rot_curent + alpha_next) * (0.5 * dt)

    translational_speed = magnitude(b.vel)
    if (translational_speed > MAX_VELOCITY):
        b.vel = b.vel * (MAX_VELOCITY / translational_speed)
    

    angular_speed = magnitude(b.angular_vel)
    if (angular_speed > MAX_VELOCITY):
        b.angular_vel = b.angular_vel * (MAX_VELOCITY / angular_speed)

def format_vec3(v : vec3) -> str:
    return f"({v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f})"

MAX_ITERATIONS = 1024
world = {
    "FLOOR" : AABB(vec3_create(-10, -5, -10), vec3_create(10,0,10)),
}

def copy_ball(ball : Ball) -> Ball:
    return Ball(np.copy(ball.pos), np.copy(ball.vel), np.copy(ball.angular_vel))

def simulate_ball_trajectory(init_pos : vec3, init_vel : vec3, init_spin : vec3, total_time : float, time_step : float):
    ball = Ball(init_pos, init_vel, init_spin)
    trajectory = []
    time_elapsed = 0.0
    
    for _ in range(MAX_ITERATIONS):
        
        pos_str = format_vec3(ball.pos)
        vel_str = format_vec3(ball.vel)
        spin_str = format_vec3(ball.angular_vel)
        t = time_elapsed
        status_str = f"t={t:.2f}s | pos={pos_str} | vel={vel_str} | spin={spin_str}"
        
        print("=" * len(status_str))
        print(status_str)
        
        trajectory.append(np.copy(ball.pos))
        prev_ball = copy_ball(ball)
        integrate_physics(ball, time_step)
        
        print(f"After integration: pos={format_vec3(ball.pos)}, vel={format_vec3(ball.vel)}, spin={format_vec3(ball.angular_vel)}")
        
        #now check for collisions and resolve them
        box = world["FLOOR"]
        
        
        #first check using
        is_tunneling, t_impact, hit_normal = check_continuous_aabb_collision(prev_ball, ball, box)
        if is_tunneling:
            print(f"Tunneling detected! t_impact={t_impact:.4f}")
            #now do the step with the impact time to resolve the collision at the exact moment of impact, this is more accurate at high speeds to avoid bouncing too high due to tunneling through the floor
            ball = copy_ball(prev_ball) # Reset ball to previous state
            integrate_physics(ball, time_step * t_impact) # Integrate only up to the point of impact
            resolve_spin_bounce(ball, hit_normal) # Resolve the bounce at the moment of impact
            
            #ball.pos = prev_ball.pos + (prev_ball.vel * time_step * t_impact)
            #resolve_spin_bounce(ball, hit_normal)
            #integrate_physics(ball, time_step * (1 - t_impact))
        else:
            #then check using discrete collision, this is more reliable at low speeds to avoid tunneling through the floor
            is_inside, normal, penetration = check_discrete_aabb_collision(ball, box)
            if is_inside: #AT LOW SPEEDS, THIS IS MORE RELIABLE TO AVOID TUNNELING THROUGH THE FLOOR
                print("DISCRETE Collision with the floor detected!")
                resolve_position_completely(ball, normal, penetration)
                resolve_spin_bounce(ball, normal)
        # is_inside, normal, penetration = check_discrete_aabb_collision(ball, box)
        # if is_inside: #AT LOW SPEEDS, THIS IS MORE RELIABLE TO AVOID TUNNELING THROUGH THE FLOOR
        #     print("Collision with the floor detected!")
        #     resolve_position_completely(ball, normal, penetration)
        #     resolve_spin_bounce(ball, normal)
        # else:
            
        #     if is_tunneling:
        #         print(f"Tunneling detected! t_impact={t_impact:.4f}")
        #         ball.pos = prev_ball.pos + (ball.vel * time_step * t_impact)
        #         resolve_spin_bounce(ball, hit_normal)
        #         integrate_physics(ball, time_step * (1 - t_impact))

        if (total_time > 0 and time_elapsed >= total_time):
            break
        time_elapsed += time_step
        
    trajectory = np.array(trajectory)
    return trajectory

if __name__ == "__main__":
    # Example usage
    init_pos = vec3_create(0, 0.5, 0)
    init_vel = vec3_create(0,0, 0)
    init_spin = vec3_create(0, 0, 0)
    FPS = 5
    trajectory = simulate_ball_trajectory(init_pos, init_vel, init_spin, total_time=5.0, time_step=(1/FPS))
    print("Done!")