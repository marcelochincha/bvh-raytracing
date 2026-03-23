import numpy as np
import numpy.typing as npt

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

#
# PHYSICS SECTION
#

# Crear table
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

#MAIN PHYSICS SUBSTEP
def physics_substep(b, dt):
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

    # Finally Solve collisions with ANY mesh in the meshes map (using AABB for broad phase and then sphere-AABB for narrow phase)
    # for (const auto& [name, mesh_ptr] : meshes) {
    #     AABB mesh_aabb = calculate_mesh_aabb(mesh_ptr)
    #     vec3 collision_normal
    #     if (collide_sphere_aabb(b, mesh_aabb, collision_normal)) {
    #         resolve_spin_bounce(b, collision_normal)
    #     }
    # }
    
#
# SOLVER SECTION: Shooting method
#

def find_closest_point(a : vec3, b : vec3, p : vec3) -> float:
    # Compute the projection of p onto the line defined by a and b
    ab = b - a
    ap = p - a
    t = dot(ap, ab) / dot(ab, ab)
    t_clamped = max(0, min(1, t))  # Clamp t to the range [0, 1]
    closest_point = a + t_clamped * ab
    return closest_point

#Apply the ecuation for simple proyectile motion with air resistence but no spin, to get an initial guess for the initial velocity needed to reach a target point
def compute_velocity_direction_no_spin(initial_pos : vec3, target_pos : vec3, speed : float) -> vec3:
    direction = target_pos - initial_pos
    distance = magnitude(direction)
    time_of_flight = distance / speed
    vertical_drop = 0.5 * GRAVITY * (time_of_flight ** 2)
    direction[1] += vertical_drop # Adjust the y component to account for gravity
    return direction / magnitude(direction) * speed # Return the velocity vector with the desired speed

def compute_velocity_direction_with_drag(initial_pos : vec3, target_pos : vec3, speed : float, drag_coeff : float = 0.5) -> vec3:
    direction = target_pos - initial_pos
    distance = magnitude(direction)
    
    t_ideal = distance / speed
    t_drag = t_ideal * (1.0 + (drag_coeff * t_ideal) / 2.0)
    
    vertical_drop = 0.5 * GRAVITY * (t_drag ** 2)
    direction[1] += vertical_drop 
    return direction / magnitude(direction) * speed

def compute_velocity_with_drag_and_spin(initial_pos, target_pos, speed, angular_vel):
    direction = target_pos - initial_pos
    distance = magnitude(direction)
    
    t_ideal = distance / speed
    t_drag = t_ideal * (1.0 + (TRANSLATIONAL_DRAG * t_ideal) / 2.0)
    
    vertical_drop = 0.5 * GRAVITY * (t_drag ** 2)
    
    approx_vel = (direction / distance) * speed
    magnus_accel = cross(angular_vel, approx_vel) * MAGNUS_STRENGTH
    magnus_drift = 0.5 * magnus_accel * (t_drag ** 2)
    
    direction[1] += vertical_drop 
    
    direction[0] -= magnus_drift[0]
    direction[1] -= magnus_drift[1]
    direction[2] -= magnus_drift[2]
    
    return direction / magnitude(direction) * speed

#EXECUTE SIMULATION

#retuns:
#   - final position of the ball after simulating its motion
#   - a boolean is passed or not without touching the net

class SimulationResult:
    def __init__(self, final_pos : vec3, final_vel : vec3, passed_net : bool,):
        self.final_pos = final_pos
        self.final_vel = final_vel
        self.passed_net = passed_net

def simulate_ball_motion_for_target(initial_pos : vec3, initial_vel : vec3, initial_angular_vel : vec3, target_pos : vec3, max_total_time : float, time_step : float):
    ball = Ball(initial_pos, initial_vel, initial_angular_vel)
    time_elapsed = 0.0
    passed_net = True
    
    #print("Simulating ball motion...")
    while time_elapsed < max_total_time:
        prev_ball = Ball(np.copy(ball.pos), np.copy(ball.vel), np.copy(ball.angular_vel)) # Store previous state in case we need to revert
        physics_substep(ball, time_step)
        #if (prev_ball.pos[1] * ball.pos[1] < 0 and abs(ball.pos[1]) < BALL_RADIUS): # If the ball cross from above tho down
        #    print("Collision with the floor detected!")
        #    ball.pos[1] += BALL_RADIUS * 2
        #    resolve_spin_bounce(ball, vec3_create(0, 1, 0))
        

        #print("PRE_BALL_POS:", prev_ball.pos, "BALL_POS:", ball.pos, "TARGET_POS:", target_pos)
        point_closest_to_net = find_closest_point(prev_ball.pos, ball.pos, vec3_create(ball.pos[0],NET_HEIGHT, 0))
        y_diff = point_closest_to_net[1] - (NET_HEIGHT + BALL_RADIUS + MARGIN_BALL)
        
        time_elapsed += time_step
        #print(f"Time: {time_elapsed:.2f}s, Ball Pos: {ball.pos}, Distance to Target: {current_dist:.2f}, Prev dist: {prev_dist:.2f}")
        if (y_diff <= 0 and prev_ball.pos[2] * ball.pos[2] < 0):
            passed_net = False
            #print("Ball hit the net!")

        #print(f"Time: {time_elapsed:.2f}s, Ball Pos: {ball.pos}, Prev ball pos: {prev_ball.pos}")            
        if (prev_ball.pos[1] > 0 and ball.pos[1] <= 0): # If the ball cross from above tho down
            #print("Ball crossed the plane of the table!")
            #interpolate both position to find the exact point of crossing
            t_cross = prev_ball.pos[1] / (prev_ball.pos[1] - ball.pos[1])
            pos_cross = prev_ball.pos + t_cross * (ball.pos - prev_ball.pos)
            vel_cross = prev_ball.vel + t_cross * (ball.vel - prev_ball.vel)
            
            #check if here the ball passed the Z = 0 plane without touching the net
            if(prev_ball.pos[2] * ball.pos[2] < 0 and pos_cross[1] < NET_HEIGHT + BALL_RADIUS + MARGIN_BALL):
                passed_net = False
                #print("Ball hit the net at crossing!")
            return SimulationResult(pos_cross, vel_cross, passed_net) 
            
    return SimulationResult(ball.pos, ball.vel, passed_net)
        
        
MAX_ITERATIONS = 32
def solve_for_initial_velocity_unit(initial_pos : vec3, target_pos : vec3, initial_angular_vel : vec3, speed : float, total_time : float, time_step : float, tol : float) -> vec3:
    false_target = target_pos.copy()
    lr = 0.1
    prev_error_vec = None
    targets = []
    
    #if error grows for too much, reset the false target to the original target and reduce the learning rate
    for iteration in range(MAX_ITERATIONS):    
        targets.append(false_target.copy())
        
        print(f"Iteration {iteration}: Solving for target: {false_target}, real target: {target_pos}, starting position: {initial_pos}")
        candidate_velocity = compute_velocity_with_drag_and_spin(initial_pos, false_target, speed, initial_angular_vel)
        sim_result = simulate_ball_motion_for_target(initial_pos, candidate_velocity, initial_angular_vel,false_target, total_time, time_step)
        
        error_vec = (sim_result.final_pos - target_pos) 
        error_mag = magnitude(error_vec)
        

        
        print(f"Iteration {iteration}: Candidate velocity: {candidate_velocity}, End position: {sim_result.final_pos}, Error: {error_mag}")
        if (error_mag < tol and sim_result.passed_net):
            #print("Converged to a solution!", f"Error: {abs(error - prev_error)}, Iterations: {iteration}, PASSED: {sim_result.passed_net}")
            return candidate_velocity, targets  
    
        prev_error_mag = magnitude(prev_error_vec) if prev_error_vec is not None else None
        print("PREV", prev_error_mag)
        # if prev_error_mag is not None and error_mag > prev_error_mag * 1.25:
        #     print(f"Error grew significantly from {prev_error_mag:.4f} to {error_mag:.4f}, resetting false target and reducing learning rate.")
        #     return candidate_velocity, targets    
    
        #first avoid the net
        if not sim_result.passed_net:
            print("Ball did not pass the net, adjusting target to try to pass it...")
            clearance_needed = (NET_HEIGHT - sim_result.final_pos[1]) + BALL_RADIUS + 0.1
            false_target[1] += clearance_needed * 2
            prev_error_vec = None
            continue
        
        #now fit for the target
        if prev_error_vec is not None:
            d = dot(error_vec, prev_error_vec)
            if d < 0:
                lr *= 0.5
                #print(f"Iteración {iteration}: Overshoot. Reduciendo LR a {lr:.3f}")
            else:
                lr = min(lr * 1.2, 1.2)
                #print(f"Iteración {iteration}: Acelerando LR a {lr:.3f}")

        false_target -= error_vec * lr
        prev_error_vec = error_vec
                
    return candidate_velocity, targets # Return the best guess after max iterations



def animate_trajectory(trajectory, targets, init_pos, target, time_step, skip_frames=5):
    """
    Anima la trayectoria de la pelota en tiempo real.
    
    Args:
        trajectory: Array numpy con las posiciones de la pelota en cada frame
        targets: Array con la evolución de los false targets
        init_pos: Posición inicial de la pelota
        target: Posición objetivo
        time_step: Paso de tiempo usado en la simulación
        skip_frames: Saltar N frames para acelerar la animación (1 = velocidad real)
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d import Axes3D
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Configurar límites
    ax.set_xlim(-3, 3)
    ax.set_ylim(-1, 2)
    ax.set_zlim(-3, 3)
    ax.set_box_aspect((
        np.ptp(ax.get_xlim()),
        np.ptp(ax.get_ylim()),
        np.ptp(ax.get_zlim())
    ))
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.view_init(90, -90, 0)
    
    # Dibujar elementos estáticos (mesa, red, targets)
    # Mesa (plano Y = 0)
    table_x = np.linspace(-TABLE_WIDTH/2, TABLE_WIDTH/2, 4)
    table_z = np.linspace(-TABLE_LENGTH/2, TABLE_LENGTH/2, 4)
    table_x, table_z = np.meshgrid(table_x, table_z)
    table_y = np.full_like(table_x, 0)
    ax.plot_surface(table_x, table_y, table_z, color='blue', alpha=0.5, label='Table')
    
    # Red (plano Z = 0)
    x = np.linspace(-0.75, 0.75, 2)
    y = np.linspace(0, NET_HEIGHT, 2)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros_like(Y)
    ax.plot_surface(X, Y, Z, alpha=0.3, color='gray')
    
    # Puntos estáticos
    ax.scatter(target[0], target[1], target[2], color='red', s=100, label='Target', marker='*')
    ax.scatter(init_pos[0], init_pos[1], init_pos[2], color='green', s=100, label='Start', marker='o')
    
    # Evolución de false targets
    if len(targets) > 0:
        targets_array = np.array(targets).reshape(-1, 3)
        ax.scatter(targets_array[:, 0], targets_array[:, 1], targets_array[:, 2], 
                  color='orange', label='False Target Evolution', s=20, alpha=0.5)
    
    # Inicializar elementos animados
    ball_point, = ax.plot([], [], [], 'yo', markersize=10, label='Ball')
    trail_line, = ax.plot([], [], [], 'y-', linewidth=1, alpha=0.6, label='Trail')
    time_text = ax.text2D(0.05, 0.95, '', transform=ax.transAxes, fontsize=12)
    
    ax.legend(loc='upper right')
    
    def init():
        ball_point.set_data([], [])
        ball_point.set_3d_properties([])
        trail_line.set_data([], [])
        trail_line.set_3d_properties([])
        time_text.set_text('')
        return ball_point, trail_line, time_text
    
    def update(frame):
        # Índice real en la trayectoria
        idx = frame * skip_frames
        if idx >= len(trajectory):
            idx = len(trajectory) - 1
        
        # Actualizar posición de la pelota
        current_pos = trajectory[idx]
        ball_point.set_data([current_pos[0]], [current_pos[1]])
        ball_point.set_3d_properties([current_pos[2]])
        
        # Actualizar trail (últimos N puntos)
        trail_length = min(30, idx + 1)
        trail_start = max(0, idx - trail_length)
        trail = trajectory[trail_start:idx+1]
        trail_line.set_data(trail[:, 0], trail[:, 1])
        trail_line.set_3d_properties(trail[:, 2])
        
        # Actualizar texto de tiempo
        time_elapsed = idx * time_step
        time_text.set_text(f'Time: {time_elapsed:.3f}s | Frame: {idx}/{len(trajectory)-1}')
        
        return ball_point, trail_line, time_text
    
    # Calcular el intervalo en milisegundos para tiempo real
    # time_step es el tiempo entre frames de simulación
    # Multiplicamos por skip_frames y por 1000 para convertir a ms
    interval_ms = time_step * skip_frames * 1000
    
    num_frames = len(trajectory) // skip_frames
    anim = FuncAnimation(fig, update, frames=num_frames, init_func=init,
                        interval=interval_ms, blit=False, repeat=True)
    
    plt.tight_layout()
    plt.show()
    
    return anim


if __name__ == "__main__":
    target = vec3_create(0, 0, -0.5)

    init_pos = vec3_create(0, 0.1, -2) # Starting at the height of the ping pong table
    init_spin = vec3_create(10, 0, 0) # Backspin
    init_speed = 10.0 # m/s
    total_time = 4.0 # seconds
    time_step = 1 / 60
    tol = 0.1 # meters
    
    best_unit_velocity, targets = solve_for_initial_velocity_unit(init_pos, target, init_spin, init_speed, total_time, time_step, tol)
    print("Done!")
    print(f"Best initial velocity found: {best_unit_velocity}")
    print("Ready.")
    print("=" * 50)
    print("=" * 50)
    
    #now plot the vector and trayectory using matplotlib 
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # Simulate the ball motion with the best initial velocity to get the trajectory
    trajectory = []
    ball = Ball(init_pos, best_unit_velocity, init_spin)
    time_elapsed = 0.0
    while time_elapsed < total_time:
        trajectory.append(np.copy(ball.pos))
        prev_ball = Ball(np.copy(ball.pos), np.copy(ball.vel), np.copy(ball.angular_vel)) # Store previous state in case we need to revert
        if (prev_ball.pos[1] * ball.pos[1] < 0 and 
            #abs(ball.pos[1]) < BALL_RADIUS and 
            abs(ball.pos[0] + BALL_RADIUS) < TABLE_WIDTH/2 and 
            abs(ball.pos[2] + BALL_RADIUS) < TABLE_LENGTH/2): # If the ball cross from above tho down
            print("Collision with the floor detected!")
            #ball.pos[1] += BALL_RADIUS * 2
            resolve_spin_bounce(ball, vec3_create(0, 1, 0))
        physics_substep(ball, time_step)
        time_elapsed += time_step
    trajectory = np.array(trajectory)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-1, 2)
    ax.set_zlim(-3, 3)
    ax.set_box_aspect((
        np.ptp(ax.get_xlim()),
        np.ptp(ax.get_ylim()),
        np.ptp(ax.get_zlim())
    ))
    
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], label='Ball Trajectory')
    ax.scatter(target[0], target[1], target[2], color='red', label='Target')
    ax.scatter(init_pos[0], init_pos[1], init_pos[2], color='green', label='Initial Position')
    #show evolution of the false target
    targets = np.array(targets)
    targets = targets.reshape(-1, 3)
    print("Targets evolution:", targets.shape)
    
    ax.scatter(targets[:, 0], targets[:, 1], targets[:, 2], color='orange', label='False Target Evolution', s=20)
    
    #PLANE Y = 0
    table_x = np.linspace(-TABLE_WIDTH/2, TABLE_WIDTH/2, 4)
    table_z = np.linspace(-TABLE_LENGTH/2, TABLE_LENGTH/2, 4)
    table_x, table_z = np.meshgrid(table_x, table_z)
    table_y = np.full_like(table_x, 0)
    ax.plot_surface(table_x, table_y, table_z, color='blue', alpha=0.5, label='Table')
    
    #PLANE Z = 0 net must be centered on the table, and also start at the surface of the table
    # rangos
    x = np.linspace(-0.75, 0.75, 2)
    y = np.linspace(0, NET_HEIGHT, 2)

    X,Y = np.meshgrid(x, y)
    Z = np.zeros_like(Y)   # x = 0
    ax.plot_surface(X, Y, Z, alpha=0.3)

    
    
    #add the net as a plane at y = NET_HEIGHT
    # net_x = np.linspace(-2, 2, 10)
    # net_z = np.linspace(-1, 1, 10)
    # net_x, net_z = np.meshgrid(net_x, net_z)
    # net_y = np.full_like(net_x, NET_HEIGHT)
    # ax.plot_surface(net_x, net_y, net_z, color='gray', alpha=0.5, label='Net')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    #change the up vector to y
    ax.view_init(90,-90,0)
    
    ax.legend()
    plt.show()
    
    # Mostrar animación en tiempo real
    print("\nMostrando animación en tiempo real...")
    print(f"Framerate de simulación: {1/time_step:.1f} FPS")
    print(f"Frames totales: {len(trajectory)}")
    animate_trajectory(trajectory, targets, init_pos, target, time_step, skip_frames=10)
    
    
    
    
    
    