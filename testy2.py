import pyvista as pv
import numpy as np

plotter = pv.Plotter()

class AppState:
    def __init__(self):
        self.is_running = True
state = AppState()
def close_callback(obj, event):
    print("Window closed, stopping animation loop.")
    state.is_running = False


plotter.iren.add_observer("ExitEvent", close_callback)

# 1. Definir el cubo de referencia (-10 a 10 en cada eje)
limit = 4
FLOOR = limit
my_bounds = [-limit, limit, 0, limit, -limit, limit]

plotter.camera.up = (0, 1, 0)
plotter.camera.focal_point = (0, 0, 0)
plotter.camera.position = (10, 10, 10)

# 2. Configurar el Grid con límites fijos
plotter.show_grid(
    bounds=my_bounds,
    location='outer', 
    ticks='both'
)
plotter.enable_terrain_style()
plotter.add_axes(interactive=True) 

#now add some objects to the scene
#floor box , Z = 0
floor = pv.Box(bounds=(-FLOOR, FLOOR, -0.1, 0.1, -FLOOR, FLOOR))
plotter.add_mesh(floor, color='lightgray', opacity=0.5)

#now add a ping pong table
TABLE_LENGTH = 2.74 #m
TABLE_WIDTH  = 1.525 #m
TABLE_HEIGHT = 0.05105 #m, this is the thickness of the table, not the height
TABLE_SURFACE_HEIGHT = 0.76 #m, this is the height of the playing surface from the ground

table = pv.Box(bounds=(-TABLE_LENGTH/2, TABLE_LENGTH/2, TABLE_SURFACE_HEIGHT - TABLE_HEIGHT, TABLE_SURFACE_HEIGHT, -TABLE_WIDTH/2, TABLE_WIDTH/2))
plotter.add_mesh(table, color='blue', opacity=1)


#Now add the net
NET_HEIGHT = 0.1525 #m
NET_THICK  = 0.01 #m
#it must be centered on the table, and also start at the surface of the table
net = pv.Box(bounds=(-NET_THICK/2, NET_THICK/2, TABLE_SURFACE_HEIGHT, TABLE_SURFACE_HEIGHT + NET_HEIGHT, -TABLE_WIDTH/2, TABLE_WIDTH/2))
plotter.add_mesh(net, color='red', opacity=1)

#Create a ball esphere with r = 0.02m (ping pong ball radius)
BALL_RADIUS = 0.5
ball = pv.Sphere(radius=BALL_RADIUS, center=(0, TABLE_SURFACE_HEIGHT + BALL_RADIUS, 0))
plotter.add_mesh(ball, color='white', opacity=1)


#now animate the ball using a simple cosine wave for the height, and a linear movement in x
plotter.show(interactive_update=True)
elapsed_time = 0
import time
try:
    while state.is_running:
        # Si el usuario cerró la ventana o el plotter ya no existe
        if plotter.renderer is None or plotter._closed:
            break
        elapsed_time += (1/60)
        #move the ball in x from -1 to 1 in 5 seconds
        ball_center_x = -1 + (elapsed_time / 5) * 2
        #make the ball bounce using a cosine wave, with a period of 2 seconds and an amplitude of 0.5m
        ball_center_z = TABLE_SURFACE_HEIGHT + BALL_RADIUS + 0.5 * np.cos(2 * np.pi * (elapsed_time / 2))
        ball.center = (ball_center_x, ball_center_z, 0)
        #print(f"Ball position: {ball.center}")
        time.sleep(1/60)  # Simulate a frame rate of ~60 FPS
        plotter.update()

except Exception as e:
    print(f"Error: {e}")

finally:
    plotter.close() 