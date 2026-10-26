import pygame
from Controller import Cone, ConeColor, VehicleState, mock_perception, compute_path, project, spline_t, init as controller_init, Parameters
from render import init, render_world
from control_plots import ControlPlots
from controller_thread import ControllerThread
from sim_thread import SimThread
from mdf_logger import MdfLogger
import ctypes
import math
import platform

# dpi awareness for windows - prevents blurry rendering on high dpi displays
if platform.system() == "Windows":
    try:
        # Set DPI Awareness (Windows 10/8)
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
    except AttributeError:
        # Windows 7/Vista
        ctypes.windll.user32.SetProcessDPIAware()

CONE_POSITIONS: list[Cone] = []
with open("labels/0000075.txt", "r") as f:
    lines = f.readlines()
    for line in lines:
        x, y, _, _, _, _, _, color = line.strip().split()
        cone_color = ConeColor.YELLOW if color == "Cone_Yellow" else ConeColor.BLUE
        CONE_POSITIONS.append(Cone(float(x), float(y), cone_color))

# pygame setup
pygame.init()
# window size is in OS points; shrink it to fit the desktop (e.g. a 1728x1117 point retina macbook)
desktop_w, desktop_h = pygame.display.get_desktop_sizes()[0]
window_size = (min(1900, int(desktop_w * 0.9)), min(1200, int(desktop_h * 0.9)))
# allow_high_dpi gives a framebuffer in physical pixels on retina macs instead of an upscaled (blurry) one
window = pygame.Window('Autonomous Sim', window_size, resizable=True, allow_high_dpi=True)
screen = window.get_surface()
clock = pygame.time.Clock()
running = True
ui_scale = screen.get_width() / window.size[0]  # pixels per point, 2.0 on retina
# ui_scale = 1.0
init(ui_scale)

vehicle_state: VehicleState = VehicleState()
# vehicle_state.v_y = 2.0
# vehicle_state.theta = math.radians(23.54)
# vehicle_state.omega = -0.2

def handle_key(key: int, state: VehicleState):
    match key:
        case pygame.K_d: state.y -= 1
        case pygame.K_a: state.y += 1
        case pygame.K_w: state.x += 1
        case pygame.K_s: state.x -= 1

ran = False
def simulate_cone_detection(state: VehicleState):
    global ran
    if not ran:
        mock_perception(CONE_POSITIONS)
        compute_path()
        ran = True
    return CONE_POSITIONS

# point the car along the center line, so that progress along the path means driving forward
simulate_cone_detection(vehicle_state)
start_t = project(vehicle_state.x, vehicle_state.y)
p0, p1 = spline_t(start_t), spline_t(start_t + 0.1)
vehicle_state.psi = math.atan2(p1[1] - p0[1], p1[0] - p0[0])

# tuned with tune.py for the 20 Hz controller and 5 ms sim step, with A_X ±5: ~14.9 s laps from 51 of the 54 start
# positions (standing starts from 30, 55 and 56 fail). with A_X ±10: ~13.9 s laps, but only from 34 of them
params = Parameters(
    deltaT_s=0.1,
    w_1=5.0,
    w_2=10.0,
    w_3=10.0,
    w_a_x=0.01,
    w_r_dot=0.01,
    w_delta=3.0,
    w_v_theta=0.01,
    w_prox=500.0,
    A_X_MIN=-10.0,
    A_X_MAX=10.0,
    R_DOT_MIN=-0.5,
    R_DOT_MAX=0.5,
    DELTA_MIN=-0.5,
    DELTA_MAX=0.5,
    V_THETA_MIN=0.0,
    V_THETA_MAX=100.0,
)
controller_init(params)
plots = ControlPlots(params, ui_scale)
logger = MdfLogger(params)
# the sim, the controller and drawing each run on their own thread, at their own rate
sim = SimThread(vehicle_state, step_ms=5, logger=logger)  # steps vehicle_state in real time
controller = ControllerThread(sim.snapshot, period_s=1.0/20.0, logger=logger)  # solves from the sim's state at 20 Hz
sim.start(lambda: controller.latest()[0])  # drives with the controller's most recent plan

# save the log even if the sim or controller thread fails, since that is when it is most useful
try:
    while running:
        # handle events
        # pygame.QUIT event means the user clicked X to close your window
        for event in pygame.event.get():
            if event.type in (pygame.QUIT, pygame.WINDOWCLOSE):
                running = False
            elif event.type == pygame.KEYDOWN:
                with sim.lock:
                    handle_key(event.key, vehicle_state)
        screen = window.get_surface()  # the surface is replaced when the window is resized

        # this is what is running on the 
        detected_cones = simulate_cone_detection(vehicle_state)
        # draw whatever the sim and controller threads are up to, so drawing never waits for (or slows down) either
        state, time_s = sim.snapshot()
        controls, trajectory = controller.latest()
        plots.push(time_s, controls, state)

        screen.fill("black")
        render_world(state, CONE_POSITIONS, screen, trajectory)
        plots.draw(screen)

        # flip() the display to put your work on screen
        window.flip()
        clock.tick(60)  # limits FPS to 60
finally:
    sim.stop()
    controller.stop()
    log_path = logger.save()
    if log_path is not None:
        print(f"logged to {log_path}")
    pygame.quit()
