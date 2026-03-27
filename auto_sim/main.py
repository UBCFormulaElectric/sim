import pygame
from Controller import compute, sim_step, Cone, ConeColor, VehicleState, compute_path
from render import init, render_world
import ctypes
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
with open("labels/0000001.txt", "r") as f:
    lines = f.readlines()
    for line in lines:
        x, y, _, _, _, _, _, color = line.strip().split()
        cone_color = ConeColor.YELLOW if color == "Cone_Yellow" else ConeColor.BLUE
        CONE_POSITIONS.append(Cone(float(x), float(y), cone_color))

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1900, 1200), pygame.RESIZABLE)
pygame.display.set_caption('Autonomous Sim')
clock = pygame.time.Clock()
running = True
time: float = 0.0
init()

vehicle_state: VehicleState = VehicleState()
# vehicle_state.v_y = 2.0
# vehicle_state.theta = math.radians(23.54)
# vehicle_state.omega = -0.2

def handle_key(key: int, state: VehicleState):
	match key:
		case pygame.K_d:
			state.y -= 1
		case pygame.K_a:
			state.y += 1
		case pygame.K_w:
			state.x += 1
		case pygame.K_s:
			state.x -= 1

ran = False
def simulate_cone_detection(state: VehicleState):
    global ran
    if not ran:
        compute_path(CONE_POSITIONS)
        ran = True
    return CONE_POSITIONS

while running:
    # handle events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            handle_key(event.key, vehicle_state)

    # this is what is running on the 
    detected_cones = simulate_cone_detection(vehicle_state)
    controls = compute(vehicle_state)

    # closing the SIL loop
    dt = clock.get_time()
    screen.fill("black")
    sim_step(vehicle_state, controls, dt)
    render_world(vehicle_state, CONE_POSITIONS, screen)

    # flip() the display to put your work on screen
    pygame.display.flip()
    clock.tick(60)  # limits FPS to 60

pygame.quit()