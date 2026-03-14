import math

import pygame
from Controller import compute, sim_step, Cone, ConeColor, VehicleState
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
vehicle_state.v_x = 2.0
# vehicle_state.theta = math.radians(23.54)
vehicle_state.omega = -0.2

def handle_key(key: int, state: VehicleState):
	match key:
		case pygame.K_d:
			state.theta += 0.1
		case pygame.K_a:
			state.theta -= 0.1
		case pygame.K_w:
			state.x += 0.1
		case pygame.K_s:
			state.x -= 0.1

while running:
    # handle events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    dt = clock.get_time()
    # fill the screen with a color to wipe away anything from last frame
    screen.fill("black")
    controls = compute(vehicle_state, CONE_POSITIONS)
    sim_step(vehicle_state, controls, dt)
    render_world(vehicle_state, CONE_POSITIONS, screen)

    # flip() the display to put your work on screen
    pygame.display.flip()
    clock.tick(60)  # limits FPS to 60

pygame.quit()