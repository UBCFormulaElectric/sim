import pygame
from cone import Cone, ConeColor
from controller import controller
from render import handle_key, init, render_world
from sim import VehicleState, sim_step
import ctypes
import platform

def set_dpi_awareness():
    if platform.system() == "Windows":
        try:
            # Set DPI Awareness (Windows 10/8)
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
        except AttributeError:
            # Windows 7/Vista
            ctypes.windll.user32.SetProcessDPIAware()
set_dpi_awareness()

CONE_POSITIONS: list[Cone] = [
	Cone(10, 10, ConeColor.BLUE),
	Cone(-10, 10, ConeColor.YELLOW),
] # TODO figure out where the cones need to be

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1900, 1200), pygame.RESIZABLE)
pygame.display.set_caption('Autonomous Sim')
clock = pygame.time.Clock()
running = True
time: float = 0.0
init()

vehicle_state: VehicleState = VehicleState(0, 0, 0, 0, 0, 0)

while running:
    # handle events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            handle_key(event.key, vehicle_state)

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("black")
    controls = controller(vehicle_state, CONE_POSITIONS)
    sim_step(vehicle_state, controls, clock.get_time())
    render_world(vehicle_state, CONE_POSITIONS, screen)

    # flip() the display to put your work on screen
    pygame.display.flip()
    clock.tick(60)  # limits FPS to 60

pygame.quit()