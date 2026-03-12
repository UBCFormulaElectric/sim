# Example file showing a basic pygame "game loop"
import pygame
from render import handle_key, init, render_world, sim_step
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

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1900, 1200), pygame.RESIZABLE)
pygame.display.set_caption('Autonomous Sim')
clock = pygame.time.Clock()
running = True
time: float = 0.0
init()

while running:
    # handle events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            handle_key(event.key)

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("black")
    sim_step(clock.get_time())
    render_world(screen)

    # flip() the display to put your work on screen
    pygame.display.flip()
    clock.tick(60)  # limits FPS to 60

pygame.quit()