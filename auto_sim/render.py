import pygame
from scipy.spatial.transform import Rotation as R
from constants import VEHICLE_WIDTH_M
from Controller import VehicleState
from math import ceil, degrees
import numpy as np
from Controller import Cone, ConeColor

PIXELS_PER_M = 	50.0
w: int
h: int

car: pygame.Surface

def init():
	global car
	car = pygame.image.load('fsae.png').convert_alpha()

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

def transform(x: float, y: float, vehicle_state: VehicleState) -> tuple[int, int]:
	"""
	Transforms from global coordinates to screen space coordinates
	"""
	global w, h
	r = R.from_euler('z', -vehicle_state.theta + np.pi/2, degrees=False)
	rel_x, rel_y = x - vehicle_state.x, y - vehicle_state.y
	rotated_x, rotated_y,_ = r.apply([rel_x, rel_y, 0])
	screen_x = int(w / 2 + rotated_x * PIXELS_PER_M)
	screen_y = int((h * 0.67) - rotated_y * PIXELS_PER_M)
	# Simple transformation - replace with actual coordinate transformation logic
	return (screen_x, screen_y)

def create_surface(l: int, spacing_px: int, color: str) -> pygame.Surface:
	grid_surf = pygame.Surface((2*l,2*l), pygame.SRCALPHA)
	for x in range(0, 2*l, spacing_px):
		pygame.draw.line(grid_surf, color, (x, 0), (x, 2*l))
	for y in range(0, 2*l, spacing_px):
		pygame.draw.line(grid_surf, color, (0, y), (2*l, y))
	return grid_surf

old_state = None, None, None
old_grid_surf = None
def drawGrid(screen: pygame.Surface, state: VehicleState, color=(45, 45, 45), spacing_m=1):
	global h, w, old_state, old_grid_surf
	spacing_px =  int(spacing_m * PIXELS_PER_M)
	l: int= ceil(np.hypot(w/2, h/2) / spacing_m) * spacing_m
	if (spacing_m, color, l) != old_state or old_grid_surf is None:
		print("RERENDERING GRID")
		old_state = (spacing_m, color, l)
		old_grid_surf = create_surface(2*l, spacing_px, color)
	grid_surf = old_grid_surf

	# Calculate grid offset to align with world coordinates
	r = R.from_euler('z', -state.theta + np.pi/2, degrees=False)
	x, y = r.apply([state.x % spacing_m, state.y % spacing_m, 0])[:2]
	rotated_surf = pygame.transform.rotate(grid_surf, degrees(-state.theta - np.pi/2))
	rect = rotated_surf.get_rect(center=(
		w/2 - x * PIXELS_PER_M,
		0.67 * h + y * PIXELS_PER_M
	))
	screen.blit(rotated_surf, rect.topleft)

def int_to_color(value: ConeColor) -> str:
	match value:
		case ConeColor.BLUE:
			return "blue"
		case ConeColor.YELLOW:
			return "yellow"
		case _:
			return "white"

def render_world(vehicle_state: VehicleState, cones: list[Cone], screen: pygame.Surface):
	global w, h
	w, h = screen.get_width(), screen.get_height()

	# grid for context
	drawGrid(screen, vehicle_state)

	p = transform(0, 0, vehicle_state)
	pygame.draw.circle(screen, "white",p , 5)

	for cone in cones:
		pygame.draw.circle(
			screen, int_to_color(cone.c),
			transform(cone.x, cone.y, vehicle_state), 0.2 * PIXELS_PER_M
		)
	
	car_rotated = pygame.transform.rotate(car,-90)
	scale_factor = VEHICLE_WIDTH_M / car_rotated.get_width() * PIXELS_PER_M
	car_scaled = pygame.transform.scale(car_rotated, (
		car_rotated.get_width() * scale_factor, car_rotated.get_height() * scale_factor
	))
	car_rect = car_scaled.get_rect(center=transform(vehicle_state.x, vehicle_state.y, vehicle_state))
	screen.blit(car_scaled, car_rect)