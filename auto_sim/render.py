
import pygame
from scipy.spatial.transform import Rotation as R
from constants import VEHICLE_WIDTH_M
from Controller import VehicleState, Cone, ConeColor, get_center_points, project, spline_t
from math import ceil, degrees
import numpy as np

PIXELS_PER_M = 	20.0
w: int
h: int

car: pygame.Surface
vignette: pygame.Surface

def init():
	global car, vignette
	car = pygame.image.load('fsae.png').convert_alpha()
	vignette_image = pygame.image.load('vignette.png').convert_alpha()
	vignette_image = pygame.transform.scale(vignette_image, (30 * PIXELS_PER_M,30 * PIXELS_PER_M))
	vignette = pygame.Surface((3000,3000), pygame.SRCALPHA)
	vignette.blit(vignette_image, vignette_image.get_rect(center=(1500,1500)))

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

def create_grid_surface(l: int, spacing_px: int, color: str) -> pygame.Surface:
	grid_surf = pygame.Surface((2*l,2*l), pygame.SRCALPHA)
	for x in range(0, 2*l, spacing_px):
		pygame.draw.line(grid_surf, color, (x, 0), (x, 2*l), 2)
	for y in range(0, 2*l, spacing_px):
		pygame.draw.line(grid_surf, color, (0, y), (2*l, y), 2)
	return grid_surf

old_state = None, None, None
old_grid_surf = None
def drawGrid(screen: pygame.Surface, state: VehicleState, color=(45, 45, 45), spacing_m=1):
	global h, w, old_state, old_grid_surf, vignette
	# create grid, cached
	spacing_px =  int(spacing_m * PIXELS_PER_M)
	l: int= ceil(np.hypot(w/2, h/2) * 0.4 / spacing_m) * spacing_m
	if (spacing_m, color, l) != old_state or old_grid_surf is None:
		print("RERENDERING GRID")
		old_state = (spacing_m, color, l)
		old_grid_surf = create_grid_surface(2*l, spacing_px, color)
	grid_surf = old_grid_surf

	# grid
	screen_grid = pygame.surface.Surface((w, h), pygame.SRCALPHA)

	# Calculate grid offset to align with world coordinates
	r = R.from_euler('z', -state.theta + np.pi/2, degrees=False)
	x, y = r.apply([state.x % spacing_m, state.y % spacing_m, 0])[:2]
	rotated_surf: pygame.Surface = pygame.transform.rotate(grid_surf, degrees(-state.theta - np.pi/2))
	screen_grid.blit(rotated_surf, rotated_surf.get_rect(center=(
		w/2 - x * PIXELS_PER_M,
		0.67 * h + y * PIXELS_PER_M
	)).topleft)

	# vignette
	screen_grid.blit(vignette, vignette.get_rect(center=(w/2, 0.67*h)), special_flags=pygame.BLEND_RGBA_MULT)

	screen.blit(screen_grid, (0, 0))


# def create_spline_surface():
# 	surf = pygame.Surface((77 * PIXELS_PER_M, 47 * PIXELS_PER_M), pygame.SRCALPHA)
# 	length = get_center_line_length()
# 	for x in range(int(77 * PIXELS_PER_M)):
# 		for y in range(int(47 * PIXELS_PER_M)):
# 			k = hsv_to_rgb(project(22-x/PIXELS_PER_M, -39+y/PIXELS_PER_M)/length, 1, 1)
# 			surf.set_at((x, y), (int(k[0]*255), int(k[1]*255), int(k[2]*255)))
# 	return surf

# spline_surface = None
# def drawSplintSurface(screen: pygame.Surface, vehicle_state: VehicleState):
# 	global spline_surface	
# 	if spline_surface is None:
# 		spline_surface = create_spline_surface()
# 	rotated_surf: pygame.Surface = pygame.transform.rotate(spline_surface, degrees(-vehicle_state.theta - np.pi/2))
# 	screen.blit(rotated_surf, rotated_surf.get_rect(center=transform(-16.5, -15.5, vehicle_state)))
	

def int_to_color(value: ConeColor) -> str:
	match value:
		case ConeColor.BLUE:
			return "blue"
		case ConeColor.YELLOW:
			return "yellow"
		case _:
			return "white"

at_t = None
def render_world(vehicle_state: VehicleState, cones: list[Cone], screen: pygame.Surface):
	global w, h, at_t
	w, h = screen.get_width(), screen.get_height()

	# grid for context
	drawGrid(screen, vehicle_state)
	# drawSplintSurface(screen, vehicle_state)

	# pygame.draw.circle(screen, "white", transform(0, 0, vehicle_state), 5)

	# DRAW TRIANGULATION
	# for edge in get_offline_edges():
	# 	v1, v2 = cones[edge.v1()], cones[edge.v2()]
	# 	default_colour = "#676767"
	# 	pygame.draw.line(screen, default_colour,
	# 		transform(v1.x, v1.y, vehicle_state),
	# 		transform(v2.x, v2.y, vehicle_state), 2
	# 	)
	# for edge in get_boundary_edges():
	# 	v1, v2 = cones[edge.v1()], cones[edge.v2()]
	# 	default_colour = "#FF0000"
	# 	if v1.c == v2.c:
	# 		match v1.c:
	# 			case ConeColor.BLUE:
	# 				default_colour = "blue"
	# 			case ConeColor.YELLOW:
	# 				default_colour = "yellow"
	# 	pygame.draw.line(screen, default_colour,
	# 		transform(v1.x, v1.y, vehicle_state),
	# 		transform(v2.x, v2.y, vehicle_state), 1
	# 	)

	# DRAW CENTER LINE
	# center_points = get_center_points()
	# for point in center_points:
	# 	pygame.draw.circle(screen, "white", transform(point.x, point.y, vehicle_state), 2)
	# for i in range(len(center_points)):
	# 	v1, v2 = center_points[i], center_points[(i+1) % len(center_points)] # returns cone objects
	# 	default_colour = "#676767"
	# 	pygame.draw.line(screen, default_colour,
	# 		transform(v1.x, v1.y, vehicle_state),
	# 		transform(v2.x, v2.y, vehicle_state), 2
	# 	)

	# DRAW PATH PROJECTION
	# at_t = project(vehicle_state.x, vehicle_state.y)
	# x = spline_t(at_t)
	# pygame.draw.circle(screen, "red", transform(x.x, x.y, vehicle_state), 5)

	# DRAW SPLINE
	# steps = 100
	# dt = get_center_line_length() / steps
	# for t in range(steps):
	# 	x1 = spline_t(t * dt)
	# 	x2= spline_t((t + 1) * dt)
	# 	pygame.draw.line(screen, "#676767",
	# 			   transform(x1.x, x1.y, vehicle_state),
	# 			   transform(x2.x, x2.y, vehicle_state), 2)

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