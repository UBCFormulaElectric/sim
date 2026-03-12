# global state
from dataclasses import dataclass
from enum import Enum

import numpy as np
import pygame
from scipy.spatial.transform import Rotation as R

class ConeColor(Enum):
	BLUE = "blue"
	YELLOW = "yellow"

@dataclass
class Cone:
	x: int
	y: int
	color: ConeColor

CONE_POSITIONS: list[Cone] = [
	Cone(10, 10, ConeColor.BLUE),
	Cone(-10, 10, ConeColor.YELLOW),
] # TODO figure out where the cones need to be
(VEHICLE_X_M, VEHICLE_Y_M) = 0.0, 0.0 # position
(VEHICLE_V_X_MPS, VEHICLE_V_Y_MPS) = 0.0, 0.0 # velocity
(VEHICLE_THETA_RAD, VEHICLE_OMEGA_RAD_PER_SEC) = 0.0, 0.0 # orientation and angular velocity
PIXELS_PER_M = 80.0
w: int
h: int

car: pygame.Surface

def init():
	global car
	car = pygame.image.load('fsae.png').convert_alpha()

def handle_key(key: int):
	pass

def sim_step(dt: int):
	global VEHICLE_X_M, VEHICLE_Y_M, VEHICLE_V_X_MPS, VEHICLE_V_Y_MPS, VEHICLE_THETA_RAD, VEHICLE_OMEGA_RAD_PER_SEC
	VEHICLE_X_M += VEHICLE_V_X_MPS * dt
	VEHICLE_Y_M += VEHICLE_V_Y_MPS * dt
	VEHICLE_THETA_RAD += (VEHICLE_OMEGA_RAD_PER_SEC * dt) % (2 * np.pi)

def transform(x: float, y: float) -> tuple[int, int]:
	global w, h, VEHICLE_X_M, VEHICLE_Y_M, VEHICLE_THETA_RAD
	r = R.from_euler('z', -VEHICLE_THETA_RAD, degrees=False)
	rel_x, rel_y = x - VEHICLE_X_M, y - VEHICLE_Y_M
	rotated_x, rotated_y,_ = r.apply([rel_x, rel_y, 0])
	screen_x = int(w / 2 + rotated_x * PIXELS_PER_M)
	screen_y = int((h * 0.67) - rotated_y * PIXELS_PER_M)
	# Simple transformation - replace with actual coordinate transformation logic
	return (screen_x, screen_y)

def render_world(screen: pygame.Surface):
	global w, h
	w, h = screen.get_width(), screen.get_height()

	# draw the car at 1/10th scale
	car_rotated = pygame.transform.rotate(car, -90)
	scale_factor = 1.2 / car_rotated.get_width() * PIXELS_PER_M
	car_scaled = pygame.transform.scale(car_rotated, (
		car_rotated.get_width() * scale_factor, car_rotated.get_height() * scale_factor
	))
	car_rect = car_scaled.get_rect(center=transform(VEHICLE_X_M, VEHICLE_Y_M))
	screen.blit(car_scaled, car_rect)

	for cone in CONE_POSITIONS:
		pygame.draw.circle(
			screen, cone.color.value,
			transform(cone.x, cone.y), 10
		)