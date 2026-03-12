# global state
from dataclasses import dataclass
from enum import Enum

import pygame
from scipy.spatial.transform import Rotation as R
from cone import Cone
from sim import VehicleState

PIXELS_PER_M = 80.0
w: int
h: int

car: pygame.Surface

def init():
	global car
	car = pygame.image.load('fsae.png').convert_alpha()

def handle_key(key: int):
	pass

def transform(x: float, y: float, vehicle_state: VehicleState) -> tuple[int, int]:
	global w, h
	r = R.from_euler('z', -vehicle_state.theta, degrees=False)
	rel_x, rel_y = x - vehicle_state.x, y - vehicle_state.y
	rotated_x, rotated_y,_ = r.apply([rel_x, rel_y, 0])
	screen_x = int(w / 2 + rotated_x * PIXELS_PER_M)
	screen_y = int((h * 0.67) - rotated_y * PIXELS_PER_M)
	# Simple transformation - replace with actual coordinate transformation logic
	return (screen_x, screen_y)

def drawGrid(screen: pygame.Surface, color=(20, 20, 20)):
	global h, w
	blockSize = 20 #Set the size of the grid block
	for x in range(0, w, blockSize):
		for y in range(0, h, blockSize):
			rect = pygame.Rect(x, y, blockSize, blockSize)
			pygame.draw.rect(screen, color, rect, 1)

def render_world(vehicle_state: VehicleState, cones: list[Cone], screen: pygame.Surface):
	global w, h
	w, h = screen.get_width(), screen.get_height()

	drawGrid(screen)

	# draw the car at 1/10th scale
	car_rotated = pygame.transform.rotate(car, -90)
	scale_factor = 1.2 / car_rotated.get_width() * PIXELS_PER_M
	car_scaled = pygame.transform.scale(car_rotated, (
		car_rotated.get_width() * scale_factor, car_rotated.get_height() * scale_factor
	))
	car_rect = car_scaled.get_rect(center=transform(vehicle_state.x, vehicle_state.y, vehicle_state))
	screen.blit(car_scaled, car_rect)

	for cone in cones:
		pygame.draw.circle(
			screen, cone.color.value,
			transform(cone.x, cone.y, vehicle_state), 10
		)