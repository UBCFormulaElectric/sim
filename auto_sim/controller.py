from dataclasses import dataclass
from cone import Cone
from vehicle_state import VehicleState
import pygame

@dataclass
class ControllerOutput:
	a_x: float
	a_y: float
	omega_dot: float

def controller(state: VehicleState, cones: list[Cone]) -> ControllerOutput:
	keys = pygame.key.get_pressed()
	return ControllerOutput(
		keys[pygame.K_w] * 0.0001 - keys[pygame.K_s] * 0.0001,
		keys[pygame.K_d] * 0.0001 - keys[pygame.K_a] * 0.0001,
		0
	)