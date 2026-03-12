from dataclasses import dataclass
from cone import Cone
from vehicle_state import VehicleState

@dataclass
class ControllerOutput:
	a_x: float
	a_y: float
	omega_dot: float

def controller(state: VehicleState, cones: list[Cone]) -> ControllerOutput:
	# TODO implement controller logic
	return ControllerOutput(0, 0, 0)