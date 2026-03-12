from dataclasses import dataclass

@dataclass
class VehicleState:
	x: float
	y: float
	v_x: float
	v_y: float
	theta: float
	omega: float