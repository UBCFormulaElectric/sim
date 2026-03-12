from dataclasses import dataclass
import numpy as np

@dataclass
class VehicleState:
	x: float
	y: float
	v_x: float
	v_y: float
	theta: float
	omega: float

# (VEHICLE_X_M, VEHICLE_Y_M) = 0.0, 0.0 # position
# (VEHICLE_V_X_MPS, VEHICLE_V_Y_MPS) = 0.0, 0.0 # velocity
# (VEHICLE_THETA_RAD, VEHICLE_OMEGA_RAD_PER_SEC) = 0.0, 0.0 # orientation and angular velocity

def sim_step(state: VehicleState, dt: int):
	state.x += state.v_x * dt
	state.y += state.v_y * dt
	state.theta += (state.omega * dt) % (2 * np.pi)