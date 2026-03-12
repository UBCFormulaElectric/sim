import numpy as np

from controller import ControllerOutput
from vehicle_state import VehicleState

def sim_step(state: VehicleState, control: ControllerOutput, dt: int):
	state.v_x += control.a_x * dt
	state.v_y += control.a_y * dt
	state.x += state.v_x * dt
	state.y += state.v_y * dt
	state.theta += (state.omega * dt) % (2 * np.pi)