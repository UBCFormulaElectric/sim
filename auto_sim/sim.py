import numpy as np

from controller import ControllerOutput
from Controller import VehicleState

def sim_step(state: VehicleState, control: ControllerOutput, dt: int):
	state.v_x += control.ax * dt
	state.v_y += control.ay * dt
	state.x += state.v_x * dt
	state.y += state.v_y * dt
	state.theta += (state.omega * dt) % (2 * np.pi)