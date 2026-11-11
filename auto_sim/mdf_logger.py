"""
Records the sim and the controller into an ASAM MDF4 (.mf4) file, readable by asammdf's GUI, CANape, MATLAB etc.

The two run at different rates, so each gets its own channel group, timestamped in sim time: "sim" has a sample every sim
step (the state after the step, and the control it was stepped with), "controller" has one per solve. Rows are kept in
memory while running (a few MB for a long run) and written out by save().
"""
import datetime
from pathlib import Path

import numpy as np
from asammdf import MDF, Signal

from Controller import ControlOutput, Parameters, VehicleState

# (name, unit, comment) of each column after the timestamp
SIM_CHANNELS = [
    ("x", "m", "global x position"),
    ("y", "m", "global y position"),
    ("psi", "rad", "heading"),
    ("v_x", "m/s", "body frame longitudinal velocity"),
    ("v_y", "m/s", "body frame lateral velocity"),
    ("r", "rad/s", "yaw rate"),
    ("speed", "m/s", "|v|"),
    ("a_x", "m/s^2", "applied longitudinal acceleration"),
    ("r_dot", "rad/s^2", "applied yaw acceleration"),
    ("delta", "rad", "applied steering angle"),
]
CONTROLLER_CHANNELS = [
    ("plan_a_x", "m/s^2", "planned longitudinal acceleration"),
    ("plan_r_dot", "rad/s^2", "planned yaw acceleration"),
    ("plan_delta", "rad", "planned steering angle"),
    ("dt", "s", "sim time since the previous solve, as passed to compute()"),
    ("solve_time", "s", "wall clock time compute() took"),
]


class MdfLogger:
    def __init__(self, params: Parameters):
        self.params = params
        self._start = datetime.datetime.now()
        # appended to from the sim and controller threads; list.append is atomic, so no lock is needed
        self._sim_rows: list[tuple[float, ...]] = []
        self._controller_rows: list[tuple[float, ...]] = []

    def log_sim_step(self, time_s: float, state: VehicleState, control: ControlOutput):
        self._sim_rows.append((time_s, state.x, state.y, state.psi, state.v_x, state.v_y, state.r,
                               float(np.hypot(state.v_x, state.v_y)), control.ax, control.omega_dot, control.delta))

    def log_solve(self, time_s: float, dt_s: float, solve_s: float, control: ControlOutput):
        self._controller_rows.append((time_s, control.ax, control.omega_dot, control.delta, dt_s, solve_s))

    def save(self, directory: str | Path = "logs") -> Path | None:
        """write everything logged so far to directory/<start time>.mf4 and return its path, or None if nothing was"""
        if not self._sim_rows and not self._controller_rows:
            return None
        mdf = MDF(version="4.10")
        mdf.header.comment = "Parameters: " + ", ".join(
            f"{name}={getattr(self.params, name)}" for name in dir(self.params) if not name.startswith("_"))
        for group, rows, channels in (("sim", self._sim_rows, SIM_CHANNELS),
                                      ("controller", self._controller_rows, CONTROLLER_CHANNELS)):
            if not rows:
                continue
            data = np.array(rows)
            mdf.append([Signal(data[:, i + 1], data[:, 0], name=name, unit=unit, comment=comment)
                        for i, (name, unit, comment) in enumerate(channels)], comment=group)

        path = Path(directory) / f"{self._start:%Y%m%d-%H%M%S}.mf4"
        path.parent.mkdir(parents=True, exist_ok=True)
        mdf.save(path, overwrite=True)
        return path
