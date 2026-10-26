"""
Steps the vehicle dynamics on its own thread, in real time, with the controller's most recent control, so the physics
neither depends on the frame rate nor stalls when drawing does. The draw loop and the controller read copies of the
state via snapshot().
"""
import threading
import time
from typing import Callable

from Controller import ControlOutput, VehicleState, sim_step
from mdf_logger import MdfLogger

MAX_LAG_S = 0.1  # if stepping falls further behind real time than this (e.g. a stall), skip ahead instead of catching up


class SimThread:
    def __init__(self, state: VehicleState, step_ms: int, logger: MdfLogger | None = None):
        """
        steps `state` by step_ms (sim_step takes whole ms) every step_ms of wall clock time, once start()ed, logging each
        step to `logger`. anything else that changes the state must hold `lock` while doing so
        """
        self.state = state
        self.time_s = 0.0
        self.lock = threading.Lock()
        self._step_ms = step_ms
        self._logger = logger
        self._control: Callable[[], ControlOutput] | None = None
        self._error: BaseException | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sim", daemon=True)

    def start(self, control: Callable[[], ControlOutput]):
        """start stepping, applying control() at each step"""
        self._control = control
        self._thread.start()

    def snapshot(self) -> tuple[VehicleState, float]:
        """a copy of the vehicle state, and its sim time"""
        with self.lock:
            if self._error is not None:
                raise RuntimeError("sim thread failed") from self._error
            return VehicleState(self.state), self.time_s

    def stop(self):
        self._stop.set()
        self._thread.join()

    def _run(self):
        step_s = self._step_ms / 1000
        try:
            self._control()  # wait for the first control (e.g. the controller's first solve) before starting the clock
            next_s = time.monotonic()
            while not self._stop.wait(max(0.0, next_s - time.monotonic())):
                next_s = max(next_s + step_s, time.monotonic() - MAX_LAG_S)
                control = self._control()
                with self.lock:
                    sim_step(self.state, control, self._step_ms)
                    self.time_s += step_s
                    if self._logger is not None:
                        self._logger.log_sim_step(self.time_s, self.state, control)
        except BaseException as e:
            # hand the error to whoever reads the state next, instead of silently freezing the car
            with self.lock:
                self._error = e
