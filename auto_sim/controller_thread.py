"""
Runs the controller's compute() on its own thread, on its own clock: every period it snapshots the shared vehicle state
and solves from it, so neither the sampling rate nor a slow solve depend on the draw loop. The draw loop drives with the
most recent plan, holding it until the next one is ready (like the real car, where the controller runs at its own rate).
"""
import threading
import time

from Controller import ControlOutput, VehicleState, compute, get_control, get_trajectory


class ControllerThread:
    def __init__(self, state: VehicleState, period_s: float):
        """
        solves from `state` every period_s seconds (wall clock). the sim keeps stepping the same state: do that while
        holding `lock`, and set `time_s` to its new sim time, so a snapshot is never of a half updated state
        """
        self.state = state
        self.time_s = 0.0
        self.lock = threading.Lock()
        self._period_s = period_s
        self._cond = threading.Condition()
        self._plan: tuple[ControlOutput, list[VehicleState]] | None = None
        self._error: BaseException | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="controller", daemon=True)
        self._thread.start()

    def latest(self) -> tuple[ControlOutput, list[VehicleState]]:
        """the next control and the predicted trajectory of the most recent plan. only blocks until the first plan"""
        with self._cond:
            self._cond.wait_for(lambda: self._plan is not None or self._error is not None)
            if self._error is not None:
                raise RuntimeError("controller thread failed") from self._error
            return self._plan

    def stop(self):
        self._stop.set()
        self._thread.join()

    def _run(self):
        prev_time_s = None
        next_s = time.monotonic()
        try:
            while not self._stop.wait(max(0.0, next_s - time.monotonic())):
                next_s = max(next_s + self._period_s, time.monotonic())  # fixed rate, unless a solve overran it
                with self.lock:
                    state, time_s = VehicleState(self.state), self.time_s
                # compute() releases the GIL while solving. its dt is the sim time since the previous snapshot
                dt_s = 0.0 if prev_time_s is None else time_s - prev_time_s
                prev_time_s = time_s
                output = compute(state, dt_s)
                plan = (get_control(output), get_trajectory(output))
                with self._cond:
                    self._plan = plan
                    self._cond.notify_all()
        except BaseException as e:
            # hand the error to the draw loop, instead of silently driving with the last plan forever
            with self._cond:
                self._error = e
                self._cond.notify_all()
