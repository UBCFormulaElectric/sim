"""
Rolling plots of the controller's outputs and the vehicle's speed, rendered with matplotlib and blitted onto the pygame
window.

The axes never change (time is plotted relative to now, each control's y range comes from its bounds in Parameters, and
speed, which has no bound there, gets a fixed range), so the static parts are rendered once and each frame only redraws
the lines on top. A full matplotlib redraw every frame would take tens of ms, slowing the frame rate and so changing the
dt the controller and sim run at.
"""
from collections import deque

import numpy as np
import pygame
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from Controller import ControlOutput, Parameters, VehicleState

HISTORY_S = 10.0
SIZE_PT = (410, 610)  # panel size in OS points
MARGIN_PT = 16
SPEED_MAX_MPS = 25.0  # top of the speed plot's y range

TEXT_PRIMARY = "#f0efec"
TEXT_SECONDARY = "#c3c2b7"
TEXT_MUTED = "#898781"
GRID = "#2c2c2a"
BASELINE = "#383835"
SERIES = "#3987e5"


class ControlPlots:
    def __init__(self, params: Parameters, ui_scale: float = 1.0):
        # (title, unit, bounds, value format) in the same order as the values in each history entry
        channels = [
            ("Acceleration $a_x$", "m/s²", (params.A_X_MIN, params.A_X_MAX), "+.3f"),
            ("Yaw acceleration $\\dot{r}$", "rad/s²", (params.R_DOT_MIN, params.R_DOT_MAX), "+.3f"),
            ("Steering $\\delta$", "rad", (params.DELTA_MIN, params.DELTA_MAX), "+.3f"),
            ("Speed $|v|$", "m/s", (0.0, SPEED_MAX_MPS), ".2f"),
        ]
        self.history: deque[tuple[float, float, float, float, float]] = deque()
        self.ui_scale = ui_scale

        # at 72 dpi a matplotlib point is one OS point, so font sizes and line widths match the rest of the UI
        self.figure = Figure(figsize=(SIZE_PT[0] / 72, SIZE_PT[1] / 72), dpi=72 * ui_scale, layout="constrained")
        self.figure.patch.set_facecolor("none")
        self.canvas = FigureCanvasAgg(self.figure)
        axes = self.figure.subplots(len(channels), 1, sharex=True)

        self.lines = []
        self.dots = []
        self.values = []
        self.formats = [fmt for *_, fmt in channels]
        for ax, (title, unit, (lo, hi), _) in zip(axes, channels):
            ax.set_facecolor("none")
            ax.xaxis.set_visible(False)
            ax.set_title(f"{title}  ({unit})", loc="left", fontsize=11, color=TEXT_PRIMARY)
            for spine in ax.spines.values():
                spine.set_visible(False)
            ax.tick_params(colors=TEXT_MUTED, labelsize=9, length=0)

            # gridlines at the bounds show when the control saturates
            ticks = sorted({lo, hi} | ({0.0} if lo <= 0 <= hi else set()))
            pad = 0.15 * (hi - lo) if hi > lo else 1.0
            ax.set_ylim(lo - pad, hi + pad)
            ax.set_yticks(ticks)
            ax.grid(axis="y", color=GRID, linewidth=1)
            if lo < 0 < hi:
                ax.axhline(0, color=BASELINE, linewidth=1)

            # animated artists are skipped by canvas.draw(), so they stay out of the cached background
            line, = ax.plot([], [], color=SERIES, linewidth=2, solid_joinstyle="round", solid_capstyle="round",
                            animated=True)
            dot, = ax.plot([], [], "o", markersize=8, color=SERIES, markeredgewidth=0, animated=True)
            # sits on the same baseline as the title, at the right edge
            value = ax.annotate("", xy=(1, 1), xycoords="axes fraction", xytext=(0, 6), textcoords="offset points",
                                ha="right", va="baseline", fontsize=11, color=TEXT_SECONDARY, animated=True)
            self.lines.append(line)
            self.dots.append(dot)
            self.values.append(value)

        # a little room right of now so the end dot isn't cut in half
        axes[-1].set_xlim(-HISTORY_S, 0.03 * HISTORY_S)

        self.canvas.draw()
        self.background = self.canvas.copy_from_bbox(self.figure.bbox)

    def push(self, time_s: float, control: ControlOutput, state: VehicleState):
        speed = np.hypot(state.v_x, state.v_y)
        self.history.append((time_s, control.ax, control.omega_dot, control.delta, speed))
        while self.history[0][0] < time_s - HISTORY_S:
            self.history.popleft()

    def draw(self, screen: pygame.Surface):
        if not self.history:
            return
        data = np.array(self.history)
        t = data[:, 0] - data[-1, 0]

        self.canvas.restore_region(self.background)
        for i, (line, dot, value, fmt) in enumerate(zip(self.lines, self.dots, self.values, self.formats)):
            y = data[:, i + 1]
            line.set_data(t, y)
            dot.set_data([0.0], [y[-1]])
            value.set_text(format(y[-1], fmt))
            for artist in (line, dot, value):
                self.figure.draw_artist(artist)

        buffer = self.canvas.buffer_rgba()
        # blending straight from matplotlib's RGBA byte order is ~7x slower than converting to the display's format first
        surface = pygame.image.frombuffer(buffer, (buffer.shape[1], buffer.shape[0]), "RGBA").convert_alpha()
        margin = MARGIN_PT * self.ui_scale
        screen.blit(surface, (screen.get_width() - surface.get_width() - margin, margin))
