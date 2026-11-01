# Copyright 2026 Pavan Patil
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Regenerate the figures in docs/images from the package's own maths.

Run from the repository root (no ROS needed, only numpy, scipy and matplotlib):

    python3 docs/scripts/make_figures.py
"""

import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'src', 'trajectory_nav'))

from trajectory_nav.path_math import (  # noqa: E402
    cumulative_arc_length, generate_waypoints, resample_path, smooth_path, wrap_angle)
from trajectory_nav.pure_pursuit import PurePursuit  # noqa: E402
from trajectory_nav.simulation import cross_track_error, simulate  # noqa: E402

OUT = os.path.join(ROOT, 'docs', 'images')
SEED = 3

# Reference palette (light): categorical slots 1-3, plus neutral ink for context.
BLUE, ORANGE, AQUA = '#2a78d6', '#eb6834', '#1baf7a'
SURFACE, INK, INK_2, MUTED, GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#9a9893', '#e4e3df'

plt.rcParams.update({
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
    'axes.edgecolor': MUTED, 'axes.labelcolor': INK_2, 'axes.titlecolor': INK,
    'axes.titlesize': 12, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left',
    'axes.labelsize': 10, 'xtick.color': INK_2, 'ytick.color': INK_2,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.8,
    'axes.spines.top': False, 'axes.spines.right': False,
    'legend.frameon': False, 'legend.fontsize': 9, 'font.size': 10,
    'lines.linewidth': 2.0, 'figure.dpi': 110, 'savefig.dpi': 160,
})


def example_path():
    """Return the seeded waypoints, smoothed path and trajectory used across the figures."""
    waypoints = generate_waypoints(0.0, 0.0, 0.0, rng=np.random.default_rng(SEED))
    sx, sy = smooth_path(waypoints[:, 0], waypoints[:, 1])
    tx, ty = resample_path(sx, sy, 0.05)
    return waypoints, (sx, sy), (tx, ty)


def save(fig, name):
    """Write a figure to docs/images and close it."""
    fig.savefig(os.path.join(OUT, name), bbox_inches='tight')
    plt.close(fig)
    print('wrote', name)


def figure_smoothing():
    """Raw waypoints against the arc-length cubic spline through them."""
    waypoints, (sx, sy), (tx, ty) = example_path()
    fig, ax = plt.subplots(figsize=(8, 3.6))
    ax.plot(waypoints[:, 0], waypoints[:, 1], color=MUTED, linewidth=1.2, linestyle='--',
            label='Raw waypoints (straight segments)')
    ax.plot(waypoints[:, 0], waypoints[:, 1], 'o', color=INK_2, markersize=7)
    ax.plot(sx, sy, color=BLUE, label='Smoothed path (cubic spline)')
    ax.plot(tx[::8], ty[::8], 'o', color=AQUA, markersize=4,
            label='Trajectory samples (every 8th of 5 cm)')
    for i, (wx, wy) in enumerate(waypoints):
        ax.annotate(f'W{i}', (wx, wy), textcoords='offset points', xytext=(0, 9),
                    ha='center', color=INK_2, fontsize=9)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_ylim(waypoints[:, 1].min() - 0.4, waypoints[:, 1].max() + 0.5)
    ax.set_title('Waypoints to smooth path to trajectory')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.22), ncol=3)
    save(fig, 'smoothing.png')


def old_resample(x, y, spacing):
    """Return the original 0.0.0 resampler's output, kept here only to plot the H2 bug."""
    out = [(x[0], y[0])]
    for i in range(len(x) - 1):
        dx, dy = x[i + 1] - x[i], y[i + 1] - y[i]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue
        n = max(1, int(math.ceil(length / spacing)))
        for j in range(1, n + 1):
            px, py = x[i] + j / n * dx, y[i] + j / n * dy
            if math.hypot(px - out[-1][0], py - out[-1][1]) >= 0.9 * spacing:
                out.append((px, py))
    return np.array(out)


def figure_resampling():
    """Endpoint-drop rate and spacing error of the old and new resamplers over random paths."""
    spacings = [0.05, 0.1, 0.2, 0.3]
    old_drop, new_drop, old_err, new_err = [], [], [], []
    for spacing in spacings:
        rng = np.random.default_rng(SEED)
        drops = [0, 0]
        errors = [[], []]
        for _ in range(300):
            w = generate_waypoints(0.0, 0.0, 0.0, rng=rng)
            sx, sy = smooth_path(w[:, 0], w[:, 1], resolution=0.075)
            old = old_resample(sx, sy, spacing)
            nx, ny = resample_path(sx, sy, spacing)
            new = np.column_stack([nx, ny])
            for k, pts in enumerate((old, new)):
                drops[k] += math.hypot(pts[-1, 0] - sx[-1], pts[-1, 1] - sy[-1]) > 1e-9
                steps = np.hypot(*np.diff(pts[:-1], axis=0).T)
                errors[k].append(abs(np.mean(steps) - spacing) / spacing * 100)
        old_drop.append(drops[0] / 3)
        new_drop.append(drops[1] / 3)
        old_err.append(np.mean(errors[0]))
        new_err.append(np.mean(errors[1]))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.4))
    pos = np.arange(len(spacings))
    width = 0.36
    for ax, old, new, title, unit in (
            (ax1, old_drop, new_drop, 'Paths that miss the goal point', '%'),
            (ax2, old_err, new_err, 'Mean spacing error', '%')):
        ax.bar(pos - width / 2 - 0.01, old, width, color=ORANGE, label='0.0.0 resampler')
        ax.bar(pos + width / 2 + 0.01, new, width, color=BLUE, label='0.1.0 resampler')
        for x_pos, value in zip(pos - width / 2, old):
            ax.annotate(f'{value:.0f}{unit}', (x_pos, value), textcoords='offset points',
                        xytext=(0, 3), ha='center', fontsize=8, color=INK_2)
        for x_pos, value in zip(pos + width / 2, new):
            ax.annotate(f'{value:.0f}{unit}', (x_pos, value), textcoords='offset points',
                        xytext=(0, 3), ha='center', fontsize=8, color=INK_2)
        ax.set_xticks(pos, [f'{s} m' for s in spacings])
        ax.set_xlabel('sampling_distance')
        ax.set_title(title)
        ax.grid(axis='x', visible=False)
        ax.set_ylim(0, max(max(old), 1) * 1.2)
    ax1.set_ylabel('share of 300 random paths (%)')
    ax2.set_ylabel('|mean step - spacing| / spacing (%)')
    ax1.legend(loc='upper left')
    fig.tight_layout()
    save(fig, 'resampling.png')


def figure_geometry():
    """Pure-pursuit geometry: robot, lookahead circle, target point, alpha and the steering arc."""
    _, _, (tx, ty) = example_path()
    path = np.column_stack([tx, ty])
    robot = np.array([1.2, -0.75])
    yaw = math.radians(-10)
    tracker = PurePursuit(lookahead_distance=0.8)
    tracker.set_path(tx, ty)
    tracker.update_progress(*robot)
    target = path[tracker.lookahead_index(*robot)]
    dx, dy = target - robot
    ld = math.hypot(dx, dy)
    alpha = wrap_angle(math.atan2(dy, dx) - yaw)
    radius = ld / (2 * math.sin(alpha))

    fig, ax = plt.subplots(figsize=(7, 4.6))
    ax.plot(tx, ty, color=BLUE, label='Trajectory')
    circle = plt.Circle(robot, ld, fill=False, color=MUTED, linestyle='--', linewidth=1.2)
    ax.add_patch(circle)
    centre = robot + radius * np.array([-math.sin(yaw), math.cos(yaw)])
    start = math.atan2(robot[1] - centre[1], robot[0] - centre[0])
    end = math.atan2(target[1] - centre[1], target[0] - centre[0])
    angles = np.linspace(start, start + wrap_angle(end - start), 60)
    ax.plot(centre[0] + abs(radius) * np.cos(angles), centre[1] + abs(radius) * np.sin(angles),
            color=ORANGE, label='Commanded arc (curvature 2 sin α / Ld)')
    ax.annotate('', xy=robot + 0.6 * np.array([math.cos(yaw), math.sin(yaw)]), xytext=robot,
                arrowprops=dict(arrowstyle='-|>', color=INK, linewidth=1.6))
    ax.plot([robot[0], target[0]], [robot[1], target[1]], color=INK_2, linewidth=1)
    ax.plot(*robot, 'o', color=INK, markersize=9)
    ax.plot(*target, 'o', color=ORANGE, markersize=9, markeredgecolor=SURFACE, markeredgewidth=2)
    ax.annotate('robot', robot, textcoords='offset points', xytext=(-12, -16), color=INK)
    ax.annotate('heading', robot + 0.6 * np.array([math.cos(yaw), math.sin(yaw)]),
                textcoords='offset points', xytext=(4, -12), color=INK_2, fontsize=9)
    ax.annotate('lookahead point', target, textcoords='offset points', xytext=(8, 4), color=INK)
    mid = (robot + target) / 2
    ax.annotate('Ld', mid, textcoords='offset points', xytext=(-16, 4), color=INK_2)
    arc_angles = np.linspace(yaw, yaw + alpha, 20)
    ax.plot(robot[0] + 0.35 * np.cos(arc_angles), robot[1] + 0.35 * np.sin(arc_angles),
            color=INK_2, linewidth=1)
    ax.annotate('α', robot + 0.42 * np.array([math.cos(yaw + alpha / 2), math.sin(yaw + alpha / 2)]),
                color=INK, fontsize=12)
    ax.annotate('lookahead circle', robot + ld * np.array([0.2, -0.98]), color=MUTED, fontsize=9,
                textcoords='offset points', xytext=(0, -12))
    ax.set_aspect('equal')
    ax.set_xlim(0, 3.2)
    ax.set_ylim(-1.9, 0.9)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Pure pursuit geometry')
    ax.legend(loc='upper left')
    save(fig, 'pure_pursuit_geometry.png')


class OldPurePursuit(PurePursuit):
    """The 0.0.0 controller as it behaved with the 10 Hz republish: search restarts at index 0."""

    def compute(self, x, y, yaw):
        """Return the 0.0.0 command: first point at least L away from index 0, constant speed."""
        if math.hypot(self.path[-1, 0] - x, self.path[-1, 1] - y) < 0.2:
            self.done = True
            return 0.0, 0.0
        distances = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)
        beyond = np.nonzero(distances >= self.lookahead_distance)[0]
        tx, ty = self.path[beyond[0]] if len(beyond) else self.path[-1]
        alpha = wrap_angle(math.atan2(ty - y, tx - x) - yaw)
        omega = 2.0 * self.velocity * math.sin(alpha) / self.lookahead_distance
        return self.velocity, float(np.clip(omega, -1.0, 1.0))


def figure_c1_fix():
    """Closed-loop runs of the old and new controllers on the same path (review C1)."""
    _, _, (tx, ty) = example_path()
    old = simulate(OldPurePursuit(), 0.0, 0.0, 0.0, max_time=60.0,
                   path_updates=lambda t: t.path is None and t.set_path(tx, ty))
    new_tracker = PurePursuit()
    new = simulate(new_tracker, 0.0, 0.0, 0.0, path_updates=lambda t: t.set_path(tx, ty))

    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(tx, ty, color=MUTED, linewidth=5, alpha=0.45, label='Trajectory')
    ax.plot(old['x'], old['y'], color=ORANGE, linewidth=1.6,
            label='0.0.0: lookahead search restarts at index 0 (60 s shown)')
    ax.plot(new['x'], new['y'], color=BLUE, linewidth=1.6,
            label=f'0.1.0: forward search from nearest point (goal in {new["t"][-1]:.0f} s)')
    ax.plot(tx[-1], ty[-1], marker='*', color=INK, markersize=12, linestyle='none')
    ax.annotate('goal', (tx[-1], ty[-1]), textcoords='offset points', xytext=(6, 6), color=INK)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Same path, path message re-sent every step')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=1)
    save(fig, 'c1_fix.png')


def figure_closed_loop():
    """Tracking accuracy and speed profile of the new controller on the example path."""
    _, _, (tx, ty) = example_path()
    tracker = PurePursuit()
    tracker.set_path(tx, ty)
    log = simulate(tracker, 0.0, 0.0, 0.0)
    error = cross_track_error(np.column_stack([tx, ty]), log['x'], log['y']) * 100
    progress = cumulative_arc_length(log['x'], log['y'])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 4.8), sharex=True)
    ax1.plot(progress, error, color=BLUE)
    ax1.set_ylabel('cross-track error (cm)')
    ax1.set_title(f'Cross-track error  (max {error.max():.1f} cm, mean {error.mean():.1f} cm)')
    ax2.plot(progress, log['v'], color=ORANGE)
    ax2.set_ylabel('linear speed (m/s)')
    ax2.set_xlabel('distance travelled (m)')
    ax2.set_title('Linear speed: cruise, then ramp down over the last 0.5 m')
    ax2.set_ylim(0, 0.35)
    fig.tight_layout()
    save(fig, 'closed_loop.png')


def main():
    """Generate every figure."""
    os.makedirs(OUT, exist_ok=True)
    figure_smoothing()
    figure_resampling()
    figure_geometry()
    figure_c1_fix()
    figure_closed_loop()


if __name__ == '__main__':
    main()
