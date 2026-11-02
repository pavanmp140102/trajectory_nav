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

"""Pure-pursuit path tracker, free of ROS so it can be unit tested and simulated."""

import math

import numpy as np

from trajectory_nav.path_math import cumulative_arc_length, wrap_angle

# Default tuning; the controller node declares its ROS parameters from this.
DEFAULT_TUNING = {
    'lookahead_distance': 0.5,
    'velocity': 0.3,
    'max_angular_velocity': 1.0,
    'goal_tolerance': 0.2,
    'slowdown_distance': 0.5,
    'min_velocity': 0.05,
}


class PurePursuit:
    """Pure-pursuit tracker that remembers its progress along the path between control steps."""

    def __init__(self, **tuning):
        """Store the tuning (any of DEFAULT_TUNING's keys); see `configure` for their meaning."""
        unknown = set(tuning) - set(DEFAULT_TUNING)
        if unknown:
            raise TypeError(f'unknown tuning: {sorted(unknown)}')
        self.path = None
        self.arc_length = None
        self.progress_index = 0
        self.progress_known = False
        self.done = False
        self.configure(**{**DEFAULT_TUNING, **tuning})

    def configure(self, lookahead_distance, velocity, max_angular_velocity,
                  goal_tolerance, slowdown_distance, min_velocity):
        """
        Validate and apply the tuning; bad values raise instead of silently stopping the robot.

        lookahead_distance: how far ahead on the path the robot steers to (m).
        velocity: cruise speed (m/s). max_angular_velocity: turn-rate limit (rad/s).
        goal_tolerance: stop when this close to the final point (m).
        slowdown_distance: start slowing down this far from the goal (m); the speed falls
            linearly to min_velocity at goal_tolerance. 0 (or <= goal_tolerance) disables it.
        min_velocity: speed on arrival at the goal tolerance, so the robot still gets there (m/s).
        """
        if lookahead_distance <= 0.0:
            raise ValueError('lookahead_distance must be positive')
        if velocity <= 0.0:
            raise ValueError('velocity must be positive')
        if max_angular_velocity <= 0.0:
            raise ValueError('max_angular_velocity must be positive')
        if goal_tolerance <= 0.0:
            raise ValueError('goal_tolerance must be positive')
        if slowdown_distance < 0.0:
            raise ValueError('slowdown_distance must not be negative')
        if not 0.0 < min_velocity <= velocity:
            raise ValueError('min_velocity must be greater than 0 and at most velocity')
        self.lookahead_distance = lookahead_distance
        self.velocity = velocity
        self.max_angular_velocity = max_angular_velocity
        self.goal_tolerance = goal_tolerance
        self.slowdown_distance = slowdown_distance
        self.min_velocity = min_velocity

    def set_path(self, x, y):
        """
        Load a path to track; return True if it differs from the current one.

        Progress only resets when the path really changes, so a repeated message
        cannot send the robot back towards the start. For a new path, the first
        `update_progress` searches the whole path, so a path received mid-route
        (for example after a controller restart) is picked up where the robot is.
        """
        new_path = np.column_stack([np.asarray(x, dtype=float), np.asarray(y, dtype=float)])
        if len(new_path) == 0:
            raise ValueError('path is empty')
        if (self.path is not None and new_path.shape == self.path.shape
                and np.allclose(new_path, self.path)):
            return False
        self.path = new_path
        self.arc_length = cumulative_arc_length(new_path[:, 0], new_path[:, 1])
        self.progress_index = 0
        self.progress_known = False
        self.done = False
        return True

    def update_progress(self, x, y):
        """
        Move progress to the path point nearest the robot, searching forward only.

        The search covers a window of a few lookahead distances, so a path that
        passes close to itself cannot make progress jump ahead. If the nearest point
        is at the window's far edge, the robot is further along, so the window slides on.
        """
        if not self.progress_known:
            distances = np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)
            self.progress_index = int(np.argmin(distances))
            self.progress_known = True
            return self.progress_index
        window = self.search_window()
        while True:
            start = self.progress_index
            end = int(np.searchsorted(self.arc_length, self.arc_length[start] + window,
                                      side='right'))
            distances = np.hypot(self.path[start:end, 0] - x, self.path[start:end, 1] - y)
            self.progress_index = start + int(np.argmin(distances))
            if self.progress_index != end - 1 or end >= len(self.path) or end - 1 == start:
                return self.progress_index

    def search_window(self):
        """Return how far along the path (m) the progress and lookahead searches look at a time."""
        return max(1.0, 3.0 * self.lookahead_distance)

    def lookahead_index(self, x, y):
        """
        Return the first point at least the lookahead away, scanning forward; else the last.

        The scan goes one window at a time, so a long path costs no more per control
        step than a short one.
        """
        start = self.progress_index
        window = self.search_window()
        while start < len(self.path):
            end = int(np.searchsorted(self.arc_length, self.arc_length[start] + window,
                                      side='right'))
            end = max(end, start + 1)
            distances = np.hypot(self.path[start:end, 0] - x, self.path[start:end, 1] - y)
            beyond = np.nonzero(distances >= self.lookahead_distance)[0]
            if len(beyond):
                return start + int(beyond[0])
            start = end
        return len(self.path) - 1

    def distance_to_goal(self, x, y):
        """Return the straight-line distance from the robot to the final path point."""
        return math.hypot(self.path[-1, 0] - x, self.path[-1, 1] - y)

    def remaining_distance(self, x, y):
        """
        Return the distance left along the path, measured from the robot's projection onto it.

        Projecting onto the segment after the progress point keeps the value smooth
        between samples, so the slow-down ramp has no steps.
        """
        i = self.progress_index
        remaining = self.arc_length[-1] - self.arc_length[i]
        if i + 1 < len(self.path):
            segment = self.path[i + 1] - self.path[i]
            length = math.hypot(*segment)
            if length > 0.0:
                along = np.dot(np.array([x, y]) - self.path[i], segment) / length
                remaining -= min(max(along, 0.0), length)
        return max(remaining, self.distance_to_goal(x, y))

    def compute(self, x, y, yaw):
        """
        Return the (linear, angular) velocity command for the robot pose.

        Steers along the arc through the lookahead point (omega = 2 v sin(alpha) / Ld,
        with Ld the real distance to that point), slows down near the goal, keeps the
        arc's curvature when the turn rate is clamped, turns in place when the target
        is behind, latches `done` once within the goal tolerance, and stops for a
        non-finite pose (for example NaN odometry).
        """
        if self.path is None or self.done:
            return 0.0, 0.0
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return 0.0, 0.0
        goal_distance = self.distance_to_goal(x, y)
        if goal_distance < self.goal_tolerance:
            self.done = True
            return 0.0, 0.0

        self.update_progress(x, y)
        target_x, target_y = self.path[self.lookahead_index(x, y)]
        dx, dy = target_x - x, target_y - y
        target_distance = math.hypot(dx, dy)
        alpha = wrap_angle(math.atan2(dy, dx) - yaw)

        if abs(alpha) > math.pi / 2.0:
            return 0.0, math.copysign(self.max_angular_velocity, alpha)

        velocity = self.velocity
        if self.slowdown_distance > self.goal_tolerance:
            ramp = ((self.remaining_distance(x, y) - self.goal_tolerance)
                    / (self.slowdown_distance - self.goal_tolerance))
            velocity = self.min_velocity + (self.velocity - self.min_velocity) * min(1.0, ramp)

        omega = 2.0 * velocity * math.sin(alpha) / target_distance
        if abs(omega) > self.max_angular_velocity:
            velocity *= self.max_angular_velocity / abs(omega)
            omega = math.copysign(self.max_angular_velocity, omega)
        return velocity, omega
