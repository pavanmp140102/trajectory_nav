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

"""Path maths for the pipeline (waypoints, smoothing, resampling, timing), free of ROS."""

import math

import numpy as np
from scipy.interpolate import CubicSpline


def generate_waypoints(start_x, start_y, start_yaw, count=6, step_range=(1.0, 2.0),
                       lateral_range=2.0, rng=None):
    """
    Make random waypoints ahead of the robot, starting at its current position.

    Anchoring the first waypoint to the robot means every new path begins where the
    robot already is, so the controller never has to drive back to the odom origin.
    """
    if count < 3:
        raise ValueError('count must be at least 3, the minimum the spline smoother needs')
    if rng is None:
        rng = np.random.default_rng()
    cos_yaw, sin_yaw = math.cos(start_yaw), math.sin(start_yaw)
    points = [(start_x, start_y)]
    forward = 0.0
    for _ in range(count - 1):
        forward += rng.uniform(*step_range)
        lateral = rng.uniform(-lateral_range, lateral_range)
        points.append((start_x + forward * cos_yaw - lateral * sin_yaw,
                       start_y + forward * sin_yaw + lateral * cos_yaw))
    return np.array(points)


def remove_close_points(x, y, min_distance=1e-3):
    """
    Drop consecutive points closer than min_distance.

    Duplicate points give a zero-length segment, which would make the spline's
    arc-length parameter non-increasing and crash the fit.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(x) == 0:
        return x, y
    keep = [0]
    for i in range(1, len(x)):
        if math.hypot(x[i] - x[keep[-1]], y[i] - y[keep[-1]]) >= min_distance:
            keep.append(i)
    return x[keep], y[keep]


def cumulative_arc_length(x, y):
    """Return the distance along the polyline at each point, used for fitting and resampling."""
    return np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))


def smooth_path(x, y, resolution=0.05):
    """
    Fit an arc-length natural cubic spline through the waypoints and sample it densely.

    The spline passes through every waypoint and turns the corners of the raw path
    into a curve the robot can follow. Samples are about `resolution` metres apart,
    so the density does not depend on the path length.
    """
    if resolution <= 0.0:
        raise ValueError('resolution must be positive')
    x, y = remove_close_points(x, y)
    if len(x) < 3:
        raise ValueError('need at least 3 distinct waypoints')
    s = cumulative_arc_length(x, y)
    num_samples = max(2, int(math.ceil(s[-1] / resolution)) + 1)
    s_dense = np.linspace(0.0, s[-1], num_samples)
    spline_x = CubicSpline(s, x, bc_type='natural')
    spline_y = CubicSpline(s, y, bc_type='natural')
    return spline_x(s_dense), spline_y(s_dense)


def resample_path(x, y, spacing):
    """
    Resample a polyline at exact arc-length steps of `spacing`, always keeping the final point.

    Keeping the end point exactly guarantees that the trajectory finishes at the goal.
    """
    if spacing <= 0.0:
        raise ValueError('spacing must be positive')
    x, y = remove_close_points(x, y)
    if len(x) < 2:
        return x, y
    s = cumulative_arc_length(x, y)
    s_new = np.arange(0.0, s[-1], spacing)
    if s[-1] - s_new[-1] > 1e-6:
        s_new = np.append(s_new, s[-1])
    else:
        s_new[-1] = s[-1]
    return np.interp(s_new, s, x), np.interp(s_new, s, y)


def assign_times(x, y, velocity):
    """Time each point for constant-speed travel; the trajectory's time parameterisation."""
    if velocity <= 0.0:
        raise ValueError('velocity must be positive')
    return cumulative_arc_length(x, y) / velocity


def path_headings(x, y):
    """Return the path heading at each point, so published poses carry a real orientation."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(x) < 2:
        return np.zeros(len(x))
    return np.arctan2(np.gradient(y), np.gradient(x))


def yaw_from_quaternion(x, y, z, w):
    """Extract yaw from an orientation quaternion, giving the controller the robot's heading."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    """Return the (x, y, z, w) quaternion for a planar heading, for pose orientations."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi], so heading errors always take the short way round."""
    return math.atan2(math.sin(angle), math.cos(angle))
