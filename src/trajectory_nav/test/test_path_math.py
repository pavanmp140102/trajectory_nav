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

"""Unit tests for the ROS-free path maths."""

import math

import numpy as np
import pytest

from trajectory_nav.path_math import (
    assign_times, cumulative_arc_length, generate_waypoints, path_headings,
    quaternion_from_yaw, remove_close_points, resample_path, smooth_path, wrap_angle,
    yaw_from_quaternion)

SEED = 42


def random_waypoint_sets(count=50):
    """Yield waypoint sets like the waypoint loader makes, from a fixed seed."""
    rng = np.random.default_rng(SEED)
    for _ in range(count):
        yield generate_waypoints(rng.uniform(-5, 5), rng.uniform(-5, 5),
                                 rng.uniform(-math.pi, math.pi), rng=rng)


def test_waypoints_start_at_robot_and_lead_away():
    """New waypoints begin at the robot and move forward along its heading."""
    rng = np.random.default_rng(SEED)
    points = generate_waypoints(3.0, -1.0, math.pi / 2, count=6, rng=rng)
    assert points.shape == (6, 2)
    assert points[0] == pytest.approx([3.0, -1.0])
    forward = points[:, 1] - (-1.0)
    assert np.all(np.diff(forward) >= 1.0)
    assert np.all(np.abs(points[1:, 0] - 3.0) <= 2.0)


def test_waypoints_are_reproducible_with_seed():
    """The same seed gives the same waypoints, for repeatable runs."""
    first = generate_waypoints(0, 0, 0, rng=np.random.default_rng(7))
    second = generate_waypoints(0, 0, 0, rng=np.random.default_rng(7))
    assert np.array_equal(first, second)


def test_remove_close_points_drops_duplicates():
    """Duplicate and near-duplicate points are removed, distinct ones kept."""
    x, y = remove_close_points([0, 0, 1, 1.0000001, 2], [0, 0, 0, 0, 0])
    assert list(x) == [0, 1, 2]
    assert list(y) == [0, 0, 0]


def test_smooth_path_passes_through_waypoints_and_endpoints():
    """The spline keeps both endpoints and passes through every waypoint."""
    for waypoints in random_waypoint_sets():
        x, y = smooth_path(waypoints[:, 0], waypoints[:, 1], resolution=0.05)
        assert (x[0], y[0]) == pytest.approx(tuple(waypoints[0]))
        assert (x[-1], y[-1]) == pytest.approx(tuple(waypoints[-1]))
        for wx, wy in waypoints:
            assert np.min(np.hypot(x - wx, y - wy)) < 0.05


def test_smooth_path_density_follows_resolution():
    """Sample density is set by the resolution, not a fixed count."""
    x, y = smooth_path([0, 5, 10], [0, 1, 0], resolution=0.1)
    steps = np.hypot(np.diff(x), np.diff(y))
    assert np.max(steps) < 0.12
    assert len(x) > 100


def test_smooth_path_handles_duplicate_waypoints():
    """Duplicate waypoints no longer crash the spline fit."""
    x, y = smooth_path([0, 1, 1, 2, 3], [0, 1, 1, 0, 1])
    assert np.all(np.isfinite(x)) and np.all(np.isfinite(y))


def test_smooth_path_rejects_too_few_points():
    """Fewer than three distinct waypoints is an error the node can catch."""
    with pytest.raises(ValueError):
        smooth_path([0, 1, 1], [0, 0, 0])


def test_resample_keeps_endpoint_and_exact_spacing():
    """Every resampled path ends at the goal and is spaced at the sampling distance."""
    for spacing in (0.05, 0.1, 0.2, 0.3):
        for waypoints in random_waypoint_sets(20):
            sx, sy = smooth_path(waypoints[:, 0], waypoints[:, 1])
            x, y = resample_path(sx, sy, spacing)
            assert (x[-1], y[-1]) == (sx[-1], sy[-1])
            assert (x[0], y[0]) == (sx[0], sy[0])
            arc = cumulative_arc_length(sx, sy)
            # all steps but the last are exactly `spacing` along the path
            source = np.interp(np.arange(len(x) - 1) * spacing, arc, sx)
            assert x[:-1] == pytest.approx(source)
            steps = np.hypot(np.diff(x), np.diff(y))
            assert np.all(steps <= spacing + 1e-9)
            assert steps[-1] > 0.0


def test_resample_rejects_bad_spacing():
    """A non-positive spacing is rejected instead of looping forever."""
    with pytest.raises(ValueError):
        resample_path([0, 1], [0, 0], 0.0)


def test_assign_times_constant_speed():
    """Times grow with distance at the given speed."""
    times = assign_times([0, 1, 3], [0, 0, 0], velocity=0.5)
    assert times == pytest.approx([0, 2, 6])


def test_headings_follow_path():
    """Headings point along the path direction."""
    headings = path_headings([0, 1, 2], [0, 1, 2])
    assert headings == pytest.approx([math.pi / 4] * 3)


def test_quaternion_yaw_round_trip():
    """Yaw survives a round trip through a quaternion, replacing tf_transformations."""
    for yaw in np.linspace(-3.1, 3.1, 25):
        assert yaw_from_quaternion(*quaternion_from_yaw(yaw)) == pytest.approx(yaw)


def test_wrap_angle():
    """Angles wrap into [-pi, pi]."""
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)


def test_waypoints_need_three_points():
    """Fewer than three waypoints cannot be smoothed, so they are rejected up front."""
    with pytest.raises(ValueError):
        generate_waypoints(0, 0, 0, count=2)
