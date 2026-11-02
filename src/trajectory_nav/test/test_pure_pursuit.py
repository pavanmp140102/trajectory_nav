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

"""Unit and closed-loop tests for the pure-pursuit tracker."""

import math

import numpy as np
import pytest

from trajectory_nav.path_math import generate_waypoints, resample_path, smooth_path
from trajectory_nav.pure_pursuit import PurePursuit
from trajectory_nav.simulation import cross_track_error, simulate

SEED = 42


def straight_path(length=5.0, spacing=0.05):
    """Return a straight path along +x, as two coordinate arrays."""
    x = np.arange(0.0, length + 1e-9, spacing)
    return x, np.zeros_like(x)


def pipeline_path(waypoints):
    """Run waypoints through the same smoothing and resampling as the nodes."""
    x, y = smooth_path(waypoints[:, 0], waypoints[:, 1])
    return resample_path(x, y, 0.05)


def test_repeated_path_does_not_reset_progress():
    """Re-sending the same path keeps progress, so the target stays ahead."""
    tracker = PurePursuit(lookahead_distance=0.5)
    tracker.set_path(*straight_path())
    tracker.compute(1.0, 0.0, 0.0)
    assert tracker.set_path(*straight_path()) is False
    tracker.compute(1.0, 0.0, 0.0)
    target_x, _ = tracker.path[tracker.lookahead_index(1.0, 0.0)]
    assert target_x >= 1.5 - 1e-9


def test_lookahead_ahead_even_after_new_path():
    """Right after a new path, the nearest-point search still puts the target ahead."""
    tracker = PurePursuit(lookahead_distance=0.5)
    tracker.set_path(*straight_path())
    tracker.update_progress(1.0, 0.0)
    target_x, _ = tracker.path[tracker.lookahead_index(1.0, 0.0)]
    assert target_x > 1.0


def test_new_path_resets_progress_and_done():
    """A different path restarts tracking."""
    tracker = PurePursuit()
    tracker.set_path(*straight_path())
    tracker.compute(4.95, 0.0, 0.0)
    assert tracker.done
    x, y = straight_path()
    assert tracker.set_path(x, y + 1.0) is True
    assert not tracker.done
    assert tracker.progress_index == 0


def test_straight_line_drives_straight():
    """On the path and aligned, the robot drives at cruise speed with no turning."""
    tracker = PurePursuit(velocity=0.3)
    tracker.set_path(*straight_path())
    v, omega = tracker.compute(0.0, 0.0, 0.0)
    assert v == pytest.approx(0.3)
    assert omega == pytest.approx(0.0)


def test_turns_towards_path():
    """A robot to the right of the path turns left (positive omega)."""
    tracker = PurePursuit()
    tracker.set_path(*straight_path())
    _, omega = tracker.compute(1.0, -0.3, 0.0)
    assert omega > 0.0


def test_uses_real_distance_to_target():
    """Near the end, omega uses the real distance to the target, not the lookahead."""
    tracker = PurePursuit(lookahead_distance=1.0, max_angular_velocity=10.0, slowdown_distance=0.0)
    x, y = straight_path(length=2.0)
    tracker.set_path(x, y)
    v, omega = tracker.compute(1.7, -0.1, 0.0)
    dx, dy = 2.0 - 1.7, 0.1
    expected = 2.0 * v * math.sin(math.atan2(dy, dx)) / math.hypot(dx, dy)
    assert omega == pytest.approx(expected)


def test_slows_down_near_goal():
    """Speed ramps down within the slow-down distance but stays above the minimum."""
    tracker = PurePursuit(velocity=0.3, slowdown_distance=0.5, min_velocity=0.05)
    tracker.set_path(*straight_path())
    v_far, _ = tracker.compute(1.0, 0.0, 0.0)
    v_near, _ = tracker.compute(4.7, 0.0, 0.0)
    assert v_far == pytest.approx(0.3)
    assert 0.05 <= v_near < 0.3


def test_turns_in_place_when_target_behind():
    """With the path behind it, the robot turns in place rather than driving away."""
    tracker = PurePursuit()
    tracker.set_path(*straight_path())
    v, omega = tracker.compute(0.0, 0.0, math.pi)
    assert v == 0.0
    assert abs(omega) == pytest.approx(tracker.max_angular_velocity)


def test_clamping_keeps_curvature():
    """When omega is clamped, speed drops so the robot still follows the same arc."""
    tracker = PurePursuit(max_angular_velocity=0.2, slowdown_distance=0.0)
    tracker.set_path(*straight_path())
    v, omega = tracker.compute(1.0, -0.4, 0.0)
    unclamped = PurePursuit(max_angular_velocity=100.0, slowdown_distance=0.0)
    unclamped.set_path(*straight_path())
    v_free, omega_free = unclamped.compute(1.0, -0.4, 0.0)
    assert omega == pytest.approx(0.2)
    assert omega / v == pytest.approx(omega_free / v_free)


def test_stops_and_latches_at_goal():
    """Within the goal tolerance the robot stops and stays stopped."""
    tracker = PurePursuit(goal_tolerance=0.2)
    tracker.set_path(*straight_path())
    assert tracker.compute(4.9, 0.0, 0.0) == (0.0, 0.0)
    assert tracker.done
    assert tracker.compute(3.0, 0.0, 0.0) == (0.0, 0.0)


@pytest.mark.parametrize('name, value', [
    ('lookahead_distance', 0.0), ('velocity', -1.0), ('max_angular_velocity', 0.0),
    ('goal_tolerance', 0.0), ('slowdown_distance', -0.1), ('min_velocity', 1.0),
    ('min_velocity', 0.0),
    ('lookahead_distance', float('nan')), ('lookahead_distance', float('inf')),
    ('goal_tolerance', float('nan')), ('max_angular_velocity', float('nan')),
    ('slowdown_distance', float('nan')), ('velocity', float('inf'))])
def test_rejects_invalid_tuning(name, value):
    """Invalid parameters raise instead of silently producing zero commands."""
    with pytest.raises(ValueError):
        PurePursuit(**{name: value})


def test_closed_loop_tracks_random_paths():
    """The robot follows pipeline paths closely and reaches the goal (end-to-end maths check)."""
    rng = np.random.default_rng(SEED)
    for _ in range(20):
        waypoints = generate_waypoints(0.0, 0.0, 0.0, rng=rng)
        x, y = pipeline_path(waypoints)
        tracker = PurePursuit()
        tracker.set_path(x, y)
        log = simulate(tracker, 0.0, 0.0, 0.0)
        assert tracker.done
        assert math.hypot(log['x'][-1] - x[-1], log['y'][-1] - y[-1]) < tracker.goal_tolerance
        assert np.max(cross_track_error(np.column_stack([x, y]), log['x'], log['y'])) < 0.6


def test_closed_loop_with_repeated_path_messages():
    """Re-sending the path every step (the old 10 Hz republish) does not break tracking."""
    rng = np.random.default_rng(SEED)
    waypoints = generate_waypoints(0.0, 0.0, 0.0, rng=rng)
    x, y = pipeline_path(waypoints)
    tracker = PurePursuit()
    log = simulate(tracker, 0.0, 0.0, 0.0, path_updates=lambda t: t.set_path(x, y))
    assert tracker.done
    assert log['t'][-1] < 1.5 * (len(x) * 0.05) / tracker.velocity + 10.0


def test_path_received_mid_route_starts_at_robot():
    """A path that arrives while the robot is partway along it is picked up at the robot."""
    rng = np.random.default_rng(SEED)
    for _ in range(30):
        x, y = pipeline_path(generate_waypoints(0.0, 0.0, 0.0, rng=rng))
        for index in range(0, len(x), 15):
            tracker = PurePursuit()
            tracker.set_path(x, y)
            progress = tracker.update_progress(x[index], y[index])
            assert abs(tracker.arc_length[progress] - tracker.arc_length[index]) < 0.05


def test_mid_route_path_drives_forward_to_goal():
    """Started mid-path with a fresh tracker, the robot finishes without going back."""
    rng = np.random.default_rng(SEED)
    x, y = pipeline_path(generate_waypoints(0.0, 0.0, 0.0, rng=rng))
    start = len(x) // 2
    heading = math.atan2(y[start + 1] - y[start], x[start + 1] - x[start])
    tracker = PurePursuit()
    tracker.set_path(x, y)
    log = simulate(tracker, x[start], y[start], heading)
    travelled = np.sum(np.hypot(np.diff(log['x']), np.diff(log['y'])))
    remaining = tracker.arc_length[-1] - tracker.arc_length[start]
    assert tracker.done
    assert travelled < remaining


def test_non_finite_pose_stops():
    """Odometry with NaN or infinity gives a stop command, not a NaN command."""
    tracker = PurePursuit()
    tracker.set_path(*straight_path())
    assert tracker.compute(float('nan'), 0.0, 0.0) == (0.0, 0.0)
    assert tracker.compute(1.0, 0.0, float('inf')) == (0.0, 0.0)
    assert not tracker.done


def test_lookahead_on_long_path():
    """The windowed lookahead scan finds the same point as a full scan on a long path."""
    x, y = straight_path(length=50.0, spacing=0.01)
    tracker = PurePursuit(lookahead_distance=0.5)
    tracker.set_path(x, y)
    tracker.update_progress(20.0, 0.3)
    distances = np.hypot(x[tracker.progress_index:] - 20.0, y[tracker.progress_index:] - 0.3)
    expected = tracker.progress_index + int(np.nonzero(distances >= 0.5)[0][0])
    assert tracker.lookahead_index(20.0, 0.3) == expected


def test_rejects_unknown_tuning():
    """A misspelt tuning name is an error, not silently ignored."""
    with pytest.raises(TypeError):
        PurePursuit(look_ahead=0.5)


@pytest.mark.parametrize('spacing', [0.05, 0.5, 1.0, 1.2, 2.0, 3.0])
def test_sparse_path_still_reaches_goal(spacing):
    """A path with points further apart than the search window is still followed to the end."""
    rng = np.random.default_rng(SEED)
    for _ in range(4):
        waypoints = generate_waypoints(0.0, 0.0, 0.0, rng=rng)
        sx, sy = smooth_path(waypoints[:, 0], waypoints[:, 1])
        x, y = resample_path(sx, sy, spacing)
        tracker = PurePursuit()
        tracker.set_path(x, y)
        simulate(tracker, 0.0, 0.0, 0.0, max_time=300.0)
        assert tracker.done


def test_search_window_holds_two_points():
    """Every search window holds at least two points, except at the very end of the path."""
    tracker = PurePursuit(lookahead_distance=0.5)
    tracker.set_path(*straight_path(length=20.0, spacing=3.0))
    for start in range(len(tracker.path) - 1):
        assert tracker.window_end(start) >= start + 2
    assert tracker.window_end(len(tracker.path) - 1) == len(tracker.path)


def test_rejects_paths_with_non_finite_points():
    """A path with NaN or infinity is refused, so repeated copies cannot reset progress."""
    tracker = PurePursuit()
    x, y = straight_path()
    for bad in (float('nan'), float('inf')):
        broken = x.copy()
        broken[10] = bad
        with pytest.raises(ValueError):
            tracker.set_path(broken, y)
    assert tracker.path is None


def test_lookahead_skips_points_already_passed():
    """On a sparse path the target is never a point the robot has already driven beyond."""
    x, y = straight_path(length=6.0, spacing=1.2)
    tracker = PurePursuit(lookahead_distance=0.5)
    tracker.set_path(x, y)
    tracker.update_progress(0.55, 0.0)
    assert tracker.progress_index == 0
    assert tracker.lookahead_index(0.55, 0.0) == 1
