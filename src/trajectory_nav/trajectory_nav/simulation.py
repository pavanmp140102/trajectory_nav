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

"""Kinematic closed-loop simulation of the tracker, for tests and documentation figures."""

import math

import numpy as np


def simulate(tracker, x, y, yaw, dt=0.05, max_time=120.0, path_updates=None):
    """
    Drive a unicycle robot with `tracker` until it is done or time runs out.

    `path_updates`, if given, is called every step with the tracker so a test can
    re-send the path (as the old 10 Hz republishing did). Returns arrays of
    t, x, y, yaw, v and omega for each step.
    """
    log = []
    t = 0.0
    while t < max_time:
        if path_updates is not None:
            path_updates(tracker)
        v, omega = tracker.compute(x, y, yaw)
        log.append((t, x, y, yaw, v, omega))
        if tracker.done:
            break
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = math.atan2(math.sin(yaw + omega * dt), math.cos(yaw + omega * dt))
        t += dt
    return {key: np.array(values) for key, values in
            zip(('t', 'x', 'y', 'yaw', 'v', 'omega'), zip(*log))}


def cross_track_error(path, x, y):
    """Return the distance from each robot position to the nearest point of the path."""
    path = np.asarray(path, dtype=float)
    return np.array([np.min(np.hypot(path[:, 0] - px, path[:, 1] - py)) for px, py in zip(x, y)])
