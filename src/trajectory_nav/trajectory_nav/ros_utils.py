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

"""Helpers shared by the nodes: path message conversion, latched QoS and the node entry point."""

import signal

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from trajectory_nav.path_math import path_headings, quaternion_from_yaw


def latched_qos():
    """
    Return the QoS for the path topics: keep the last message for late subscribers.

    Each path is published once when it changes, instead of being republished at
    a fixed rate, so downstream nodes only react to real changes.
    """
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)


def event_qos():
    """
    Return the QoS for /goal_reached: reliable, but not kept for late subscribers.

    An old "goal reached" must not make a restarted waypoint loader skip a path.
    """
    return QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,
                      reliability=ReliabilityPolicy.RELIABLE)


def declare_read_only(node, name, default):
    """
    Declare a parameter that is only read at startup and return its value.

    Marking it read-only makes `ros2 param set` fail instead of appearing to work.
    """
    return node.declare_parameter(
        name, default, ParameterDescriptor(read_only=True, description='set at startup only')
    ).value


def path_to_xy(msg):
    """Extract the x and y coordinates of a nav_msgs/Path as arrays for the path maths."""
    x = np.array([pose.pose.position.x for pose in msg.poses], dtype=float)
    y = np.array([pose.pose.position.y for pose in msg.poses], dtype=float)
    return x, y


def make_path(x, y, frame_id, stamp, times=None):
    """
    Build a nav_msgs/Path from coordinates, with orientations along the path heading.

    With `times` (seconds from `stamp`), each pose's header stamp is the time the
    robot is planned to reach it; z is always 0.
    """
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp.to_msg()
    for i, (px, py, yaw) in enumerate(zip(x, y, path_headings(x, y))):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose_time = stamp if times is None else stamp + Duration(nanoseconds=int(times[i] * 1e9))
        pose.header.stamp = pose_time.to_msg()
        pose.pose.position.x = float(px)
        pose.pose.position.y = float(py)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        msg.poses.append(pose)
    return msg


def run_node(node_class, args=None, on_shutdown=None):
    """
    Create and spin a node until Ctrl-C, then shut down cleanly.

    The default SIGINT handler is disabled so the context stays valid while
    `on_shutdown(node)` runs; the controller uses this to send a final stop command.
    Spinning with a short timeout lets Python handle Ctrl-C promptly even when no
    message or timer wakes the executor (for example, sim time with no /clock yet).
    SIGTERM (kill, systemd, docker stop) is handled the same way as Ctrl-C.
    """
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = node_class()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A repeated signal (a process-group Ctrl-C plus the launch forward) must not
        # interrupt the shutdown that is already under way.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if on_shutdown is not None and rclpy.ok():
            on_shutdown(node)
        node.destroy_node()
        rclpy.try_shutdown()


def _raise_keyboard_interrupt(signum, frame):
    """Turn SIGTERM into KeyboardInterrupt so run_node shuts down cleanly."""
    raise KeyboardInterrupt
