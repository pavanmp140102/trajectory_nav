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

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
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
    """
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
        if on_shutdown is not None and rclpy.ok():
            on_shutdown(node)
        node.destroy_node()
        rclpy.try_shutdown()
