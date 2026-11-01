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

"""Waypoint loader node: publishes random waypoints ahead of the robot on /raw_path."""

import math

from nav_msgs.msg import Odometry, Path
import numpy as np
from rclpy.node import Node

from trajectory_nav.path_math import generate_waypoints, yaw_from_quaternion
from trajectory_nav.ros_utils import latched_qos, make_path, run_node


class WaypointLoader(Node):
    """Generate a new set of waypoints from the robot pose each time the last goal is reached."""

    def __init__(self):
        """Declare parameters and set up the odometry input and latched /raw_path output."""
        super().__init__('waypoint_loader')
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.2).value
        self.num_waypoints = self.declare_parameter('num_waypoints', 6).value
        self.step_min = self.declare_parameter('step_min', 1.0).value
        self.step_max = self.declare_parameter('step_max', 2.0).value
        self.lateral_range = self.declare_parameter('lateral_range', 2.0).value
        seed = self.declare_parameter('seed', -1).value
        self.rng = np.random.default_rng(None if seed < 0 else seed)

        self.waypoints = None
        self.publisher = self.create_publisher(Path, '/raw_path', latched_qos())
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.get_logger().info('Waypoint loader started, waiting for /odom')

    def odom_callback(self, msg):
        """Publish waypoints once the robot pose is known, and again each time the goal is met."""
        pose = msg.pose.pose
        x, y = pose.position.x, pose.position.y
        if self.waypoints is not None:
            goal_x, goal_y = self.waypoints[-1]
            distance = math.hypot(goal_x - x, goal_y - y)
            if distance >= self.goal_tolerance:
                return
            self.get_logger().info(
                f'Goal reached ({distance:.3f} m away), generating new waypoints')
        q = pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.publish_waypoints(x, y, yaw)

    def publish_waypoints(self, x, y, yaw):
        """Generate waypoints from the robot pose and publish them once on latched /raw_path."""
        self.waypoints = generate_waypoints(
            x, y, yaw, count=self.num_waypoints, step_range=(self.step_min, self.step_max),
            lateral_range=self.lateral_range, rng=self.rng)
        self.publisher.publish(make_path(self.waypoints[:, 0], self.waypoints[:, 1],
                                         self.frame_id, self.get_clock().now()))
        points = ', '.join(f'({px:.2f}, {py:.2f})' for px, py in self.waypoints)
        self.get_logger().info(f'Published {len(self.waypoints)} waypoints: {points}')


def main(args=None):
    """Run the waypoint loader node."""
    run_node(WaypointLoader, args)


if __name__ == '__main__':
    main()
