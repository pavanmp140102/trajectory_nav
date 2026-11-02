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

from nav_msgs.msg import Odometry, Path
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty

from trajectory_nav.path_math import generate_waypoints, yaw_from_quaternion
from trajectory_nav.ros_utils import declare_read_only, event_qos, latched_qos, make_path, \
    run_node


class WaypointLoader(Node):
    """Generate a new set of waypoints from the robot pose each time the controller ends a path."""

    def __init__(self):
        """Declare and check parameters, and set up /odom, /goal_reached and latched /raw_path."""
        super().__init__('waypoint_loader')
        self.frame_id = declare_read_only(self, 'frame_id', 'odom')
        self.num_waypoints = declare_read_only(self, 'num_waypoints', 6)
        self.step_min = declare_read_only(self, 'step_min', 1.0)
        self.step_max = declare_read_only(self, 'step_max', 2.0)
        self.lateral_range = declare_read_only(self, 'lateral_range', 2.0)
        seed = declare_read_only(self, 'seed', -1)
        if self.num_waypoints < 3:
            raise ValueError('num_waypoints must be at least 3 for the spline smoother')
        if not 0.0 < self.step_min <= self.step_max:
            raise ValueError('need 0 < step_min <= step_max')
        if self.lateral_range < 0.0:
            raise ValueError('lateral_range must not be negative')
        self.rng = np.random.default_rng(None if seed < 0 else seed)

        self.waypoints = None
        self.robot_pose = None
        self.publisher = self.create_publisher(Path, '/raw_path', latched_qos())
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Empty, '/goal_reached', self.goal_callback, event_qos())
        self.get_logger().info('Waypoint loader started, waiting for /odom')

    def odom_callback(self, msg):
        """Keep the latest robot pose, and publish the first waypoint set once it is known."""
        self.robot_pose = msg.pose.pose
        if self.waypoints is None:
            self.publish_waypoints()

    def goal_callback(self, msg):
        """Publish the next waypoint set when the controller reports that it reached the goal."""
        if self.robot_pose is None:
            return
        self.get_logger().info('Goal reached, generating new waypoints')
        self.publish_waypoints()

    def publish_waypoints(self):
        """Generate waypoints from the robot pose and publish them once on latched /raw_path."""
        position, q = self.robot_pose.position, self.robot_pose.orientation
        self.waypoints = generate_waypoints(
            position.x, position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w),
            count=self.num_waypoints, step_range=(self.step_min, self.step_max),
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
