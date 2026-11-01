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

"""Trajectory generator node: /smoothed_path to an evenly spaced, timed /trajectory."""

from nav_msgs.msg import Path
from rclpy.node import Node

from trajectory_nav.path_math import assign_times, resample_path
from trajectory_nav.ros_utils import latched_qos, make_path, path_to_xy, run_node


class TrajectoryGenerator(Node):
    """Resample the smoothed path at a fixed spacing and stamp each pose with its arrival time."""

    def __init__(self):
        """Declare parameters and connect /smoothed_path to the latched /trajectory."""
        super().__init__('trajectory_generator')
        self.frame_id = self.declare_parameter('frame_id', 'odom').value
        self.sampling_distance = self.declare_parameter('sampling_distance', 0.05).value
        self.velocity = self.declare_parameter('velocity', 0.3).value
        if self.sampling_distance <= 0.0 or self.velocity <= 0.0:
            raise ValueError('sampling_distance and velocity must be positive')
        self.publisher = self.create_publisher(Path, '/trajectory', latched_qos())
        self.create_subscription(Path, '/smoothed_path', self.path_callback, latched_qos())
        self.get_logger().info(
            f'Trajectory generator started (sampling_distance={self.sampling_distance} m, '
            f'velocity={self.velocity} m/s)')

    def path_callback(self, msg):
        """
        Resample the path and publish it as the trajectory.

        Each pose's header stamp is the time the robot should reach it, assuming
        constant `velocity` from now.
        """
        x, y = path_to_xy(msg)
        x, y = resample_path(x, y, self.sampling_distance)
        if len(x) < 2:
            self.get_logger().warn('Not enough distinct poses for a trajectory')
            return
        times = assign_times(x, y, self.velocity)
        self.publisher.publish(make_path(x, y, self.frame_id, self.get_clock().now(), times))
        self.get_logger().info(f'Published trajectory: {len(x)} points, {times[-1]:.1f} s')


def main(args=None):
    """Run the trajectory generator node."""
    run_node(TrajectoryGenerator, args)


if __name__ == '__main__':
    main()
