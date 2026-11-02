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

"""Spline smoother node: /raw_path waypoints to a smooth, dense /smoothed_path."""

from nav_msgs.msg import Path
from rclpy.node import Node

from trajectory_nav.path_math import require_positive, smooth_path
from trajectory_nav.ros_utils import declare_read_only, latched_qos, make_path, path_to_xy, \
    run_node


class SplineSmoother(Node):
    """Turn the corner-to-corner waypoint path into a curve the robot can follow."""

    def __init__(self):
        """Declare parameters and connect /raw_path to the latched /smoothed_path."""
        super().__init__('spline_smoother')
        self.frame_id = declare_read_only(self, 'frame_id', 'odom')
        self.resolution = require_positive(
            'resolution', declare_read_only(self, 'resolution', 0.05))
        self.publisher = self.create_publisher(Path, '/smoothed_path', latched_qos())
        self.create_subscription(Path, '/raw_path', self.path_callback, latched_qos())
        self.get_logger().info(f'Spline smoother started (resolution={self.resolution} m)')

    def path_callback(self, msg):
        """Fit the spline through the waypoints and publish it; bad input is logged and skipped."""
        x, y = path_to_xy(msg)
        try:
            x_smooth, y_smooth = smooth_path(x, y, self.resolution)
        except ValueError as error:
            self.get_logger().warn(f'Cannot smooth {len(x)} waypoints: {error}')
            return
        self.publisher.publish(
            make_path(x_smooth, y_smooth, self.frame_id, self.get_clock().now()))
        self.get_logger().info(f'Smoothed {len(x)} waypoints into {len(x_smooth)} points')


def main(args=None):
    """Run the spline smoother node."""
    run_node(SplineSmoother, args)


if __name__ == '__main__':
    main()
