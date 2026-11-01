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

"""Trajectory visualizer node: RViz markers for the raw, smoothed and trajectory paths."""

from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from trajectory_nav.ros_utils import latched_qos, run_node
from visualization_msgs.msg import Marker, MarkerArray

# namespace: (marker type, size in m, (r, g, b, a), keep every n-th point)
STYLES = {
    'raw_waypoints': (Marker.SPHERE_LIST, 0.2, (0.9, 0.2, 0.2, 1.0), 1),
    'raw_path_line': (Marker.LINE_STRIP, 0.03, (0.9, 0.2, 0.2, 0.6), 1),
    'smoothed_path': (Marker.LINE_STRIP, 0.035, (0.2, 0.4, 1.0, 0.9), 1),
    'trajectory_points': (Marker.SPHERE_LIST, 0.11, (0.1, 0.8, 0.3, 1.0), 5),
}
TOPICS = {
    '/raw_path': ('raw_waypoints', 'raw_path_line'),
    '/smoothed_path': ('smoothed_path',),
    '/trajectory': ('trajectory_points',),
}


class TrajectoryVisualizer(Node):
    """Show each pipeline stage in RViz so the smoothing and sampling can be checked by eye."""

    def __init__(self):
        """Subscribe to the three path topics and set up the latched marker output."""
        super().__init__('trajectory_visualizer')
        self.markers = {}
        self.publisher = self.create_publisher(MarkerArray, '/trajectory_markers', latched_qos())
        for topic, namespaces in TOPICS.items():
            self.create_subscription(
                Path, topic,
                lambda msg, namespaces=namespaces: self.path_callback(msg, namespaces),
                latched_qos())
        self.get_logger().info('Trajectory visualizer started')

    def path_callback(self, msg, namespaces):
        """Rebuild the markers for one path and republish all of them, so every stage stays."""
        for namespace in namespaces:
            self.markers[namespace] = self.make_marker(msg, namespace)
        self.publisher.publish(MarkerArray(markers=list(self.markers.values())))

    def make_marker(self, msg, namespace):
        """
        Build one marker holding all points of a path in the style for `namespace`.

        A single SPHERE_LIST or LINE_STRIP per namespace, always with id 0, replaces
        the previous one in RViz, so no stale markers are left behind.
        """
        marker_type, size, (r, g, b, a), step = STYLES[namespace]
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = 0
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color = ColorRGBA(r=r, g=g, b=b, a=a)
        poses = msg.poses[::step]
        if step > 1 and msg.poses and poses[-1] is not msg.poses[-1]:
            poses.append(msg.poses[-1])
        marker.points = [Point(x=p.pose.position.x, y=p.pose.position.y, z=0.0) for p in poses]
        return marker


def main(args=None):
    """Run the trajectory visualizer node."""
    run_node(TrajectoryVisualizer, args)


if __name__ == '__main__':
    main()
