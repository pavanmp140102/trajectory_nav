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

"""Pure-pursuit controller node: tracks /trajectory using /odom and publishes /cmd_vel."""

import math

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Empty

from trajectory_nav.path_math import require_positive, yaw_from_quaternion
from trajectory_nav.pure_pursuit import DEFAULT_TUNING, PurePursuit
from trajectory_nav.ros_utils import declare_read_only, event_qos, latched_qos, path_to_xy, \
    run_node


class PurePursuitController(Node):
    """Drive the robot along the latest trajectory with the pure-pursuit law."""

    def __init__(self):
        """Declare parameters, build the tracker and start the fixed-rate control loop."""
        super().__init__('pure_pursuit_controller')
        for name, default in DEFAULT_TUNING.items():
            self.declare_parameter(name, default)
        self.odom_timeout = require_positive(
            'odom_timeout', declare_read_only(self, 'odom_timeout', 0.5))
        control_frequency = require_positive(
            'control_frequency', declare_read_only(self, 'control_frequency', 20.0))
        self.goal_retry_period = require_positive(
            'goal_retry_period', declare_read_only(self, 'goal_retry_period', 2.0))
        self.stamped_cmd_vel = declare_read_only(self, 'stamped_cmd_vel', True)

        self.tracker = PurePursuit(**{name: self.get_parameter(name).value
                                      for name in DEFAULT_TUNING})
        self.add_on_set_parameters_callback(self.on_parameters_set)

        self.robot_pose = None
        self.odom_time = None
        self.stopped = True
        self.goal_sent_time = None

        self.create_subscription(Path, '/trajectory', self.trajectory_callback, latched_qos())
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped if self.stamped_cmd_vel else Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(Empty, '/goal_reached', event_qos())
        self.create_timer(1.0 / control_frequency, self.control_loop)
        self.get_logger().info(
            'Pure pursuit controller started (' + ', '.join(
                f'{name}={getattr(self.tracker, name)}' for name in DEFAULT_TUNING) + ')')

    def on_parameters_set(self, parameters):
        """Apply tuning changes made with `ros2 param set` at runtime, rejecting invalid values."""
        values = {name: getattr(self.tracker, name) for name in DEFAULT_TUNING}
        for parameter in parameters:
            if parameter.name in values:
                values[parameter.name] = parameter.value
        try:
            self.tracker.configure(**values)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def trajectory_callback(self, msg):
        """Hand a new trajectory to the tracker; an unchanged one keeps the current progress."""
        x, y = path_to_xy(msg)
        try:
            changed = self.tracker.set_path(x, y)
        except ValueError as error:
            self.get_logger().warn(f'Ignoring trajectory: {error}')
            return
        if changed:
            self.goal_sent_time = None
            self.get_logger().info(f'Tracking new trajectory with {len(x)} points')

    def odom_callback(self, msg):
        """Store the latest robot pose and its arrival time, for control and staleness checks."""
        self.robot_pose = msg.pose.pose
        self.odom_time = self.get_clock().now()

    def control_loop(self):
        """
        Compute and publish the velocity command; stop when done or odometry is stale or invalid.

        On reaching the goal, an Empty message on /goal_reached tells the waypoint
        loader to make the next set of waypoints.
        """
        if self.tracker.path is None or self.robot_pose is None:
            return
        age = self.get_clock().now() - self.odom_time
        if age > Duration(seconds=self.odom_timeout):
            self.get_logger().warn('Odometry is stale, stopping', throttle_duration_sec=2.0)
            self.stop()
            return

        position, q = self.robot_pose.position, self.robot_pose.orientation
        pose = (position.x, position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if not all(math.isfinite(value) for value in pose):
            self.get_logger().warn('Odometry is not finite, stopping', throttle_duration_sec=2.0)
            self.stop()
            return

        linear, angular = self.tracker.compute(*pose)
        if self.tracker.done:
            self.stop()
            self.announce_goal()
            return
        self.publish_cmd_vel(linear, angular)
        self.stopped = False

    def announce_goal(self):
        """
        Tell the waypoint loader the goal is reached, and ask again if no new trajectory follows.

        /goal_reached is a single volatile message, so if the next path never arrives (a
        node was not listening yet, or a path was rejected) the controller would wait
        silently for ever. Repeating it every `goal_retry_period` seconds recovers.
        """
        now = self.get_clock().now()
        if self.goal_sent_time is None:
            self.get_logger().info('Trajectory complete')
        elif now - self.goal_sent_time < Duration(seconds=self.goal_retry_period):
            return
        else:
            self.get_logger().warn('No new trajectory since the goal was reached, asking again')
        self.goal_sent_time = now
        self.goal_publisher.publish(Empty())

    def stop(self):
        """Send one zero-velocity command when the robot stops, instead of flooding /cmd_vel."""
        if not self.stopped:
            self.publish_cmd_vel(0.0, 0.0)
            self.stopped = True

    def publish_cmd_vel(self, linear, angular):
        """Publish a velocity command, stamped or plain to match the robot driver."""
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        if self.stamped_cmd_vel:
            cmd = TwistStamped(twist=twist)
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            self.cmd_vel_publisher.publish(cmd)
        else:
            self.cmd_vel_publisher.publish(twist)

    def send_final_stop(self):
        """Publish a zero command and wait briefly for it to be delivered before shutdown."""
        self.publish_cmd_vel(0.0, 0.0)
        self.cmd_vel_publisher.wait_for_all_acked(Duration(seconds=0.5))


def main(args=None):
    """Run the controller, sending a final stop command on shutdown."""
    run_node(PurePursuitController, args, on_shutdown=PurePursuitController.send_final_stop)


if __name__ == '__main__':
    main()
