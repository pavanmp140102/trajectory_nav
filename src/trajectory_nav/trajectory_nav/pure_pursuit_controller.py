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

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node

from trajectory_nav.path_math import yaw_from_quaternion
from trajectory_nav.pure_pursuit import PurePursuit
from trajectory_nav.ros_utils import latched_qos, path_to_xy, run_node

TUNING_PARAMETERS = ('lookahead_distance', 'velocity', 'max_angular_velocity',
                     'goal_tolerance', 'slowdown_distance', 'min_velocity')


class PurePursuitController(Node):
    """Drive the robot along the latest trajectory with the pure-pursuit law."""

    def __init__(self):
        """Declare parameters, build the tracker and start the fixed-rate control loop."""
        super().__init__('pure_pursuit_controller')
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('slowdown_distance', 0.5)
        self.declare_parameter('min_velocity', 0.05)
        self.odom_timeout = self.declare_parameter('odom_timeout', 0.5).value
        control_frequency = self.declare_parameter('control_frequency', 20.0).value
        self.stamped_cmd_vel = self.declare_parameter('stamped_cmd_vel', True).value

        self.tracker = PurePursuit(**{name: self.get_parameter(name).value
                                      for name in TUNING_PARAMETERS})
        self.add_on_set_parameters_callback(self.on_parameters_set)

        self.robot_pose = None
        self.odom_time = None
        self.stopped = True

        self.create_subscription(Path, '/trajectory', self.trajectory_callback, latched_qos())
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped if self.stamped_cmd_vel else Twist, '/cmd_vel', 10)
        self.create_timer(1.0 / control_frequency, self.control_loop)
        self.get_logger().info(
            'Pure pursuit controller started (' + ', '.join(
                f'{name}={getattr(self.tracker, name)}' for name in TUNING_PARAMETERS) + ')')

    def on_parameters_set(self, parameters):
        """Apply tuning changes made with `ros2 param set` at runtime, rejecting invalid values."""
        values = {name: getattr(self.tracker, name) for name in TUNING_PARAMETERS}
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
        if len(x) == 0:
            self.get_logger().warn('Received an empty trajectory')
            return
        if self.tracker.set_path(x, y):
            self.get_logger().info(f'Tracking new trajectory with {len(x)} points')

    def odom_callback(self, msg):
        """Store the latest robot pose and its arrival time, for control and staleness checks."""
        self.robot_pose = msg.pose.pose
        self.odom_time = self.get_clock().now()

    def control_loop(self):
        """Compute and publish the velocity command; stop when done or when odometry is stale."""
        if self.tracker.path is None or self.robot_pose is None:
            return
        age = self.get_clock().now() - self.odom_time
        if age > Duration(seconds=self.odom_timeout):
            self.get_logger().warn('Odometry is stale, stopping', throttle_duration_sec=2.0)
            self.stop()
            return

        was_done = self.tracker.done
        position, q = self.robot_pose.position, self.robot_pose.orientation
        linear, angular = self.tracker.compute(
            position.x, position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        if self.tracker.done:
            if not was_done:
                self.get_logger().info('Trajectory complete')
            self.stop()
            return
        self.publish_cmd_vel(linear, angular)
        self.stopped = False

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


def main(args=None):
    """Run the controller, sending a final stop command on shutdown."""
    run_node(PurePursuitController, args, on_shutdown=lambda node: node.publish_cmd_vel(0.0, 0.0))


if __name__ == '__main__':
    main()
