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

"""Demo without Gazebo: the pipeline driving TurtleBot3's kinematic fake node, shown in RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start turtlebot3_fake_node (burger, plain Twist), its robot model, the pipeline and RViz."""
    share = get_package_share_directory('trajectory_nav')
    fake_share = get_package_share_directory('turtlebot3_fake_node')
    urdf_path = os.path.join(get_package_share_directory('turtlebot3_description'),
                             'urdf', 'turtlebot3_burger.urdf')
    # The TurtleBot3 URDF uses xacro properties for its link names, so expand it first.
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    fake_robot = Node(
        package='turtlebot3_fake_node', executable='turtlebot3_fake_node', output='screen',
        parameters=[os.path.join(fake_share, 'param', 'burger.yaml')])
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}])
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'trajectory_nav.launch.py')),
        launch_arguments={'params_file': LaunchConfiguration('params_file'),
                          'rviz': LaunchConfiguration('rviz'),
                          'stamped_cmd_vel': 'false'}.items())
    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'trajectory_nav.yaml'),
                              description='Parameter file for all nodes'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        fake_robot,
        robot_state_publisher,
        pipeline,
    ])
