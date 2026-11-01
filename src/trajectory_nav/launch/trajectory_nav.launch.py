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

"""Launch the five trajectory_nav nodes, and optionally RViz, for a robot that provides /odom."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = ('waypoint_loader', 'spline_smoother', 'trajectory_generator',
         'pure_pursuit_controller', 'trajectory_visualizer')


def generate_launch_description():
    """Start the pipeline with one shared parameter file; RViz is opt-in with rviz:=true."""
    share = get_package_share_directory('trajectory_nav')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    arguments = [
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'trajectory_nav.yaml'),
                              description='Parameter file for all nodes'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock from a simulator'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument('stamped_cmd_vel', default_value='true',
                              description='Publish TwistStamped on /cmd_vel (false: Twist)'),
    ]
    overrides = {'pure_pursuit_controller': {
                 'stamped_cmd_vel': LaunchConfiguration('stamped_cmd_vel')}}
    nodes = [
        Node(package='trajectory_nav', executable=name, name=name, output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}, overrides.get(name, {})])
        for name in NODES
    ]
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', os.path.join(share, 'rviz', 'trajectory_nav.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription(arguments + nodes + [rviz])
