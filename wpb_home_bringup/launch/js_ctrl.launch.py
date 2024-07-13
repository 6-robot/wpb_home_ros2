#!/usr/bin/env python3
#
# Copyright 2024 6-robot.
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
#
# Authors: Zhang Wanjie

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the path to the config file
    wpb_home_config_path = os.path.join(
        get_package_share_directory('wpb_home_bringup'),
        'config',
        'wpb_home.yaml'
    )

    return LaunchDescription([
        # Declare the serial_port and odom parameters
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ftdi',
            description='Serial port for wpb_home_core'
        ),
        DeclareLaunchArgument(
            'odom',
            default_value='false',
            description='Enable odometry'
        ),

        # wpb_home core node
        Node(
            package='wpb_home_bringup',
            executable='wpb_home_core',
            name='wpb_home_core',
            output='screen',
            parameters=[
                {'serial_port': LaunchConfiguration('serial_port')},
                {'odom': LaunchConfiguration('odom')},
                wpb_home_config_path
            ]
        ),

        # joy node
        Node(
            package='joy',
            executable='joy_node',
            name='wpb_home_joy',
            respawn=True,
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.12,
            }]
        ),

        # Axes Velcmd parameters
        Node(
            package='wpb_home_bringup',
            executable='wpb_home_js_vel',
            name='teleop',
            parameters=[{
                'axis_linear': 1,
                'axis_angular': 0,
                'scale_linear': 1.0,
                'scale_angular': 1.0,
            }],
        ),
    ])
