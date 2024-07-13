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
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Define the path to the configuration file
    wpb_home_config_path = os.path.join(
        get_package_share_directory('wpb_home_bringup'),
        'config',
        'wpb_home.yaml'
    )

    return LaunchDescription([
        # Declare the serial port as a launch argument
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ftdi',
            description='Serial port device'
        ),
        
        # Node for wpb_home_core
        Node(
            package='wpb_home_bringup',
            executable='wpb_home_core',
            name='wpb_home_core',
            output='screen',
            parameters=[
                {'serial_port': LaunchConfiguration('serial_port')},
                wpb_home_config_path
            ],
        ),
        
        # Node for wpb_home_calibrate_velocity
        Node(
            package='wpb_home_bringup',
            executable='wpb_home_calibrate_velocity',
            name='wpb_home_calibrate_velocity',
            output='screen',
        )
    ])
