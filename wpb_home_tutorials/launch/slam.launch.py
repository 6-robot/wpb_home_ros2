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

import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description(): 
    launch_file_dir = os.path.join(get_package_share_directory('wpb_home_bringup'), 'launch')
    wpb_home_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'base_lidar.launch.py')
        )
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        parameters=[{
            "use_sim_time": True,
            "base_frame": "base_footprint",
            "odom_frame": "odom",
            "map_frame": "map"
        }]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='wpb_home_joy',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.12,
        }]
    )

    js_vel_node = Node(
        package='wpb_home_bringup',
        executable='wpb_home_js_vel',
        name='teleop',
        parameters=[{
            'axis_linear': 1,
            'axis_angular': 0,
            'scale_linear': 1.0,
            'scale_angular': 1.0,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(get_package_share_directory('wpb_home_tutorials'), 'rviz', 'slam.rviz')]]
    )

    return LaunchDescription([
        wpb_home_launch,
        joy_node,
        js_vel_node,
        slam_node,
        rviz_node
    ])
