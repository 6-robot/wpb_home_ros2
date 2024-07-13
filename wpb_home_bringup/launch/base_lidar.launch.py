#!/usr/bin/env python3
#
# Copyright 2023 6-robot.
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
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.actions
import launch.actions

def generate_launch_description():
    descriptio_path = get_package_share_path('wpb_home_description')
    default_model_path = descriptio_path / 'urdf/wpb_home_mani.urdf'
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    # 机器人配置文件wpb_home.yaml文件路径
    config_file = os.path.join(
        get_package_share_directory('wpb_home_bringup'),
        'config',
        'wpb_home.yaml'
    )

    wpb_home_core = Node(
        package='wpb_home_bringup',
        executable='wpb_home_core',
        name='wpb_home_core',
        parameters=[{'serial_port': '/dev/ftdi'},config_file],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    lidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{'serial_port': '/dev/rplidar','serial_baudrate': 115200,  'frame_id': 'laser','inverted': False,'angle_compensate': True,}],
        remappings=[('scan', 'scan_raw'),]
    )

    lidar_filter_node = Node(
        package='wpb_home_bringup',
        executable='wpb_home_lidar_filter',
        name='wpb_home_lidar_filter',
        parameters=[{'pub_topic': '/scan',}]
    )
    
    return LaunchDescription([
        model_arg,
        wpb_home_core,
        robot_state_publisher_node,
        lidar_node,
        lidar_filter_node
    ])
