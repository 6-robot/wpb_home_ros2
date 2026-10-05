"""Experiment 04 dependencies; start the student program separately."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package = share('wpb_home_tutorials')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share('wpb_home_tutorials'), 'launch', 'hardware.launch.py')),
            launch_arguments={'lidar': 'true', 'kinect': 'false',
                              'joy': 'false', 'use_sim_time': 'false'}.items()),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(package, 'rviz', 'lidar.rviz')]),
    ])
