"""Experiment 06 dependencies; start the student program separately."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package = share('wpb_home_tutorials')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share('wpb_home_tutorials'), 'launch', 'hardware.launch.py')),
            launch_arguments={'lidar': 'false', 'kinect': 'false',
                              'joy': 'false', 'use_sim_time': 'false'}.items()),
    ])
