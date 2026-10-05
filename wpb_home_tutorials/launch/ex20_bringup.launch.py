"""Experiment 20 dependencies; start the student program separately."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package = share('wpb_home_tutorials')
    map_file = os.path.join(package, 'maps', 'map.yaml')
    waypoints_file = os.path.expanduser('~/waypoint.xml')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share('wpb_home_tutorials'), 'launch', 'fetch.launch.py')),
            launch_arguments={'map': map_file, 'waypoints': waypoints_file}.items()),
    ])
