"""Real robot infrastructure for experiment 20; run ex20_fetch separately to start motion."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC


def check_files(context):
    for name in ('map', 'waypoints'):
        path = LC(name).perform(context)
        if not os.path.isfile(path):
            raise RuntimeError(f'{name} file is missing: {path}')
    return []


def generate_launch_description():
    package = share('wpb_home_tutorials')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(package, 'maps', 'map.yaml'),
                              description='Absolute path to real map YAML'),
        DeclareLaunchArgument('waypoints', description='Absolute path to saved kitchen/guest waypoints'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=check_files),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            package, 'launch', 'waypoint_nav.launch.py')),
            launch_arguments={'map': LC('map'), 'waypoints': LC('waypoints'),
                              'kinect': 'true', 'use_sim_time': 'false',
                              'rviz': LC('rviz'),
                              'rviz_config': os.path.join(package, 'rviz', 'fetch.rviz')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share('wpb_home_behaviors'), 'launch', 'behaviors.launch.py'))),
    ])
