"""Textbook waypoint editing and navigation with wp_map_tools."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = share('wpb_home_tutorials')
    common = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(package, 'maps', 'map.yaml')),
        DeclareLaunchArgument('waypoints', default_value=os.path.expanduser('~/waypoint.xml'), description='Saved waypoint file in the home directory'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('kinect', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(package, 'rviz', 'navi_waypoint.rviz')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            package, 'launch', 'navigation.launch.py')),
            launch_arguments={'map': LC('map'), 'start_hardware': LC('start_hardware'),
                              'kinect': LC('kinect'), 'use_sim_time': LC('use_sim_time'),
                              'rviz': LC('rviz'), 'rviz_config': LC('rviz_config')}.items()),
        Node(package='wp_map_tools', executable='wp_edit_node', name='wp_edit_node',
             output='screen', parameters=[common, {'load': LC('waypoints')}]),
        Node(package='wp_map_tools', executable='wp_navi_server', name='wp_navi_server',
             output='screen', parameters=[common]),
    ])
