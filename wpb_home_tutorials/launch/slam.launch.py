"""Real robot mapping with SLAM Toolbox and optional joystick driving."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = share('wpb_home_tutorials')
    return LaunchDescription([
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('joy', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(
            package, 'config', 'slam.yaml')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            package, 'launch', 'hardware.launch.py')),
            condition=IfCondition(LC('start_hardware')),
            launch_arguments={'lidar': 'true', 'joy': LC('joy'),
                              'use_sim_time': LC('use_sim_time')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share('slam_toolbox'), 'launch', 'online_sync_launch.py')),
            launch_arguments={'use_sim_time': LC('use_sim_time'),
                              'slam_params_file': LC('slam_params_file')}.items()),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LC('rviz')),
             parameters=[{'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}],
             arguments=['-d', os.path.join(package, 'rviz', 'slam.rviz')]),
    ])
