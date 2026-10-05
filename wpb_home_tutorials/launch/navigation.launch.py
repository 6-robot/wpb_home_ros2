"""Nav2 on the real robot; map is supplied by the experiment operator."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_map(context):
    path = LC('map').perform(context)
    if not os.path.isfile(path):
        raise RuntimeError('Map YAML does not exist: ' + path + '. Save a map first.')
    return []


def generate_launch_description():
    package = share('wpb_home_tutorials')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(package, 'maps', 'map.yaml'),
                              description='Absolute path to a saved map YAML'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('kinect', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(package, 'rviz', 'navi.rviz')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(package, 'config', 'nav2_params.yaml')),
        OpaqueFunction(function=check_map),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            package, 'launch', 'hardware.launch.py')),
            condition=IfCondition(LC('start_hardware')),
            launch_arguments={'lidar': 'true', 'kinect': LC('kinect'),
                              'use_sim_time': LC('use_sim_time')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={'map': LC('map'), 'use_sim_time': LC('use_sim_time'),
                              'params_file': LC('params_file'), 'use_composition': 'False'}.items()),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LC('rviz')),
             parameters=[{'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}],
             arguments=['-d', LC('rviz_config')]),
    ])
