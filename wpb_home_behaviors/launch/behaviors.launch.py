"""Start only reusable behaviors; hardware and camera have separate owners."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('wpb_home_behaviors'),
                          'config', 'behaviors.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        Node(package='wpb_home_behaviors', executable='wpb_home_objects_3d',
             output='screen', parameters=[LC('params_file')]),
        Node(package='wpb_home_behaviors', executable='wpb_home_grab_server',
             output='screen', parameters=[LC('params_file')]),
    ])
