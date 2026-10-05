import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_file = os.path.join(get_package_share_directory('wpb_home_bringup'), 'rviz', 'sensor.rviz')
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_file]),
    ])
