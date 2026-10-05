# 完整参考示例：wpb_home_tutorials/launch/ex01_intro.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = os.path.join(get_package_share_directory('wpb_home_bringup'), 'launch', 'urdf.launch.py')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path), launch_arguments={'gui': 'true'}.items())])
