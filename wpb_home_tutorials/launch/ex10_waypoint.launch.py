# 完整参考示例：wpb_home_tutorials/launch/ex10_waypoint.launch.py
# 先运行 ex10_bringup.launch.py 和 ex10_map_tools.launch.py，设置初始位姿与航点，再运行本文件。
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    path = os.path.join(get_package_share_directory('wpb_home_tutorials'), 'launch', 'experiment.launch.py')
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments={'experiment': 'waypoint', 'reference': 'true'}.items())])
