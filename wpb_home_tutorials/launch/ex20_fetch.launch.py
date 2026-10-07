# 完整参考示例：wpb_home_tutorials/launch/ex20_fetch.launch.py
# 先运行 fetch.launch.py 准备真机，再运行本文件开始运动。
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='wpb_home_tutorials', executable='ex20_fetch', output='screen'),
    ])
