# 完整参考示例：wpb_home_tutorials/launch/ex18_tts.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([Node(package='wpb_home_tutorials', executable='ex18_tts', output='screen')])
