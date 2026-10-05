"""Display the real arm feedback using the description package's model."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = share('wpb_home_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=os.path.join(
            share('wpb_home_description'), 'urdf', 'wpb_home_mani.urdf')),
        DeclareLaunchArgument('serial_port', default_value='/dev/ftdi'),
        DeclareLaunchArgument('config', default_value=os.path.join(
            bringup, 'config', 'wpb_home.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rvizconfig', default_value=os.path.join(
            bringup, 'rviz', 'urdf.rviz')),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(
                 Command(['xacro ', LC('model')]), value_type=str)}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LC('rviz')),
             arguments=['-d', LC('rvizconfig')]),
        Node(package='wpb_home_bringup', executable='wpb_home_core', name='wpb_home_core',
             output='screen', parameters=[LC('config'), {'serial_port': LC('serial_port')}]),
    ])
