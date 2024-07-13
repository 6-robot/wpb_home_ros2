from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, FindPackage
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([FindPackageShare('wpb_home_bringup'), 'urdf', 'wpb_home_mani.urdf']),
        description='Path to the robot urdf file'
    )

    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Flag to enable GUI'
    )

    declare_rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([FindPackageShare('wpb_home_bringup'), 'rviz', 'urdf.rviz']),
        description='Path to the RViz config file'
    )

    # Set robot_description parameter using xacro
    set_robot_description = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            LaunchConfiguration('model')
        ],
        output='screen',
        shell=True
    )

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', LaunchConfiguration('model')])}],
    )

    # rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
    )

    # wpb_home_core node
    wpb_home_core_node = Node(
        package='wpb_home_bringup',
        executable='wpb_home_core',
        name='wpb_home_core',
        output='screen',
        parameters=[
            {'serial_port': '/dev/ftdi'},
            PathJoinSubstitution([FindPackageShare('wpb_home_bringup'), 'config', 'wpb_home.yaml'])
        ],
    )

    return LaunchDescription([
        declare_model_arg,
        declare_gui_arg,
        declare_rvizconfig_arg,
        set_robot_description,
        robot_state_publisher_node,
        rviz_node,
        wpb_home_core_node
    ])
