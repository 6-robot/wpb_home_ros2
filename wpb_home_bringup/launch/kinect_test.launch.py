import os

from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('wpb_home_description')
    default_model_path = urdf_tutorial_path / 'urdf/wpb_home.urdf'
    bringup_path = get_package_share_path('wpb_home_bringup')
    default_rviz_config_path = bringup_path / 'rviz/sensor.rviz'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # 机器人配置文件 wpb_home.yaml 文件路径
    config_file = os.path.join(
        get_package_share_directory('wpb_home_bringup'),
        'config',
        'wpb_home.yaml'
    )
    wpb_home_core = Node(
        package='wpb_home_bringup',
        executable='wpb_home_core',
        name='wpb_home_core',
        parameters=[{'serial_port': '/dev/ftdi'},config_file],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    kinect2_launch_dir = os.path.join(get_package_share_directory('wpb_home_bringup'), 'launch')

    kinect2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kinect2_launch_dir, 'include_kinect2_bridge.launch.py')
        )
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        wpb_home_core,
        robot_state_publisher_node,
        rviz_node,
        kinect2_cmd
    ])
