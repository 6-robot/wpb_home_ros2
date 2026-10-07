"""实验08的配套节点；实验程序单独启动。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('wpb_home_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'wpb_home.yaml')
    model_file = os.path.join(
        get_package_share_directory('wpb_home_description'), 'urdf', 'wpb_home_mani.urdf')

    # 底盘驱动：从 wpb_home.yaml 加载 kinect_height、kinect_pitch 等参数。
    wpb_home_core_cmd = Node(
        package='wpb_home_bringup',
        executable='wpb_home_core',
        name='wpb_home_core',
        output='screen',
        parameters=[config_file, {
            'serial_port': '/dev/ftdi',
            'odom': True,
            'imu': True,
            'use_sim_time': False,
        }],
    )

    # 发布机器人各关节的坐标变换。
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', model_file]), value_type=str),
            'use_sim_time': False,
        }],
    )

    # 激光雷达及数据过滤，向实验节点提供 /scan。
    lidar_cmd = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'use_sim_time': False,
        }],
        remappings=[('scan', 'scan_raw')],
    )

    lidar_filter_cmd = Node(
        package='wpb_home_bringup',
        executable='wpb_home_lidar_filter',
        parameters=[{'pub_topic': '/scan', 'use_sim_time': False}],
    )

    ld = LaunchDescription()

    ld.add_action(wpb_home_core_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(lidar_cmd)
    ld.add_action(lidar_filter_cmd)

    return ld
