"""实验14的配套节点；实验程序单独启动。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('wpb_home_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'wpb_home.yaml')
    model_file = os.path.join(
        get_package_share_directory('wpb_home_description'), 'urdf', 'wpb_home_mani.urdf')
    tutorials_dir = get_package_share_directory('wpb_home_tutorials')
    rviz_file = os.path.join(tutorials_dir, 'rviz', 'pointcloud.rviz')

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

    # Kinect2 图像、SD 点云和 QHD 点云。
    kinect2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'include', 'include_kinect2_bridge.launch.py')),
        launch_arguments={'enable_qhd_points': 'true'}.items(),
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
    )

    ld = LaunchDescription()

    ld.add_action(wpb_home_core_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(kinect2_cmd)
    ld.add_action(rviz_cmd)

    return ld
