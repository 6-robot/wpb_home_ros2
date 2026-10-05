"""One owner for the real base, TF and optional sensors; no teaching program."""
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    bringup = share('wpb_home_bringup')
    defaults = {
        'serial_port': '/dev/ftdi', 'lidar_port': '/dev/rplidar',
        'lidar': 'false', 'kinect': 'false', 'joy': 'false',
        'use_sim_time': 'false', 'odom': 'true', 'imu': 'true',
        'config': os.path.join(bringup, 'config', 'wpb_home.yaml'),
        'model': os.path.join(share('wpb_home_description'), 'urdf', 'wpb_home_mani.urdf'),
        'joy_device_id': '0', 'enable_qhd_points': 'true',
    }
    common = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        Node(package='wpb_home_bringup', executable='wpb_home_core', name='wpb_home_core',
             output='screen', parameters=[LC('config'), common, {
                 'serial_port': LC('serial_port'),
                 'odom': ParameterValue(LC('odom'), value_type=bool),
                 'imu': ParameterValue(LC('imu'), value_type=bool)}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[common, {'robot_description': ParameterValue(
                 Command(['xacro ', LC('model')]), value_type=str)}]),
        Node(package='rplidar_ros', executable='rplidar_composition',
             condition=IfCondition(LC('lidar')), parameters=[common, {
                 'serial_port': LC('lidar_port'), 'serial_baudrate': 115200,
                 'frame_id': 'laser', 'inverted': False, 'angle_compensate': True}],
             remappings=[('scan', 'scan_raw')]),
        Node(package='wpb_home_bringup', executable='wpb_home_lidar_filter',
             condition=IfCondition(LC('lidar')), parameters=[common, {'pub_topic': '/scan'}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            bringup, 'launch', 'include', 'include_kinect2_bridge.launch.py')),
            condition=IfCondition(LC('kinect')),
            launch_arguments={'enable_qhd_points': LC('enable_qhd_points')}.items()),
        Node(package='joy', executable='joy_node', condition=IfCondition(LC('joy')),
             parameters=[{'device_id': ParameterValue(LC('joy_device_id'), value_type=int),
                          'deadzone': 0.12, 'autorepeat_rate': 20.0}]),
        Node(package='wpb_home_bringup', executable='wpb_home_js_vel',
             condition=IfCondition(LC('joy'))),
    ])
