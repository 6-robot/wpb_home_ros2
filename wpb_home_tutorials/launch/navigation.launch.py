"""实物机器人 Nav2 定位与导航。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_map(context):
    path = LaunchConfiguration('map').perform(context)
    if not os.path.isfile(path):
        raise RuntimeError('Map YAML does not exist: ' + path + '. Save a map first.')
    return []


def generate_launch_description():
    tutorials_dir = get_package_share_directory('wpb_home_tutorials')
    bringup_dir = get_package_share_directory('wpb_home_bringup')
    model_file = os.path.join(
        get_package_share_directory('wpb_home_description'), 'urdf', 'wpb_home_mani.urdf')
    common = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}

    # 启动参数。
    start_hardware_arg = DeclareLaunchArgument('start_hardware', default_value='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    joy_arg = DeclareLaunchArgument('joy', default_value='false')
    kinect_arg = DeclareLaunchArgument('kinect', default_value='false')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ftdi')
    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar')
    odom_arg = DeclareLaunchArgument('odom', default_value='true')
    imu_arg = DeclareLaunchArgument('imu', default_value='true')
    config_arg = DeclareLaunchArgument(
        'config', default_value=os.path.join(bringup_dir, 'config', 'wpb_home.yaml'))
    model_arg = DeclareLaunchArgument('model', default_value=model_file)
    joy_device_id_arg = DeclareLaunchArgument('joy_device_id', default_value='0')
    enable_qhd_points_arg = DeclareLaunchArgument('enable_qhd_points', default_value='true')
    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(tutorials_dir, 'maps', 'map.yaml'))
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(tutorials_dir, 'rviz', 'navi.rviz'))
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(tutorials_dir, 'config', 'nav2_params.yaml'))

    # 底盘参数文件包含 Kinect2 的高度和俯仰角。
    wpb_home_core_cmd = Node(
        package='wpb_home_bringup',
        executable='wpb_home_core',
        name='wpb_home_core',
        output='screen',
        parameters=[LaunchConfiguration('config'), common, {
            'serial_port': LaunchConfiguration('serial_port'),
            'odom': ParameterValue(LaunchConfiguration('odom'), value_type=bool),
            'imu': ParameterValue(LaunchConfiguration('imu'), value_type=bool),
        }],
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[common, {
            'robot_description': ParameterValue(
                Command(['xacro ', LaunchConfiguration('model')]), value_type=str),
        }],
    )

    lidar_cmd = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        parameters=[common, {
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[('scan', 'scan_raw')],
    )

    lidar_filter_cmd = Node(
        package='wpb_home_bringup',
        executable='wpb_home_lidar_filter',
        parameters=[common, {'pub_topic': '/scan'}],
    )

    kinect2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'include', 'include_kinect2_bridge.launch.py')),
        condition=IfCondition(LaunchConfiguration('kinect')),
        launch_arguments={'enable_qhd_points': LaunchConfiguration('enable_qhd_points')}.items(),
    )

    joy_cmd = Node(
        package='joy',
        executable='joy_node',
        condition=IfCondition(LaunchConfiguration('joy')),
        parameters=[{
            'device_id': ParameterValue(LaunchConfiguration('joy_device_id'), value_type=int),
            'deadzone': 0.12,
            'autorepeat_rate': 20.0,
        }],
    )

    teleop_cmd = Node(
        package='wpb_home_bringup',
        executable='wpb_home_js_vel',
        condition=IfCondition(LaunchConfiguration('joy')),
    )

    # 已单独启动硬件时，可用 start_hardware:=false 跳过这一组。
    hardware_cmd = GroupAction(
        condition=IfCondition(LaunchConfiguration('start_hardware')),
        scoped=False,
        actions=[
            wpb_home_core_cmd,
            robot_state_publisher_cmd,
            lidar_cmd,
            lidar_filter_cmd,
            kinect2_cmd,
            joy_cmd,
            teleop_cmd,
        ],
    )

    check_map_cmd = OpaqueFunction(function=check_map)

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'use_composition': 'False',
        }.items(),
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[common],
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    ld = LaunchDescription()

    ld.add_action(start_hardware_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(joy_arg)
    ld.add_action(kinect_arg)
    ld.add_action(rviz_arg)
    ld.add_action(serial_port_arg)
    ld.add_action(lidar_port_arg)
    ld.add_action(odom_arg)
    ld.add_action(imu_arg)
    ld.add_action(config_arg)
    ld.add_action(model_arg)
    ld.add_action(joy_device_id_arg)
    ld.add_action(enable_qhd_points_arg)
    ld.add_action(map_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(params_file_arg)

    ld.add_action(check_map_cmd)
    ld.add_action(hardware_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)

    return ld
