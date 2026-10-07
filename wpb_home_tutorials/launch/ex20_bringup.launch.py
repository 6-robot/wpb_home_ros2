"""实验20的配套节点；实验程序单独启动。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_files(context, map_file, waypoints_file):
    # 导航前需先保存地图和 kitchen、guest 航点。
    if not os.path.isfile(map_file):
        raise RuntimeError('Map file is missing: ' + map_file)
    if not os.path.isfile(waypoints_file):
        raise RuntimeError('Waypoints file is missing: ' + waypoints_file)
    return []


def generate_launch_description():
    bringup_dir = get_package_share_directory('wpb_home_bringup')
    config_file = os.path.join(bringup_dir, 'config', 'wpb_home.yaml')
    model_file = os.path.join(
        get_package_share_directory('wpb_home_description'), 'urdf', 'wpb_home_mani.urdf')
    tutorials_dir = get_package_share_directory('wpb_home_tutorials')
    rviz_file = os.path.join(tutorials_dir, 'rviz', 'fetch.rviz')
    map_file = os.path.join(tutorials_dir, 'maps', 'map.yaml')
    waypoints_file = os.path.expanduser('~/waypoint.xml')
    nav_params_file = os.path.join(tutorials_dir, 'config', 'nav2_params.yaml')

    check_files_cmd = OpaqueFunction(function=check_files, args=[map_file, waypoints_file])

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

    # Kinect2 图像、SD 点云和 QHD 点云。
    kinect2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'include', 'include_kinect2_bridge.launch.py')),
        launch_arguments={'enable_qhd_points': 'true'}.items(),
    )

    # Nav2 定位与导航。
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': nav_params_file,
            'use_sim_time': 'false',
            'use_composition': 'False',
        }.items(),
    )

    # 加载航点，并提供按航点名称导航的接口。
    wp_edit_cmd = Node(
        package='wp_map_tools',
        executable='wp_edit_node',
        name='wp_edit_node',
        output='screen',
        parameters=[{'load': waypoints_file, 'use_sim_time': False}],
    )

    wp_navi_server_cmd = Node(
        package='wp_map_tools',
        executable='wp_navi_server',
        name='wp_navi_server',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # 物体检测与抓取行为。
    behaviors_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('wpb_home_behaviors'), 'launch', 'behaviors.launch.py')),
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', rviz_file],
    )

    ld = LaunchDescription()

    ld.add_action(check_files_cmd)
    ld.add_action(wpb_home_core_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(lidar_cmd)
    ld.add_action(lidar_filter_cmd)
    ld.add_action(kinect2_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(wp_edit_cmd)
    ld.add_action(wp_navi_server_cmd)
    ld.add_action(behaviors_cmd)
    ld.add_action(rviz_cmd)

    return ld
