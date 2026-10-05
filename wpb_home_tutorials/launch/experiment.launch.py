"""Start a textbook program and its helper; hardware is started separately."""
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

PROGRAMS = {
    'velocity': 'ex03_velocity', 'lidar_data': 'ex04_lidar',
    'lidar_behavior': 'ex05_lidar_behavior', 'imu_data': 'ex06_imu_data',
    'imu_behavior': 'ex07_imu_behavior', 'waypoint': 'ex10_waypoint',
    'image': 'ex12_image', 'hsv': 'ex12_hsv', 'face': 'ex14_face',
    'pointcloud': 'ex13_pointcloud', 'objects': 'ex15_objects',
    'arm': 'ex16_manipulator', 'grab': 'ex17_grab',
}


def start(context):
    experiment = LC('experiment').perform(context)
    common = {'use_sim_time': LC('use_sim_time').perform(context) == 'true'}
    executable = PROGRAMS[experiment]
    nodes = [Node(package='wpb_home_tutorials', executable=executable,
                  output='screen', parameters=[common])]
    if experiment == 'face':
        nodes.append(Node(package='wpb_home_tutorials', executable='face_detector.py',
                          output='screen', parameters=[common]))
    if experiment == 'grab':
        nodes.append(Node(package='wpb_home_tutorials', executable='objects_publisher',
                          output='screen', parameters=[common]))
    if experiment in ('pointcloud', 'objects', 'grab') and LC('rviz').perform(context) == 'true':
        config = 'objects.rviz' if experiment == 'grab' else 'pointcloud.rviz'
        nodes.append(Node(package='rviz2', executable='rviz2', parameters=[common],
                          arguments=['-d', os.path.join(share('wpb_home_tutorials'), 'rviz', config)]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('experiment', choices=list(PROGRAMS)),
        DeclareLaunchArgument('reference', default_value='false', choices=['true', 'false'],
                              description='Compatibility argument; both values use the single exXX program'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start),
    ])
