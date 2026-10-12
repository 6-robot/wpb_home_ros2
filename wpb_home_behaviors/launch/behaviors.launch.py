"""Start only reusable behaviors; hardware and camera have separate owners."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='wpb_home_behaviors', executable='wpb_home_objects_3d',
             output='screen', parameters=[{
                 'auto_start': False,
                 'object_min_height': 0.02,
                 'cluster_tolerance': 0.03,
                 'min_cluster_size': 30,
                 'stable_frame_count': 3,
                 'stable_distance_threshold': 0.05,
                 'lock_radius': 0.10,
             }]),
        Node(package='wpb_home_behaviors', executable='wpb_home_grab_server',
             output='screen'),
    ])
