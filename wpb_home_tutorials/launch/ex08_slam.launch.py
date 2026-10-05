import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    slam_params = {
        'use_sim_time': False,
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_footprint',
        'map_update_interval': 2.0,
    }
    slam_cmd = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    rviz_file = os.path.join(get_package_share_directory('wpb_home_tutorials'), 'rviz', 'slam.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file]
    )

    joy_cmd = Node(
        package='joy',
        executable='joy_node',
        name='wpb_home_joy',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.12,
            'autorepeat_rate': 20.0
        }]
    )

    teleop_cmd = Node(
        package='wpb_home_bringup',
        executable='wpb_home_js_vel',
        name='teleop',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(joy_cmd)
    ld.add_action(teleop_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)

    return ld
