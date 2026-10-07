from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_cmd = Node(
        package='joy',
        executable='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.12,
            'autorepeat_rate': 20.0
        }]
    )

    teleop_cmd = Node(
        package='wpb_home_bringup',
        executable='wpb_home_js_vel',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(joy_cmd)
    ld.add_action(teleop_cmd)

    return ld
