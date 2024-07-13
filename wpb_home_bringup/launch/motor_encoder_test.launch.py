from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # wpb_home_motor_encoder node
        Node(
            package='wpb_home_bringup',
            executable='wpb_home_motor_encoder',
            name='wpb_home_motor_encoder',
            output='screen'
        ),
        
        # joy node
        Node(
            package='joy',
            executable='joy_node',
            name='wpb_home_joy',
            respawn=True,
            parameters=[
                {'dev': '/dev/input/js0'},
                {'deadzone': 0.12}
            ],
            output='screen'
        ),
        
        # Global parameters for teleoperation
        DeclareLaunchArgument(
            'axis_linear',
            default_value='1',
            description='Axis number for linear movement'
        ),
        DeclareLaunchArgument(
            'axis_angular',
            default_value='0',
            description='Axis number for angular movement'
        ),
        DeclareLaunchArgument(
            'scale_linear',
            default_value='1.0',
            description='Scale for linear movement'
        ),
        DeclareLaunchArgument(
            'scale_angular',
            default_value='1.0',
            description='Scale for angular movement'
        ),

        # teleop node
        Node(
            package='wpb_home_bringup',
            executable='wpb_home_js_vel',
            name='teleop',
            output='screen',
            parameters=[
                {'axis_linear': LaunchConfiguration('axis_linear')},
                {'axis_angular': LaunchConfiguration('axis_angular')},
                {'scale_linear': LaunchConfiguration('scale_linear')},
                {'scale_angular': LaunchConfiguration('scale_angular')}
            ]
        ),
    ])
