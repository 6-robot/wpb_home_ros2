from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare a launch argument for the serial port, with a default value
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ftdi',
            description='Serial port device'
        ),
        
        # Define the node for wpb_home_test_mani
        Node(
            package='wpb_home_bringup',  # Name of the package where the executable is located
            executable='wpb_home_test_mani',  # Name of the executable to run
            name='wpb_home_test_mani',  # Name of the node
            output='screen',  # Show node output in the screen
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}],  # Use the serial_port launch argument as a parameter
        ),
    ])
