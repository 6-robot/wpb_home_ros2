import os
from launch import LaunchDescription 
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    map_file = LaunchConfiguration('map')
    
    nav_param_file = os.path.join(
        get_package_share_directory('wpb_home_tutorials'),
        'config',
        'ex09_nav2.yaml'
    )
    
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 
        'launch'
    )
    
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'False',
            'params_file': nav_param_file}.items(),
    )
    
    rviz_file = os.path.join(get_package_share_directory('wp_map_tools'), 'rviz', 'navi.rviz')
    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file]
        )

    wp_edit_cmd = Node(
            package='wp_map_tools',
            executable='wp_edit_node',
            name='wp_edit_node',
            parameters=[{'load': LaunchConfiguration('waypoints')}]
        )
    
    wp_navi_server_cmd = Node(
            package='wp_map_tools',
            executable='wp_navi_server',
            name='wp_navi_server'
        )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('map', default_value=os.path.join(get_package_share_directory('wpb_home_tutorials'), 'maps', 'map.yaml'),
                                      description='Absolute path to map YAML'))
    ld.add_action(DeclareLaunchArgument('waypoints', default_value=os.path.expanduser('~/waypoint.xml')))
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(wp_edit_cmd)
    ld.add_action(wp_navi_server_cmd)

    return ld
