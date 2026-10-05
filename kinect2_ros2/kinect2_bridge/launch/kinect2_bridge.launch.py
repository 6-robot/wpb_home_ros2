import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


parameters=[    {'base_name': 'kinect2',
                'sensor': '',
                'calib_path': LaunchConfiguration('calib_path'),
                'fps_limit': -1.0,
                'use_png': False,
                'jpeg_quality': 90,
                'png_level': 1,
                'use_vaapi': ParameterValue(LaunchConfiguration('use_vaapi'), value_type=bool),
                'depth_method': 'default',
                'depth_device': -1,
                'reg_method': 'cpu',
                'reg_device': -1,
                'max_depth': 12.0,
                'min_depth': 0.1,
                'queue_size': 5,
                'bilateral_filter': True,
                'edge_aware_filter': True,
                'worker_threads': 4,
                'publish_tf': True}]

point_cloud_res='sd' # ['hd','qhd','sd']

def generate_launch_description():
    kinect2 = Node(
            package='kinect2_bridge',
            executable='kinect2_bridge',
            emulate_tty=True,
            name='kinect2_bridge',
            parameters=parameters,
            output='screen')
    

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='both',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='pointcloud',
                remappings=[
                    ("rgb/camera_info", f"/kinect2/{point_cloud_res}/camera_info"),
                    ("rgb/image_rect_color", f"/kinect2/{point_cloud_res}/image_raw_rect"),
                    ("depth_registered/image_rect",f"/kinect2/{point_cloud_res}/image_depth_rect"),
                    ("points",f"/kinect2/{point_cloud_res}/points")]
            ),
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='pointcloud_qhd',
                condition=IfCondition(LaunchConfiguration('enable_qhd_points')),
                remappings=[
                    ('rgb/camera_info', '/kinect2/qhd/camera_info'),
                    ('rgb/image_rect_color', '/kinect2/qhd/image_raw_rect'),
                    ('depth_registered/image_rect', '/kinect2/qhd/image_depth_rect'),
                    ('points', '/kinect2/qhd/points')],
            ),
        ],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_vaapi', default_value='false',
            description='Allow VA-API RGB decoding; false uses the TurboJPEG fallback'),
        DeclareLaunchArgument(
            'calib_path', default_value=PathJoinSubstitution([
                FindPackageShare('kinect2_bridge'), 'data']),
            description='Calibration root, containing one directory per device serial'),
        DeclareLaunchArgument(
            'enable_qhd_points', default_value='true',
            description='Publish QHD XYZRGB points in addition to the SD point cloud'),
        kinect2, container
    ])

