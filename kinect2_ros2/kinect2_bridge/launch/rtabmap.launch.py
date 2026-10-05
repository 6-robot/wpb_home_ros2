"""QHD RGB-D mapping, adapted from krepa098/kinect2_ros2 (3bc7974).

Start the camera separately. The robot mode uses existing odometry and URDF.
For a standalone Kinect, enable visual_odometry; it uses a separate odom frame.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    name = LC('name').perform(context).strip('/')
    visual = IfCondition(LC('visual_odometry')).evaluate(context)
    frame = LC('frame_id').perform(context)
    if not frame:
        frame = name + '_link' if visual else 'base_footprint'
    odom_topic = '/rtabmap/odom' if visual else LC('odom_topic').perform(context)
    common = {
        'frame_id': frame,
        'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool),
        'approx_sync': True,
        'approx_sync_max_interval': 0.05,
        'topic_queue_size': 10,
        'sync_queue_size': 10,
        'qos': 2,
        'qos_camera_info': 2,
    }
    rgbd = [
        ('rgb/image', '/' + name + '/qhd/image_raw_rect'),
        ('rgb/camera_info', '/' + name + '/qhd/camera_info'),
        ('depth/image', '/' + name + '/qhd/image_depth_rect'),
    ]
    mapping = dict(common, subscribe_depth=True, subscribe_rgb=True,
                   subscribe_scan_cloud=True, subscribe_odom_info=visual,
                   qos_scan=2, qos_odom=1)
    mapping.update({'Rtabmap/DetectionRate': '3.5', 'Mem/IncrementalMemory': 'true'})
    remappings = rgbd + [
        ('scan_cloud', '/' + name + '/qhd/points'),
        ('odom', odom_topic), ('odom_info', '/rtabmap/odom_info'),
    ]
    nodes = []
    if visual:
        nodes.append(Node(
            package='rtabmap_odom', executable='rgbd_odometry',
            name='rgbd_odometry', namespace='rtabmap', output='screen',
            parameters=[common, {'odom_frame_id': LC('visual_odom_frame_id'),
                                 'publish_tf': True}],
            remappings=rgbd + [('odom', '/rtabmap/odom')]))
    nodes.extend([
        Node(package='rtabmap_slam', executable='rtabmap',
             name='rtabmap', namespace='rtabmap', output='screen',
             parameters=[mapping, {'database_path': os.path.expanduser(LC('database_path').perform(context))}],
             remappings=remappings),
        Node(package='rtabmap_viz', executable='rtabmap_viz',
             name='rtabmap_viz', namespace='rtabmap', output='screen',
             condition=IfCondition(LC('viz')), parameters=[mapping],
             remappings=remappings),
    ])
    return nodes


def generate_launch_description():
    defaults = {
        'name': ('kinect2', 'Camera topic prefix'),
        'frame_id': ('', 'Base frame: defaults to base_footprint, or kinect2_link in visual mode'),
        'odom_topic': ('/odom', 'Existing robot odometry topic'),
        'visual_odometry': ('false', 'Use RGB-D odometry for a standalone Kinect'),
        'visual_odom_frame_id': ('kinect2_odom', 'Standalone visual odometry frame'),
        'database_path': ('~/.ros/wpb_kinect_rtabmap.db', 'RTAB-Map database path'),
        'viz': ('true', 'Start RTAB-Map visualization'),
        'use_sim_time': ('false', 'Use ROS simulation time'),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v, description=d)
          for k, (v, d) in defaults.items()],
        OpaqueFunction(function=launch_setup),
    ])
