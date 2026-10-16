"""Run real executables on isolated DDS topics, without any hardware driver."""
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import uuid

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import Imu, JointState, LaserScan, Image, RegionOfInterest, PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from wpb_home_msgs.msg import Object

TOPICS = ['/kinect2/sd/points', '/objects_marker', '/cmd_vel', '/scan', '/imu', '/imu/data', '/wpb_home/mani_ctrl',
          '/wpb_home/behavior', '/wpb_home/objects_3d', '/wpb_home/grab_result',
          '/wpb_home/ctrl', '/wpb_home/pose_diff',
          '/kinect2/qhd/image_raw', '/face_detector_input', '/face_position',
          '/waterplus/navi_waypoint', '/waterplus/navi_result']


class Harness:
    def __init__(self, tmp_path):
        self.prefix = '/test_' + uuid.uuid4().hex
        self.context = Context()
        rclpy.init(context=self.context, domain_id=175 + os.getpid() % 40)
        self.node = rclpy.create_node('test_node', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(175 + os.getpid() % 40),
                        ROS_LOCALHOST_ONLY='1')
        self.processes = []
        self.tmp_path = tmp_path

    def start(self, executable, **params):
        path = Path(os.environ['TUTORIAL_BIN_DIR']) / executable
        args = [str(path), '--ros-args']
        for topic in TOPICS:
            args += ['-r', topic + ':=' + self.prefix + topic]
        for name, value in params.items():
            args += ['-p', name + ':=' + str(value).lower()]
        log = open(self.tmp_path / (Path(executable).name + '.log'), 'w+')
        process = subprocess.Popen(args, env=self.env, stdout=log, stderr=subprocess.STDOUT)
        self.processes.append((process, log))
        return process

    def publisher(self, cls, topic, sensor=False):
        return self.node.create_publisher(cls, self.prefix + topic,
                                          qos_profile_sensor_data if sensor else 10)

    def collect(self, cls, topic):
        messages = []
        self.node.create_subscription(cls, self.prefix + topic, messages.append, 10)
        return messages

    def wait(self, predicate, timeout=5, tick=None):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if tick:
                tick()
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        logs = []
        for _, log in self.processes:
            log.flush()
            logs.append(Path(log.name).read_text())
        pytest.fail('Timed out\n' + '\n'.join(logs))

    def pump(self, seconds=0.2):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.02)

    def close(self):
        for process, log in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            log.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def h(tmp_path):
    harness = Harness(tmp_path)
    yield harness
    harness.close()


def stopped(msg):
    return msg.linear.x == msg.linear.y == msg.angular.z == 0


def test_velocity_default_command_and_sigint_exit(h):
    values = h.collect(Twist, '/cmd_vel')
    process = h.start('ex03_velocity')
    h.wait(lambda: any(m.linear.x == 0.1 and m.linear.y == m.angular.z == 0 for m in values))
    process.send_signal(signal.SIGINT)
    h.wait(lambda: process.poll() is not None)
    assert process.returncode == 0


def test_lidar_empty_scan_and_obstacle_turn(h):
    pub = h.publisher(LaserScan, '/scan', sensor=True)
    values = h.collect(Twist, '/cmd_vel')
    process = h.start('ex05_lidar_behavior')
    h.wait(lambda: pub.get_subscription_count() > 0)
    pub.publish(LaserScan())
    h.pump()
    assert not values
    scan = LaserScan(range_min=0.1, range_max=10.0, ranges=[3.0])
    h.wait(lambda: any(m.linear.x == 0.1 for m in values), tick=lambda: pub.publish(scan))
    values.clear()
    scan.ranges = [0.5]
    h.wait(lambda: any(m.angular.z == 0.3 for m in values), tick=lambda: pub.publish(scan))
    values.clear()
    scan.ranges = [3.0]
    h.wait(lambda: any(m.linear.x == 0.1 and m.angular.z == 0 for m in values),
           timeout=10, tick=lambda: pub.publish(scan))
    assert process.poll() is None


def test_imu_proportional_heading_command(h):
    pub = h.publisher(Imu, '/imu/data', sensor=True)
    values = h.collect(Twist, '/cmd_vel')
    h.start('ex07_imu_behavior')
    h.wait(lambda: pub.get_subscription_count() > 0)
    msg = Imu()
    msg.orientation.w = 1.0
    h.wait(lambda: any(abs(m.angular.z - 0.9) < 1e-5 and m.linear.x == 0.1 for m in values),
           tick=lambda: pub.publish(msg))
    values.clear()
    angle = math.radians(90)
    msg.orientation.z = math.sin(angle / 2)
    msg.orientation.w = math.cos(angle / 2)
    h.wait(lambda: any(abs(m.angular.z) < 1e-5 and m.linear.x == 0.1 for m in values),
           tick=lambda: pub.publish(msg))


def grab_behavior_executable():
    return str(Path(get_package_prefix('wpb_home_behaviors')) / 'lib' / 'wpb_home_behaviors' / 'wpb_home_grab_server')


class GrabIO:
    def __init__(self, h, with_arm=True):
        self.h = h
        self.objects = h.publisher(Object, '/wpb_home/objects_3d')
        self.control = h.publisher(String, '/wpb_home/behavior')
        self.poses = h.publisher(Pose2D, '/wpb_home/pose_diff')
        self.velocity = h.collect(Twist, '/cmd_vel')
        self.arm = h.collect(JointState, '/wpb_home/mani_ctrl') if with_arm else []
        self.results = h.collect(String, '/wpb_home/grab_result')
        self.commands = h.collect(String, '/wpb_home/behavior')
        self.resets = h.collect(String, '/wpb_home/ctrl')
        self.pose = Pose2D(x=0.4, y=0.2, theta=0.1)
        self.feedback = True
        self.timer = h.node.create_timer(
            0.02, lambda: self.poses.publish(self.pose) if self.feedback else None)

    def start(self, **params):
        # These scenarios use a 6 cm lateral target explicitly.
        defaults = dict(auto_start=True, align_y=0.06, lift_wait=0.1, grip_wait=0.1,
                        raise_wait=0.1, reset_wait=0.05, stop_wait=0.1)
        defaults.update(params)
        self.h.start(grab_behavior_executable(), **defaults)
        self.h.wait(lambda: self.objects.get_subscription_count() > 0 and
                    self.poses.get_subscription_count() > 0 and
                    self.control.get_subscription_count() > 0)

    def object(self, x, y, z=0.7, stale=False):
        msg = Object(x=[x], y=[y], z=[z])
        msg.header.frame_id = 'base_footprint'
        if not stale:
            msg.header.stamp = self.h.node.get_clock().now().to_msg()
        self.objects.publish(msg)

    def speed(self, x, y=0.0):
        return bool(self.velocity) and self.velocity[-1].linear.x == pytest.approx(x) and \
            self.velocity[-1].linear.y == pytest.approx(y) and self.velocity[-1].angular.z == 0

    def cancel(self):
        self.control.publish(String(data='stop grab'))
        self.h.wait(lambda: any(m.data == 'grab failed' for m in self.results))


@pytest.mark.parametrize('object_x', [0.98, 1.02])
def test_grab_behavior_full_sequence_and_release(h, object_x):
    g = GrabIO(h)
    g.start()
    h.wait(lambda: any(m.data == 'start objects' for m in g.commands))
    g.object(object_x, 0.06, stale=True)
    h.pump(0.1)
    assert not g.arm
    g.object(object_x, 0.06)
    h.wait(lambda: len(g.resets) == 1)
    h.pump(0.15)
    assert all(stopped(m) for m in g.velocity)
    assert g.arm[0].position == pytest.approx([0.74, 0.15])
    g.pose = Pose2D()
    h.wait(lambda: g.speed(0.1))
    distance = 0.7 * (object_x - 0.7)
    g.object(1.5, -0.2, z=0.9)  # Moving targets cannot replace the saved measurement.
    g.pose = Pose2D(x=distance - 0.03)
    h.pump(0.2)
    assert g.speed(0.1) and len(g.arm) == 1
    g.pose = Pose2D(x=distance + 0.03)
    h.wait(lambda: g.speed(-0.1))
    g.pose = Pose2D(x=distance - 0.005)
    h.wait(lambda: len(g.resets) == 2)
    h.pump(0.15)
    assert g.speed(0.0)
    assert g.arm[1].position == pytest.approx([0.74, 0.045])
    assert g.arm[2].position == pytest.approx([0.79, 0.045])
    g.pose = Pose2D()
    h.wait(lambda: g.speed(-0.1))
    g.pose = Pose2D(x=-distance + 0.03)
    h.pump(0.2)
    assert g.speed(-0.1)
    g.pose = Pose2D(x=-distance - 0.03)
    h.wait(lambda: g.speed(0.1))
    g.pose = Pose2D(x=-distance + 0.005)
    h.wait(lambda: any(m.data == 'grab done' for m in g.results))
    h.pump(0.2)
    count = len(g.velocity)
    h.pump(0.2)
    assert len(g.velocity) == count and g.speed(0.0)
    assert len(g.arm) == 3 and all(not m.velocity for m in g.arm)
    assert [m.data for m in g.resets] == ['pose_diff reset'] * 2
    assert sum(m.data == 'start objects' for m in g.commands) == 1
    assert all(m.linear.y == m.angular.z == 0 for m in g.velocity)


def test_grab_behavior_odometry_alignment_and_recheck(h):
    g = GrabIO(h)
    g.start(lift_wait=18.0)
    h.wait(lambda: any(m.data == 'start objects' for m in g.commands))
    g.object(1.12, -0.08)
    h.wait(lambda: len(g.resets) == 1)
    h.pump(0.15)
    assert all(stopped(m) for m in g.velocity)
    g.pose = Pose2D()
    h.wait(lambda: g.speed(0.05, -0.05))
    g.object(1.0, 0.06, z=0.9)
    g.pose = Pose2D(x=0.115, y=-0.03)
    h.wait(lambda: g.speed(0.0, -0.05))
    g.pose = Pose2D(x=0.12, y=-0.14)
    h.wait(lambda: sum(m.data == 'start objects' for m in g.commands) == 2)
    assert not g.arm and g.speed(0.0)
    g.object(1.0, 0.06, stale=True)
    h.pump(0.1)
    assert not g.arm
    g.object(1.0, 0.06, z=0.72)
    h.wait(lambda: bool(g.arm))
    assert g.arm[0].position == pytest.approx([0.76, 0.15])
    assert len(g.resets) == 1
    g.cancel()


def test_grab_behavior_pose_feedback_timeout(h):
    g = GrabIO(h)
    g.start(pose_timeout=0.15)
    h.wait(lambda: any(m.data == 'start objects' for m in g.commands))
    g.object(1.2, 0.1)
    h.wait(lambda: len(g.resets) == 1)
    g.pose = Pose2D()
    h.wait(lambda: g.speed(0.05, 0.05))
    g.feedback = False
    h.wait(lambda: any(m.data == 'grab failed' for m in g.results))
    assert not g.arm
    h.pump()
    assert g.speed(0.0)
    assert 'Pose feedback timed out' in Path(h.processes[-1][1].name).read_text()


def test_grab_behavior_invalid_targets_timeout_and_restart(h):
    g = GrabIO(h)
    g.start(auto_start=False, detection_timeout=2.0, lift_wait=18.0)
    h.pump()
    assert not g.velocity and not g.arm and not g.results
    g.control.publish(String(data='start grab'))
    h.wait(lambda: any(m.data == 'start objects' for m in g.commands))
    invalid = [
        Object(),
        Object(x=[1.0], y=[], z=[0.7]),
        Object(x=[float('nan')], y=[0.0], z=[0.7]),
        Object(x=[0.5], y=[0.0], z=[0.7]),
        Object(x=[2.1], y=[0.0], z=[0.7]),
        Object(x=[1.0], y=[0.7], z=[0.7]),
        Object(x=[1.0], y=[0.0], z=[-0.1]),
        Object(x=[1.0], y=[0.0], z=[1.2]),
    ]
    for msg in invalid:
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp = h.node.get_clock().now().to_msg()
        g.objects.publish(msg)
        h.pump(0.05)
    wrong_frame = Object(x=[1.0], y=[0.0], z=[0.7])
    wrong_frame.header.frame_id = 'camera'
    wrong_frame.header.stamp = h.node.get_clock().now().to_msg()
    g.objects.publish(wrong_frame)
    h.wait(lambda: any(m.data == 'grab failed' for m in g.results))
    assert not g.arm and all(stopped(m) for m in g.velocity)

    g.commands.clear()
    g.control.publish(String(data='start grab'))
    h.wait(lambda: any(m.data == 'start objects' for m in g.commands))
    g.object(1.0, 0.06)
    h.wait(lambda: bool(g.arm))
    failures = len(g.results)
    g.control.publish(String(data='grab stop'))
    h.wait(lambda: len(g.results) > failures, timeout=1.0)
    h.pump()
    assert g.results[-1].data == 'grab failed'
    assert len(g.arm) == 1 and g.speed(0.0)


def test_grab_behavior_fixed_speed_and_motion_timeout(h):
    g = GrabIO(h)
    g.start(motion_timeout=0.5)
    h.wait(lambda: any(m.data == 'start objects' for m in g.commands))
    g.object(1.9, -0.5)
    h.wait(lambda: len(g.resets) == 1)
    g.pose = Pose2D()
    h.wait(lambda: g.speed(0.05, -0.05))
    g.control.publish(String(data='start grab'))
    h.wait(lambda: any(m.data == 'grab failed' for m in g.results))
    assert all(abs(m.linear.x) <= 0.05 and abs(m.linear.y) <= 0.05 for m in g.velocity)
    assert not g.arm
    log = Path(h.processes[-1][1].name).read_text()
    assert 'Grab already active; duplicate start ignored' in log
    assert 'Odometry movement timed out' in log
    h.pump()
    assert g.speed(0.0)


def test_grab_behavior_reset_timeout(h):
    g = GrabIO(h)
    g.start(motion_timeout=0.3)
    h.wait(lambda: any(m.data == 'start objects' for m in g.commands))
    g.object(1.0, 0.06)
    h.wait(lambda: any(m.data == 'grab failed' for m in g.results))
    assert all(stopped(m) for m in g.velocity)
    assert 'Pose reset timed out' in Path(h.processes[-1][1].name).read_text()


@pytest.mark.parametrize('with_arm', [False, True])
def test_grab_behavior_arm_and_approach_guards(h, with_arm):
    g = GrabIO(h, with_arm=with_arm)
    g.start(detection_timeout=1.0, max_forward_distance=0.1)
    h.wait(lambda: any(m.data == 'start objects' for m in g.commands))
    g.object(1.0, 0.06)
    h.wait(lambda: any(m.data == 'grab failed' for m in g.results))
    assert not g.arm and all(stopped(m) for m in g.velocity)
    log = Path(h.processes[-1][1].name).read_text()
    expected = ('Approach distance outside calibrated range' if with_arm else
                'No valid object or arm driver before timeout')
    assert expected in log


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='OpenCV face window requires a graphical display')
def test_face_image_forwarding_and_valid_roi(h):
    images = h.publisher(Image, '/kinect2/qhd/image_raw', sensor=True)
    rois = h.publisher(RegionOfInterest, '/face_position')
    forwarded = h.collect(Image, '/face_detector_input')
    process = h.start('ex13_face')
    h.wait(lambda: rois.get_subscription_count() > 0 and images.get_subscription_count() > 0)
    msg = Image(height=8, width=8, encoding='bgr8', step=24, data=bytes(192))
    h.wait(lambda: forwarded, tick=lambda: images.publish(msg))
    rois.publish(RegionOfInterest(x_offset=1, y_offset=1, width=4, height=4))
    h.pump()
    assert forwarded[0].width == 8 and process.poll() is None


def test_waypoint_fixed_name_and_arrival_log(h):
    requests = h.collect(String, '/waterplus/navi_waypoint')
    result = h.publisher(String, '/waterplus/navi_result')
    process = h.start('ex10_waypoint')
    h.wait(lambda: requests)
    assert requests[0].data == '1'
    h.pump()
    assert process.poll() is None
    result.publish(String(data='navi done'))
    h.wait(lambda: 'Arrived !' in Path(h.processes[-1][1].name).read_text())
    assert process.poll() is None
    process.send_signal(signal.SIGINT)
    h.wait(lambda: process.poll() is not None)
    assert process.returncode == 0


def test_objects_plane_cluster_and_empty_frame(h):
    pub = h.publisher(PointCloud2, '/kinect2/sd/points', sensor=True)
    results = h.collect(Object, '/wpb_home/objects_3d')
    markers = h.collect(MarkerArray, '/objects_marker')
    process = h.start('objects_publisher', auto_start=True)
    h.wait(lambda: pub.get_subscription_count() > 0)
    points = [(0.55 + i * 0.01, -0.45 + j * 0.01, 0.7, 0)
              for i in range(90) for j in range(90)]
    points += [(0.9 + i * 0.01, -0.05 + j * 0.01, 0.85, 0)
               for i in range(15) for j in range(15)]
    fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(['x', 'y', 'z'])]
    fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
    cloud = PointCloud2(height=1, width=len(points), fields=fields, point_step=16,
                        row_step=16 * len(points), is_dense=True,
                        data=b''.join(struct.pack('<fffI', *p) for p in points))
    cloud.header.frame_id = 'base_footprint'
    h.wait(lambda: any(m.x for m in results), tick=lambda: pub.publish(cloud))
    obj = next(m for m in results if m.x)
    assert obj.header.frame_id == 'base_footprint'
    assert obj.x[0] == pytest.approx(1.04, abs=0.02)
    assert obj.z[0] == pytest.approx(0.85, abs=0.02)
    h.pump()
    results.clear()
    cloud.width = 0
    cloud.row_step = 0
    cloud.data = b''
    markers.clear()
    h.wait(lambda: any(len(m.markers) == 1 and m.markers[0].action == 3 for m in markers),
           tick=lambda: pub.publish(cloud))
    h.pump()
    assert not results
    assert process.poll() is None
