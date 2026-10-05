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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState, LaserScan, Image, RegionOfInterest, PointCloud2, PointField
from std_msgs.msg import String
from wpr_simulation2.msg import Object

TOPICS = ['/kinect2/sd/points', '/objects_marker', '/cmd_vel', '/scan', '/imu', '/imu/data', '/wpb_home/mani_ctrl',
          '/wpb_home/behavior', '/wpb_home/objects_3d', '/wpb_home/grab_result',
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


def test_grab_behavior_full_sequence_and_continuous_stop(h):
    pub = h.publisher(Object, '/wpb_home/objects_3d')
    velocity = h.collect(Twist, '/cmd_vel')
    arm = h.collect(JointState, '/wpb_home/mani_ctrl')
    result = h.collect(String, '/wpb_home/grab_result')
    commands = h.collect(String, '/wpb_home/behavior')
    h.start(grab_behavior_executable(), auto_start=True, lift_wait=0.2, grip_wait=0.2, raise_wait=0.2,
            retreat_time=0.3, approach_seconds_per_meter=1.0)
    h.wait(lambda: pub.get_subscription_count() > 0)
    pub.publish(Object())
    h.pump()
    assert not arm and all(stopped(m) for m in velocity)
    msg = Object(x=[1.0], y=[0.0], z=[0.7])
    msg.header.frame_id = 'base_footprint'
    h.wait(lambda: any(m.data == 'grab done' for m in result),
           tick=lambda: pub.publish(msg))
    assert len(arm) == 3
    assert list(arm[0].position) == pytest.approx([0.7, 0.15])
    assert list(arm[1].position) == pytest.approx([0.7, 0.07])
    assert list(arm[2].position) == pytest.approx([0.75, 0.07])
    assert all(not m.velocity for m in arm)
    assert any(m.linear.x > 0 for m in velocity)
    assert any(m.linear.x < 0 for m in velocity)
    assert sum(stopped(m) for m in velocity) >= 10
    assert any(m.data == 'stop objects' for m in commands)
    h.pump()
    assert stopped(velocity[-1])


def test_grab_behavior_stale_target_and_stop_command(h):
    pub = h.publisher(Object, '/wpb_home/objects_3d')
    control = h.publisher(String, '/wpb_home/behavior')
    values = h.collect(Twist, '/cmd_vel')
    h.collect(JointState, '/wpb_home/mani_ctrl')
    results = h.collect(String, '/wpb_home/grab_result')
    h.start(grab_behavior_executable(), auto_start=True, object_timeout=0.15)
    h.wait(lambda: pub.get_subscription_count() > 0)
    msg = Object(x=[1.2], y=[0.1], z=[0.7])
    msg.header.frame_id = 'base_footprint'
    h.wait(lambda: any(m.linear.x > 0 for m in values), tick=lambda: pub.publish(msg))
    # Drain queued DDS callbacks and wait for the observed timeout stop. A fixed
    # sleep can end before queued target messages have reached the controller.
    h.wait(lambda: values and stopped(values[-1]), timeout=1.0)
    h.pump(0.1)
    assert stopped(values[-1])
    h.wait(lambda: any(m.data == 'grab failed' for m in results),
           tick=lambda: control.publish(String(data='stop grab')))
    h.pump()
    assert stopped(values[-1])


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='OpenCV face window requires a graphical display')
def test_face_image_forwarding_and_valid_roi(h):
    images = h.publisher(Image, '/kinect2/qhd/image_raw', sensor=True)
    rois = h.publisher(RegionOfInterest, '/face_position')
    forwarded = h.collect(Image, '/face_detector_input')
    process = h.start('ex14_face')
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
    results.clear()
    cloud.width = 0
    cloud.row_step = 0
    cloud.data = b''
    h.wait(lambda: any(not m.x for m in results), tick=lambda: pub.publish(cloud))
    assert process.poll() is None
