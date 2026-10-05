"""Exercise the real core executable over a pseudo terminal, without hardware."""

import os
import pty
import signal
import struct
import subprocess
import tempfile
import time
import uuid

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int32MultiArray, String, UInt64
from tf2_msgs.msg import TFMessage


def feedback_frame(module, payload, offset=8):
    frame = bytearray(offset)
    frame[:3] = bytes([0x55, 0xAA, 0x40])
    frame[4] = module
    frame.extend(payload)
    frame[3] = len(frame) + 1 - 8
    frame.append(sum(frame) & 0xFF)
    return frame


def command_frame(module, method, payload=b''):
    frame = bytes([0x55, 0xAA, 0x40, len(payload), module, method]) + payload
    return frame + bytes([sum(frame) & 0xFF])


@pytest.fixture(scope='module')
def ros_context():
    # Use a separate domain and remap every hardware topic into a unique namespace.
    previous = {key: os.environ.get(key) for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY')}
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    context = rclpy.Context()
    rclpy.init(context=context)
    yield context
    context.shutdown()
    for key, value in previous.items():
        if value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = value


class Driver:
    def __init__(self, context, parameters):
        self.namespace = '/driver_test_' + uuid.uuid4().hex
        self.node = rclpy.create_node('probe', namespace=self.namespace, context=context)
        self.executor = SingleThreadedExecutor(context=context)
        self.executor.add_node(self.node)
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.log = tempfile.TemporaryFile()
        self.received = {}
        self.subscriptions = []
        for topic, message_type in (
            ('wpb_home/ad', Int32MultiArray),
            ('wpb_home/input', Int32MultiArray),
            ('wpb_home/sound_source', UInt64),
            ('wpb_home/pose_diff', Pose2D),
            ('imu/data', Imu), ('odom', Odometry),
            ('joint_states', JointState), ('tf', TFMessage),
        ):
            self.received[topic] = []
            self.subscriptions.append(self.node.create_subscription(
                message_type, self.namespace + '/' + topic,
                lambda msg, key=topic: self.received[key].append(msg), 100))
        self.publishers = {
            topic: self.node.create_publisher(message_type, self.namespace + '/' + topic, 10)
            for topic, message_type in (
                ('wpb_home/ctrl', String), ('wpb_home/output', Int32MultiArray),
                ('wpb_home/mani_ctrl', JointState), ('cmd_vel', Twist))
        }
        command = [os.environ['WPB_HOME_CORE_EXECUTABLE'], '--ros-args',
                   '-r', '__ns:=' + self.namespace,
                   '-p', 'serial_port:=' + os.ttyname(self.slave)]
        for topic in (*self.received, *self.publishers):
            command.extend(['-r', '/' + topic + ':=' + self.namespace + '/' + topic])
        for key, value in parameters.items():
            command.extend(['-p', key + ':=' + str(value).lower()])
        self.process = subprocess.Popen(command, stdout=self.log, stderr=self.log)

    def spin(self, duration=0.3):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert self.process.poll() is None, 'core exited unexpectedly'
            self.executor.spin_once(timeout_sec=0.02)

    def wait_for(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.spin(0.03)
        assert predicate(), 'timed out waiting for driver interface'

    def ready(self):
        self.wait_for(lambda: self.received['joint_states'] and all(
            publisher.get_subscription_count() for publisher in self.publishers.values()))

    def read_serial(self, count, timeout=3.0):
        data = bytearray()
        deadline = time.monotonic() + timeout
        while len(data) < count and time.monotonic() < deadline:
            try:
                data.extend(os.read(self.master, 4096))
            except BlockingIOError:
                pass
            self.spin(0.02)
        assert len(data) == count, f'expected {count} serial bytes, received {data.hex()}'
        return bytes(data)

    def expect_command(self, topic, message, expected):
        self.publishers[topic].publish(message)
        assert self.read_serial(len(expected)) == expected

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.log.seek(0)
        print(self.log.read().decode(errors='replace'))
        self.log.close()
        self.executor.shutdown()
        self.node.destroy_node()
        os.close(self.master)
        os.close(self.slave)


@pytest.fixture
def driver_factory(ros_context):
    drivers = []

    def create(**parameters):
        driver = Driver(ros_context, parameters)
        drivers.append(driver)
        driver.ready()
        return driver

    yield create
    for driver in reversed(drivers):
        driver.close()


def test_ad_and_digital_io(driver_factory):
    driver = driver_factory()
    values = [0, 1, 255, 256, 65535, 42, 1024, 2048, 4095, 5000, 12, 34, 56, 78, 90]
    for group in range(3):
        payload = bytes([group + 1]) + struct.pack('>5H', *values[group * 5:group * 5 + 5])
        os.write(driver.master, feedback_frame(0x07, payload))
    os.write(driver.master, feedback_frame(0x06, bytes([0b1010])))
    driver.wait_for(lambda: any(
        list(msg.data) == values for msg in driver.received['wpb_home/ad']))
    driver.wait_for(lambda: any(
        list(msg.data) == [0, 1, 0, 1] for msg in driver.received['wpb_home/input']))
    # First eight channels, positive means on; short arrays preserve remaining channels.
    for values, mask in (([1, 0, 2, -1, 0, 1, 0, 1, 1], 0xA5),
                         ([0, 1], 0xA6), ([], 0xA6), ([0] * 8, 0)):
        driver.expect_command('wpb_home/output', Int32MultiArray(data=values),
                              command_frame(0x06, 0x70, bytes([mask])))


def test_sound_query_and_one_result_per_feedback(driver_factory):
    driver = driver_factory()
    driver.expect_command('wpb_home/ctrl', String(data='sound local'),
                          bytes.fromhex('55 aa 40 00 0a 09 52'))
    assert not driver.received['wpb_home/sound_source']
    os.write(driver.master, feedback_frame(0x0A, struct.pack('>H', 270)))
    driver.wait_for(lambda: driver.received['wpb_home/sound_source'])
    driver.spin(0.3)
    assert [msg.data for msg in driver.received['wpb_home/sound_source']] == [270]
    os.write(driver.master, feedback_frame(0x0A, struct.pack('>H', 90)))
    driver.wait_for(lambda: len(driver.received['wpb_home/sound_source']) == 2)
    assert driver.received['wpb_home/sound_source'][-1].data == 90


@pytest.mark.parametrize('imu,odom', [(True, True), (True, False), (False, True), (False, False)])
def test_startup_switches_and_pose_reset(driver_factory, imu, odom):
    driver = driver_factory(imu=imu, odom=odom)
    client = driver.node.create_client(
        GetParameters, driver.namespace + '/wpb_home_core/get_parameters')
    assert client.wait_for_service(timeout_sec=5)
    future = client.call_async(GetParameters.Request(names=['imu', 'odom']))
    driver.wait_for(future.done)
    assert [value.bool_value for value in future.result().values] == [imu, odom]
    # Known quaternion, gyro and accelerometer feedback in the existing wire format.
    for kind, values in ((1, (10, 20, 30)), (2, (40, 50, 60)), (3, (1, 0)), (4, (0, 0))):
        os.write(driver.master, feedback_frame(0x09, bytes([kind]) + struct.pack(
            '>' + 'i' * len(values), *values), offset=6))
    os.write(driver.master, feedback_frame(0x08, bytes([3]) + struct.pack('>ii', 0, 1840)))
    driver.wait_for(lambda: any(msg.theta > 0.1 for msg in driver.received['wpb_home/pose_diff']))
    driver.spin(0.3)
    assert bool(driver.received['imu/data']) == imu
    assert bool(driver.received['odom']) == odom
    assert bool(driver.received['tf']) == odom
    if imu:
        driver.wait_for(lambda: driver.received['imu/data'][-1].linear_acceleration.z == 60.0)
        msg = driver.received['imu/data'][-1]
        assert msg.orientation.w == 1.0
        assert msg.angular_velocity.y == 20.0
    driver.publishers['wpb_home/ctrl'].publish(String(data='pose_diff reset'))
    driver.wait_for(lambda: driver.received['wpb_home/pose_diff'][-1].theta == 0.0)


def test_demo_commands_and_legacy_velocities(driver_factory):
    driver = driver_factory()
    driver.wait_for(lambda: driver.received['imu/data'] and driver.received['odom'])
    # The lateral Twist format used by the examples must still reach all three wheels.
    twist = Twist()
    twist.linear.y = 0.1
    expected = (command_frame(0x08, 0x60, struct.pack('>HiHi', 1, 97, 2, 97)) +
                command_frame(0x08, 0x60, struct.pack('>HiHi', 3, -195, 4, 0)))
    driver.expect_command('cmd_vel', twist, expected)
    for velocities, lift_speed, gripper_speed in (
        ([], 980, 890), ([0.25], 490, 890), ([0.25, 2.0], 490, 356)
    ):
        msg = JointState(name=['lift', 'gripper'], position=[0.5, 0.07], velocity=velocities)
        payload = struct.pack('>HiiHii', 5, lift_speed, 11737, 6, gripper_speed, 30000)
        driver.expect_command('wpb_home/mani_ctrl', msg, command_frame(0x08, 0x63, payload))
    for msg in (JointState(), JointState(position=[0.5]),
                JointState(position=[float('nan'), 0.07])):
        driver.publishers['wpb_home/mani_ctrl'].publish(msg)
        driver.spin(0.1)
        with pytest.raises(BlockingIOError):
            os.read(driver.master, 4096)
    assert driver.process.poll() is None


def test_configurable_default_manipulator_velocities(driver_factory):
    driver = driver_factory(mani_default_lift_velocity=0.25, mani_default_gripper_velocity=2.0)
    msg = JointState(name=['lift', 'gripper'], position=[0.5, 0.07])
    payload = struct.pack('>HiiHii', 5, 490, 11737, 6, 356, 30000)
    driver.expect_command('wpb_home/mani_ctrl', msg, command_frame(0x08, 0x63, payload))
