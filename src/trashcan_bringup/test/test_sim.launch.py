"""
End-to-end check of the headless simulation.

Starts sim.launch.py without GUI and checks that the robot drives, that the
LiDAR sees the room and that the bin fill-level sensor reports an empty bin,
then a filling one once a box of trash is dropped in.
"""

import math
import subprocess
import time
import unittest

from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

# Generous, since a CI runner renders the sensors on the CPU.
STARTUP_TIMEOUT_S = 180.0
DRIVE_SPEED_MPS = 0.3
DRIVE_DURATION_S = 2.0


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('trashcan_bringup'), 'launch', 'sim.launch.py'])),
        launch_arguments={'gui': 'false', 'rviz': 'false'}.items(),
    )
    return LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class Latest:
    """Keeps the most recent message of a topic."""

    def __init__(self, node, msg_type, topic, qos=10):
        self.msg = None
        node.create_subscription(msg_type, topic, self._store, qos)

    def _store(self, msg):
        self.msg = msg


class TestSimulation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('trashcan_sim_test')
        cls.clock = Latest(cls.node, Clock, '/clock')
        cls.odom = Latest(cls.node, Odometry, '/odom')
        cls.scan = Latest(cls.node, LaserScan, '/scan', qos_profile_sensor_data)
        cls.fill_level = Latest(cls.node, Float32, '/bin/fill_level')
        cls.cmd_vel = cls.node.create_publisher(TwistStamped, '/cmd_vel', 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, condition, timeout_s, what):
        deadline = time.monotonic() + timeout_s
        while not condition():
            self.assertLess(time.monotonic(), deadline, f'timed out waiting for {what}')
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def sim_time(self):
        stamp = self.clock.msg.clock
        return stamp.sec + stamp.nanosec * 1e-9

    def position(self):
        p = self.odom.msg.pose.pose.position
        return p.x, p.y

    def test_1_drives_forward(self):
        self.spin_until(lambda: self.odom.msg and self.clock.msg, STARTUP_TIMEOUT_S, '/odom')
        start = self.position()

        # Drive for a span of simulated time, so a slow runner still covers the distance.
        twist = TwistStamped()
        twist.twist.linear.x = DRIVE_SPEED_MPS
        end_time = self.sim_time() + DRIVE_DURATION_S
        while self.sim_time() < end_time:
            twist.header.stamp = self.node.get_clock().now().to_msg()
            self.cmd_vel.publish(twist)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.cmd_vel.publish(TwistStamped())

        # Let the odometry catch up with the last commands.
        settle_until = self.sim_time() + 0.5
        self.spin_until(lambda: self.sim_time() > settle_until, 30.0, 'the robot to settle')
        x, y = self.position()
        distance = math.hypot(x - start[0], y - start[1])
        self.assertGreater(distance, 0.3, f'the robot moved only {distance:.3f} m')

    def test_2_lidar_sees_the_room(self):
        self.spin_until(lambda: self.scan.msg, STARTUP_TIMEOUT_S, '/scan')
        ranges = self.scan.msg.ranges
        self.assertEqual(len(ranges), 720)
        # The room walls are 2 to 2.5 m away; more than half the beams must hit them.
        walls = [r for r in ranges if math.isfinite(r) and r > 1.0]
        self.assertGreater(len(walls), len(ranges) // 2, 'the LiDAR does not see the walls')

    def test_3_fill_sensor_reports_empty_then_filling_bin(self):
        self.spin_until(lambda: self.fill_level.msg, STARTUP_TIMEOUT_S, '/bin/fill_level')
        self.assertLess(self.fill_level.msg.data, 5.0, 'an empty bin should read about 0 %')

        # Drop a 20 cm box into the bin, wherever the robot is now.
        self.spin_until(lambda: self.odom.msg, STARTUP_TIMEOUT_S, '/odom')
        x, y = self.position()
        box = (
            '<sdf version="1.9"><model name="trash"><link name="link">'
            '<inertial><mass>0.3</mass><inertia><ixx>0.002</ixx><iyy>0.002</iyy>'
            '<izz>0.002</izz></inertia></inertial>'
            '<collision name="c"><geometry><box><size>0.2 0.2 0.2</size></box>'
            '</geometry></collision>'
            '<visual name="v"><geometry><box><size>0.2 0.2 0.2</size></box>'
            '</geometry></visual>'
            '</link></model></sdf>')
        subprocess.run(
            ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'minimal_room',
             '-string', box, '-x', str(x), '-y', str(y), '-z', '0.8'],
            check=True, timeout=60)

        # The box rests about 0.17 m under the sensor: roughly 60 % full.
        self.spin_until(
            lambda: self.fill_level.msg.data > 40.0, 60.0, 'the fill level to rise')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
