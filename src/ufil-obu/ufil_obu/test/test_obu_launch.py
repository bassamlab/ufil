# Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Launch the OBU state estimation and CAM pipeline and check the own vehicle object."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import AccelWithCovarianceStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ufil_msgs.msg import ObjectStamped

NAMESPACE = 'obu_test'


@pytest.mark.launch_test
def generate_test_description():
    launch_dir = os.path.join(get_package_share_directory('ufil_obu'), 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(launch_dir, 'state_estimation.launch.xml')),
            launch_arguments={
                'namespace': NAMESPACE,
                'robot_localization_params_file':
                    os.path.join(os.path.dirname(__file__), 'robot_localization.yaml'),
            }.items()),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(launch_dir, 'cam.launch.xml')),
            launch_arguments={'namespace': NAMESPACE, 'target_port': '47999'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestObuPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('obu_launch_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for(self, condition, timeout=30.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def test_nodes_are_running(self):
        expected = {'ekf_filter_node_odom', 'ekf_filter_node_map', 'navsat_transform_node',
                    'object_aggregation', 'vehicle_cam_generator_node'}

        def nodes_up():
            names = {name for name, namespace in self.node.get_node_names_and_namespaces()
                     if namespace == f'/{NAMESPACE}'}
            return expected <= names
        self.assertTrue(self.wait_for(nodes_up), 'OBU nodes did not start')

    def test_object_is_published_in_utm(self):
        received = []
        self.node.create_subscription(
            ObjectStamped, f'/{NAMESPACE}/object', received.append, 10)
        accel_pub = self.node.create_publisher(
            AccelWithCovarianceStamped, f'/{NAMESPACE}/filtered/accel', 10)
        odom_pub = self.node.create_publisher(Odometry, f'/{NAMESPACE}/odometry/global', 10)
        fix_pub = self.node.create_publisher(NavSatFix, f'/{NAMESPACE}/filtered/gps', 10)
        self.assertTrue(self.wait_for(lambda: fix_pub.get_subscription_count() > 0))

        start = time.monotonic()
        while time.monotonic() - start < 20.0 and not received:
            stamp = self.node.get_clock().now().to_msg()
            accel = AccelWithCovarianceStamped()
            accel.header.stamp = stamp
            odom = Odometry()
            odom.header.stamp = stamp
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = 5.0
            fix = NavSatFix()
            fix.header.stamp = stamp
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.latitude = 50.7779
            fix.longitude = 6.0778
            accel_pub.publish(accel)
            odom_pub.publish(odom)
            fix_pub.publish(fix)
            self.wait_for(lambda: False, timeout=0.1)

        self.assertTrue(received, 'no object published')
        msg = received[0]
        self.assertEqual(msg.header.frame_id, 'utm_32n')
        self.assertAlmostEqual(msg.object.state.state.x, 293990.85, delta=1.0)
        self.assertAlmostEqual(msg.object.state.state.v_x, 5.0)
        self.assertAlmostEqual(msg.object.dimension.dimension.length, 4.5)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
