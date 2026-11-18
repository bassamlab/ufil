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

"""Launch the OSN pipeline and push a synthetic point cloud through it."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from ufil_msgs.msg import ObjectList

NAMESPACE = 'osn_test'


def static_transform(parent, child, z=0.0):
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--z', str(z), '--frame-id', parent, '--child-frame-id', child])


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('ufil_osn'), 'launch', 'osn.launch.xml')
    return LaunchDescription([
        static_transform('map', 'sensor_node_base'),
        static_transform('sensor_node_base', 'sensor_node', z=2.0),
        static_transform('sensor_node', 'lidar'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file),
            launch_arguments={'namespace': NAMESPACE}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def box_cloud(stamp, x):
    """Dense box, 2 m x 1 m x 1.2 m, centred 1 m above ground (z = -1 in the lidar frame)."""
    points = []
    for i in range(21):
        for j in range(11):
            for k in range(7):
                points.append((x - 1.0 + 0.1 * i, -0.5 + 0.1 * j, -1.6 + 0.2 * k))
    return point_cloud2.create_cloud_xyz32(Header(frame_id='lidar', stamp=stamp), points)


class TestOsnPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('osn_launch_test')

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

    def test_components_are_loaded_and_connected(self):
        def nodes_up():
            names = self.node.get_node_names_and_namespaces()
            return all((name, f'/{NAMESPACE}') in names
                       for name in ('ufil_pcl_preprocessor', 'ufil_osn_tracker'))
        self.assertTrue(self.wait_for(nodes_up), 'pipeline nodes did not start')

        topic = f'/{NAMESPACE}/dynamic_cloud'
        self.assertTrue(self.wait_for(
            lambda: self.node.count_publishers(topic) == 1 and
            self.node.count_subscribers(topic) == 1))

    def test_parameters_are_loaded(self):
        for node_name, expected in (
                ('ufil_pcl_preprocessor', {'filter.voxel_size': 0.1,
                                           'target_frame': 'sensor_node_base',
                                           'use_sim_time': False}),
                ('ufil_osn_tracker', {'association.threshold': 4.0,
                                      'target_frame': 'map'})):
            client = AsyncParameterClient(self.node, f'/{NAMESPACE}/{node_name}')
            self.assertTrue(client.wait_for_services(timeout_sec=30.0))
            future = client.get_parameters(list(expected))
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            values = [
                rclpy.parameter.parameter_value_to_python(v) for v in future.result().values]
            self.assertEqual(dict(zip(expected, values)), expected)

    def test_point_cloud_produces_tracks(self):
        received = []
        self.node.create_subscription(
            ObjectList, f'/{NAMESPACE}/object_list', received.append, 10)
        publisher = self.node.create_publisher(
            PointCloud2, f'/{NAMESPACE}/input_cloud', qos_profile_sensor_data)
        self.assertTrue(self.wait_for(lambda: publisher.get_subscription_count() > 0))

        def track_received():
            return any(msg.objects for msg in received)

        start = time.monotonic()
        x = 0.0
        while time.monotonic() - start < 30.0 and not track_received():
            publisher.publish(box_cloud(self.node.get_clock().now().to_msg(), x))
            x += 0.05
            self.wait_for(lambda: False, timeout=0.1)

        self.assertTrue(track_received(), 'no track published')
        msg = next(msg for msg in received if msg.objects)
        self.assertEqual(msg.header.frame_id, 'map')
        self.assertAlmostEqual(msg.objects[0].state.state.x, x, delta=2.0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
