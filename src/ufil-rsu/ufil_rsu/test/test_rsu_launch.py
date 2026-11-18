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

"""Launch the RSU central fusion and CAM reception and check the fused object list."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import launch_testing
import launch_testing.asserts
import pytest
import rclpy
from ufil_msgs.msg import Object, ObjectList


@pytest.mark.launch_test
def generate_test_description():
    launch_dir = os.path.join(get_package_share_directory('ufil_rsu'), 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(launch_dir, 'central_fusion.launch.xml'))),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(launch_dir, 'cam.launch.xml')),
            launch_arguments={'listen_port': '47998'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def car(stamp, x):
    msg = ObjectList()
    msg.header.frame_id = 'map'
    msg.header.stamp = stamp
    obj = Object()
    obj.id = 1
    obj.state.state.x = x
    obj.state.state.y = 20.0
    obj.state.state.v_x = 5.0
    for index in (0, 9, 18, 27, 54):
        obj.state.covariance[index] = 0.1
    obj.dimension.dimension.length = 4.5
    obj.dimension.dimension.width = 1.8
    obj.dimension.dimension.height = 1.5
    for index in (0, 4, 8):
        obj.dimension.covariance[index] = 0.1
    obj.existence_probability = 0.95
    obj.classification.classification[0] = 1.0
    msg.objects.append(obj)
    return msg


class TestRsuPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('rsu_launch_test')

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

    def test_cam_objects_reach_the_fusion(self):
        topic = '/rsu/cam/object_list'
        self.assertTrue(self.wait_for(
            lambda: self.node.count_publishers(topic) >= 1 and
            self.node.count_subscribers(topic) >= 1),
            'CAM reception and central fusion are not connected')

    def test_object_lists_are_fused(self):
        received = []
        self.node.create_subscription(ObjectList, '/rsu/object_list', received.append, 10)
        publisher = self.node.create_publisher(ObjectList, '/osn/object_list', 10)
        self.assertTrue(self.wait_for(lambda: publisher.get_subscription_count() > 0))

        start = time.monotonic()
        x = 0.0
        while time.monotonic() - start < 20.0 and not any(m.objects for m in received):
            publisher.publish(car(self.node.get_clock().now().to_msg(), x))
            x += 0.5
            self.wait_for(lambda: False, timeout=0.1)

        fused = [m for m in received if m.objects]
        self.assertTrue(fused, 'no fused objects published')
        self.assertEqual(fused[0].header.frame_id, 'map')
        self.assertAlmostEqual(fused[0].objects[0].state.state.y, 20.0, delta=1.0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
