# Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import lanelet2.core as l2_core
import lanelet2.io as l2_io
import lanelet2.projection as l2_proj
import os
import threading
import sys
import tty
import termios
import atexit

class PointToOSMNode(Node):
    def __init__(self):
        super().__init__('point_to_osm_node')

        # Parameters
        # UofA 53.528321801207966, -113.53042672991906
        self.declare_parameter('reference_lat', 53.528321801207966)
        self.declare_parameter('reference_lon', -113.53042672991906)
        self.declare_parameter('output_file', 'clicked_points.osm')
        self.declare_parameter('autosave', True)

        self.reference_lat = self.get_parameter('reference_lat').get_parameter_value().double_value
        self.reference_lon = self.get_parameter('reference_lon').get_parameter_value().double_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.autosave = self.get_parameter('autosave').get_parameter_value().bool_value

        # Projection and Map setup
        self.projection = l2_proj.UtmProjector(l2_io.Origin(self.reference_lat, self.reference_lon))
        self.map = l2_core.LaneletMap()
        self.points = []

        # Default point type
        self.current_type = 'curb'

        # Key to type mapping
        self.key_type_map = {
            '1': 'curb',
            '2': 'tree',
            '3': 'sign'
        }

        # Subscriber
        self.sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )

        # Terminal setup
        self.terminal_fd = sys.stdin.fileno()
        self.old_terminal_settings = termios.tcgetattr(self.terminal_fd)
        tty.setcbreak(self.terminal_fd)

        # Register cleanup with atexit
        atexit.register(self.restore_terminal)

        # Start keyboard thread
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

        self.get_logger().info('point_to_osm_node started. Reference: ({}, {})'.format(
            self.reference_lat, self.reference_lon))
        self.get_logger().info('Keyboard type switcher active. Press:')
        for key, type_name in self.key_type_map.items():
            self.get_logger().info(f"  {key}: {type_name}")

    def point_callback(self, msg: PointStamped):
        osm_point = l2_core.Point3d(len(self.points) + 1, msg.point.x, msg.point.y, 0.0)
        osm_point.attributes["type"] = self.current_type
        self.map.add(osm_point)
        self.points.append(osm_point)

        self.get_logger().info(f"Added point {osm_point.id} as '{self.current_type}'")

        if self.autosave:
            self.save_map()

    def keyboard_loop(self):
        while True:
            ch = sys.stdin.read(1)
            if ch in self.key_type_map:
                self.current_type = self.key_type_map[ch]
                self.get_logger().info(f"Switched point type to '{self.current_type}'")

    def save_map(self):
        if not self.points:
            self.get_logger().warn('No points to save.')
            return
        l2_io.write(self.output_file, self.map, self.projection)
        self.get_logger().info(f"Saved {len(self.points)} points to {self.output_file}")

    def restore_terminal(self):
        try:
            termios.tcsetattr(self.terminal_fd, termios.TCSADRAIN, self.old_terminal_settings)
            self.get_logger().info("Terminal restored to normal mode.")
        except Exception as e:
            self.get_logger().warn(f"Failed to restore terminal: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointToOSMNode()

    def on_shutdown():
        try:
            node.restore_terminal()
        except Exception as e:
            node.get_logger().warn(f"Failed to restore terminal on shutdown: {e}")

    rclpy.get_default_context().on_shutdown(on_shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handles Ctrl+C gracefully
    finally:
        node.destroy_node()  # ROS2 handles shutdown automatically

if __name__ == '__main__':
    main()
