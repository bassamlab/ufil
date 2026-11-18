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

"""
ROS 2 node that turns clicked points (/clicked_point) into a Lanelet2 OSM landmark map.

The type of the next point is the ``point_type`` parameter. It can be changed with
``ros2 param set`` or, when the node runs in an interactive terminal, by pressing the number
of the type (1 for the first entry of ``point_types``).
"""

import sys
import threading
from typing import List, Optional

from geometry_msgs.msg import PointStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.parameter import Parameter

from .point_map import PointMap, PointMapConfig


class PclMapBuilderParameters:
    """Declares, reads and validates the ROS parameters of ``PclMapBuilderNode``."""

    def __init__(self, node: Node):
        read_only = ParameterDescriptor(read_only=True)
        self.origin_latitude: float = node.declare_parameter(
            'origin.latitude', 53.528321801207966, read_only).value
        self.origin_longitude: float = node.declare_parameter(
            'origin.longitude', -113.53042672991906, read_only).value
        self.point_types: List[str] = list(node.declare_parameter(
            'point_types', ['curb', 'tree', 'sign'], read_only).value)
        self.output_file: str = node.declare_parameter(
            'output_file', 'clicked_points.osm', read_only).value
        self.autosave: bool = node.declare_parameter('autosave', True, read_only).value
        self.point_type: str = node.declare_parameter(
            'point_type', self.point_types[0] if self.point_types else '',
            ParameterDescriptor(description='Type assigned to the next clicked point.')).value

        self.map_config().validate()
        if not self.output_file:
            raise ValueError('output_file must not be empty.')
        self.validate_point_type(self.point_type)

    def map_config(self) -> PointMapConfig:
        return PointMapConfig(self.origin_latitude, self.origin_longitude,
                              tuple(self.point_types))

    def validate_point_type(self, point_type: str) -> None:
        if point_type not in self.point_types:
            raise ValueError(
                f"point_type '{point_type}' is not one of point_types {self.point_types}.")


class PclMapBuilderNode(Node):
    """Adds every point received on ``clicked_point`` to the map and saves it as OSM."""

    def __init__(self, **kwargs):
        super().__init__('pcl_map_builder', **kwargs)
        self._settings = PclMapBuilderParameters(self)
        self._map = PointMap(self._settings.map_config())
        self._unsaved_points = 0
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._subscription = self.create_subscription(
            PointStamped, 'clicked_point', self._on_point, 10)

        self.get_logger().info(
            f'Map origin at ({self._settings.origin_latitude}, '
            f'{self._settings.origin_longitude}), saving to '
            f"'{self._settings.output_file}'. Current point type: "
            f"'{self._settings.point_type}'.")

    @property
    def point_map(self) -> PointMap:
        return self._map

    @property
    def point_type(self) -> str:
        return self._settings.point_type

    def _on_set_parameters(self, parameters: List[Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != 'point_type':
                continue
            try:
                self._settings.validate_point_type(parameter.value)
            except ValueError as error:
                return SetParametersResult(successful=False, reason=str(error))
            self._settings.point_type = parameter.value
            self.get_logger().info(f"Switched point type to '{parameter.value}'.")
        return SetParametersResult(successful=True)

    def select_point_type(self, key: str) -> None:
        """Select the point type with the given 1-based number key, ignore other keys."""
        if key.isdigit() and 1 <= int(key) <= len(self._settings.point_types):
            self.set_parameters([Parameter(
                'point_type', value=self._settings.point_types[int(key) - 1])])

    def _on_point(self, msg: PointStamped) -> None:
        point_id = self._map.add_point(msg.point.x, msg.point.y, self._settings.point_type)
        self._unsaved_points += 1
        self.get_logger().info(
            f"Added point {point_id} as '{self._settings.point_type}' at "
            f'({msg.point.x:.2f}, {msg.point.y:.2f}) in frame {msg.header.frame_id}.')
        if self._settings.autosave:
            self.save()

    def save(self) -> None:
        if self._unsaved_points == 0:
            return
        try:
            count = self._map.save(self._settings.output_file)
        except (OSError, RuntimeError) as error:
            self.get_logger().error(
                f"Saving the map to '{self._settings.output_file}' failed: {error}")
            return
        self._unsaved_points = 0
        self.get_logger().debug(f"Saved {count} points to '{self._settings.output_file}'.")

    def destroy_node(self) -> None:
        self.save()
        super().destroy_node()


class _KeyboardTypeSwitcher:
    """Reads single key presses from an interactive terminal to switch the point type."""

    def __init__(self, node: PclMapBuilderNode):
        import termios
        import tty
        self._node = node
        self._fd = sys.stdin.fileno()
        self._termios = termios
        self._settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        threading.Thread(target=self._loop, daemon=True).start()
        node.get_logger().info('Press a number to switch the point type: ' + ', '.join(
            f'{index}: {name}' for index, name in enumerate(node.point_map.point_types, 1)))

    def _loop(self) -> None:
        while True:
            key = sys.stdin.read(1)
            if not key:
                return
            self._node.select_point_type(key)

    def restore(self) -> None:
        self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._settings)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    try:
        node = PclMapBuilderNode()
    except ValueError as error:
        get_logger('pcl_map_builder').fatal(f'Invalid parameters: {error}')
        rclpy.try_shutdown()
        sys.exit(1)
    keyboard = _KeyboardTypeSwitcher(node) if sys.stdin.isatty() else None
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if keyboard is not None:
            keyboard.restore()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
