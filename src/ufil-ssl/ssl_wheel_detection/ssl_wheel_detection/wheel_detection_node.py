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
import itertools
from typing import Iterable, List, Tuple

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.subscription import Subscription
from scipy import ndimage
from scipy.interpolate import NearestNDInterpolator
from scipy.spatial.transform import Rotation
from skimage.feature import peak_local_max
from ssl_vehicle_tracking_msgs.msg import Wheel, WheelArray
from std_msgs.msg import Header


def unpack_data(grid: OccupancyGrid) -> tuple[np.ndarray, float, Pose]:
    data: np.ndarray = np.reshape(
        grid.data, (grid.info.height, grid.info.width)
    ).astype('B').T
    grid_resolution: float = grid.info.resolution
    grid_origin: Pose = grid.info.origin
    return data, grid_resolution, grid_origin


def pack_data(results: np.ndarray, input_data: OccupancyGrid) -> WheelArray:
    header: Header = input_data.header
    wheels = [
        Wheel(x=x, y=y, intensity=val, wheel_id=WheelDetectionNode.new_id())
        for x, y, val in results
    ]
    return WheelArray(header=header, wheels=wheels)


class WheelDetectionNode(Node):
    new_id = itertools.count().__next__

    def __init__(self) -> None:
        super().__init__('wheel_detection_node')
        self.get_logger().info('Starting wheel detection node')

        self.__threshold: int = 10
        self.declare_parameter('threshold', self.__threshold)
        self.__threshold = self.get_parameter('threshold').value
        self.get_logger().info('Use pressure threshold: %i' % self.__threshold)

        self.__local_max_radius: int = 2
        self.declare_parameter('local_max_radius', self.__local_max_radius)
        self.__local_max_radius = self.get_parameter('local_max_radius').value
        self.get_logger().info(
            'Detect local maxima in radius: %i' % self.__local_max_radius
        )

        self.__dilation_radius: int = 3
        self.declare_parameter('dilation_radius', self.__dilation_radius)
        self.__dilation_radius = self.get_parameter('dilation_radius').value
        self.get_logger().info(
            'Dilate local maxima with radius: %i' % self.__dilation_radius
        )

        self.__exclude_border: int = 1
        self.declare_parameter('exclude_border', self.__exclude_border)
        self.__exclude_border = self.get_parameter('exclude_border').value
        self.get_logger().info(
            'Exclude borders of size: %i' % self.__exclude_border
        )

        self.__dilation_footprint: np.ndarray = (
            ndimage.iterate_structure(
                ndimage.generate_binary_structure(2, 1),
                self.__dilation_radius,
            )
        )
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.__subscription: Subscription = self.create_subscription(
            OccupancyGrid,
            'load_map',
            self.receive,
            qos,
        )
        self.__publisher: Publisher = self.create_publisher(
            WheelArray,
            'wheel_detection',
            qos,
        )

    def receive(self, data: OccupancyGrid) -> None:
        unpacked_data = unpack_data(data)
        results = self.run(*unpacked_data)

        packed_data: WheelArray = pack_data(*results, input_data=data)
        self.__publisher.publish(packed_data)

    def run(
        self, data: np.ndarray, grid_resolution: float, grid_origin: Pose
    ) -> tuple[np.ndarray]:
        # apply local maximum detection
        local_maxima_indices: np.ndarray = peak_local_max(
            data,
            min_distance=self.__local_max_radius,
            threshold_abs=self.__threshold,
            exclude_border=self.__exclude_border,
        )
        local_maxima: np.ndarray = np.zeros_like(data, dtype=bool)
        local_maxima[tuple(local_maxima_indices.T)] = True
        maxima_map: np.ndarray | int
        num_maxima: int
        maxima_map, num_maxima = ndimage.label(local_maxima)
        wheel_labels: List[int] = list(range(1, num_maxima + 1))

        if num_maxima > 0:
            # use nearest-neighbor interpolation and dilation for map refinement
            mask: np.ndarray = ~(maxima_map == 0)
            xy: np.ndarray = np.where(mask)
            interp: NearestNDInterpolator = NearestNDInterpolator(np.transpose(xy), maxima_map[xy])
            nn_map: np.ndarray = interp(*np.indices(maxima_map.shape))
            bin_dilated: np.ndarray = ndimage.binary_dilation(
                mask,
                structure=self.__dilation_footprint,
                iterations=1,
            ).astype(maxima_map.dtype)
            circle_map: np.ndarray = bin_dilated * nn_map

            # apply center-of-mass calculation
            centers: Iterable | Tuple = ndimage.center_of_mass(data, circle_map, wheel_labels)

            # calculate noise level
            noise_level: Iterable = ndimage.mean(data, circle_map, index=0)

            # calculate pressure sum over CCs (without noise level)
            pressure_sums: np.ndarray = ndimage.labeled_comprehension(
                data,
                circle_map,
                wheel_labels,
                lambda array: np.sum(array - noise_level),
                float,
                0.0,
            )
            # lambda a: np.sum(a - noise_level), float, 0.0)
            results: np.ndarray = np.append(
                np.array(centers), np.array([pressure_sums]).T, axis=1
            )
        else:
            results: np.ndarray = np.empty((0, 3))

        # correct, scale and rotate positions and assemble ROS message
        results += [0.5, 0.5, 0]
        results *= grid_resolution
        rotation = Rotation.from_quat(
            [
                grid_origin.orientation.x,
                grid_origin.orientation.y,
                grid_origin.orientation.z,
                grid_origin.orientation.w,
            ]
        )
        results = rotation.apply(results)
        results += np.array([[grid_origin.position.x, grid_origin.position.y, 0]])
        return (results,)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node: Node = WheelDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutdown node on user request.')
        return
    rclpy.shutdown()


if __name__ == '__main__':
    main()
