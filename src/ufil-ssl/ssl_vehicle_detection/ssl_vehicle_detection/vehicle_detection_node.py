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
from typing import List, Tuple

import numpy as np

import rclpy
from rclpy import Parameter
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.subscription import Subscription
from ssl_vehicle_tracking_msgs.msg import TrackArray
from std_msgs.msg import Header
from ufil_msgs.msg import Axle, Object, ObjectList

from .vehicle import Vehicle
from .vehicle_tracking import VehicleTracker


class VehicleTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('vehicle_detection_node')
        self.get_logger().info('Starting vehicle detection node')

        self.__last_n_vehicles: int = 0

        vehicle_template_strs: List[str] = ['']
        self.declare_parameter('vehicle_templates', vehicle_template_strs)
        self.get_logger().info(
            'Vehicle Templates: %s'
            % ' '.join(self.get_parameter('vehicle_templates').value)
        )
        vehicle_template_strs = self.get_parameter('vehicle_templates').value

        self.search_width: Parameter = self.declare_parameter(
            'search_width', 0.9
        )
        """Search area width or diameter. Denoted as 'e' in the thesis"""
        self.get_logger().info('Using detection epsilon: %f' % self.search_width.value)

        self.considered_rotation_pairs: Parameter = self.declare_parameter(
            'considered_rotation_pairs', 2
        )
        """Number of point pairs for rotation computation"""
        self.get_logger().info(
            'Considering rotational pairs: %i' % self.considered_rotation_pairs.value
        )

        self.dropping_threshold: Parameter = self.declare_parameter(
            'dropping_threshold', 0.05
        )
        """Threshold probability to drop vehicles or wheels that fall below"""
        self.get_logger().info(
            'Vehicle dropping threshold: %f' % self.dropping_threshold.value
        )

        self.publishing_threshold: Parameter = self.declare_parameter(
            'publishing_threshold', 0.0
        )
        """Threshold probability to publish only vehicles above this value"""
        self.get_logger().info(
            'Vehicle publishing threshold: %f' % self.publishing_threshold.value
        )

        self.history_size: Parameter = self.declare_parameter(
            'history_size', 10
        )
        """used to trim the history of incoming wheel tracks"""
        self.get_logger().info('Wheel history size: %f' % self.history_size.value)

        self.tracker: VehicleTracker = VehicleTracker(
            self.publishing_threshold.value,
            self.dropping_threshold.value,
            self.search_width.value,
            self.considered_rotation_pairs.value,
            vehicle_template_strs,
        )

        self.__subscription: Subscription = self.create_subscription(
            TrackArray,
            'wheel_tracks',
            self.receive,
            QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
        )
        self.__publisher: Publisher = self.create_publisher(ObjectList, 'vehicles', 10)

    def receive(self, data: TrackArray) -> None:
        unpacked_data = self.unpack_data(data)
        results: List[Vehicle] = self.run(*unpacked_data)

        packed_data: ObjectList = self.pack_data(results, data.header)

        veh_n = len(packed_data.objects)
        if veh_n > 0 or self.__last_n_vehicles > 0:
            self.__last_n_vehicles = veh_n
            self.__publisher.publish(packed_data)

    def unpack_data(self, data: TrackArray) -> Tuple[np.ndarray]:
        """Unpacks TrackArray data into a numpy array for vehicle detection"""
        if len(data.tracks) == 0:
            return (np.empty((0, 8)),)

        return (
            np.array(
                [
                    [
                        track.track_id,
                        (state_id - len(track.states) + 1),
                        state.x,
                        state.y,
                        state.vx,
                        state.vy,
                        track.frames_unseen,
                        state.covariance,
                    ]
                    for track in data.tracks
                    for state_id, state in enumerate(track.states)
                    if state_id >= (len(track.states) - self.history_size.value)
                ],
                dtype=object,
            ),
        )

    def pack_data(self, vehicles: List[Vehicle], header: Header) -> ObjectList:
        object_list: ObjectList = ObjectList()
        object_list.header = header
        for vehicle in vehicles:
            if vehicle.vehicle_existence_probability > self.tracker.publishing_threshold:
                vehicle_object: Object = Object()

                # ID
                vehicle_object.id = vehicle.id

                # State
                x: float
                yaw: float = vehicle.last_rot.as_euler('zyx', False)[0]
                x, y = vehicle.last_pos
                vehicle_object.state.state.x = x
                vehicle_object.state.state.y = y
                vehicle_object.state.state.v_x = vehicle.vx
                vehicle_object.state.state.v_y = vehicle.vy
                vehicle_object.state.state.a_x = 0.0  # Not measured
                vehicle_object.state.state.a_y = 0.0  # Not measured
                vehicle_object.state.state.yaw = yaw
                vehicle_object.state.state.yaw_rate = 0.0  # Not measured
                # State Covariance
                # TODO This is broken, matrix is not positive semi-definite
                # vehicle_object.state.covariance[0] = vehicle.covariance[0]  # X-X
                # vehicle_object.state.covariance[1] = vehicle.covariance[2]  # X-Y
                # vehicle_object.state.covariance[2] = vehicle.covariance[1]  # X-VX
                # vehicle_object.state.covariance[3] = vehicle.covariance[3]  # X-VY

                # vehicle_object.state.covariance[8] = vehicle.covariance[8]  # Y-X
                # vehicle_object.state.covariance[9] = vehicle.covariance[10]  # Y-Y
                # vehicle_object.state.covariance[10] = vehicle.covariance[9]  # Y-VX
                # vehicle_object.state.covariance[11] = vehicle.covariance[11]  # Y-VY

                # vehicle_object.state.covariance[16] = vehicle.covariance[4]  # VX-X
                # vehicle_object.state.covariance[17] = vehicle.covariance[6]  # VX-Y
                # vehicle_object.state.covariance[18] = vehicle.covariance[5]  # VX-VX
                # vehicle_object.state.covariance[19] = vehicle.covariance[7]  # VX-VY

                # vehicle_object.state.covariance[24] = vehicle.covariance[12]  # VY-X
                # vehicle_object.state.covariance[25] = vehicle.covariance[14]  # VY-Y
                # vehicle_object.state.covariance[26] = vehicle.covariance[13]  # VY-VX
                # vehicle_object.state.covariance[27] = vehicle.covariance[15]  # VY-VY

                vehicle_object.state.covariance[0] = 0.01
                vehicle_object.state.covariance[9] = 0.01
                vehicle_object.state.covariance[18] = 0.01
                vehicle_object.state.covariance[27] = 1e-9
                vehicle_object.state.covariance[36] = -1.0  # AX-AX (not measured)
                vehicle_object.state.covariance[45] = -1.0  # AY-AY (not measured)
                vehicle_object.state.covariance[54] = 0.01
                vehicle_object.state.covariance[63] = -1.0  # YawRate-YawRate (not measured)

                # Dimension
                vehicle_object.dimension.dimension.length = vehicle.template.object.length
                vehicle_object.dimension.dimension.width = vehicle.template.object.width

                # Dimension Covariance (very small, since Dim is known)
                vehicle_object.dimension.covariance[0] = 1.0
                vehicle_object.dimension.covariance[4] = 1.0
                vehicle_object.dimension.covariance[8] = -1

                # Existence Probability
                vehicle_object.existence_probability = vehicle.vehicle_existence_probability

                # Classification
                vehicle_object.classification.classification[
                    vehicle.template.object.classification
                ] = 1

                # # Features
                # vehicle_object.features.fl = False
                # vehicle_object.features.fr = False
                # vehicle_object.features.rl = False
                # vehicle_object.features.rr = False
                # vehicle_object.features.fm = False
                # vehicle_object.features.rm = False
                # vehicle_object.features.ml = False
                # vehicle_object.features.mr = False

                # Axle Geometry
                # vehicle_object.axles_is_present = True
                for axle in vehicle.template.object.axles:
                    axle_object: Axle = Axle()
                    axle_object.single_track = axle.single_track
                    axle_object.center_to_axle = axle.center_to_axle
                    axle_object.center_to_axle_variance = axle.center_to_axle_variance
                    axle_object.track_width = axle.track_width
                    axle_object.track_width_variance = axle.track_width_variance
                    axle_object.wheel_detected = axle.wheel_detected

                    vehicle_object.axles.append(axle_object)

                object_list.objects.append(vehicle_object)

        return object_list

    def run(self, track_data: np.ndarray) -> List[Vehicle]:
        """Executes the tracker with new wheel track data"""

        self.tracker.execute(track_data)
        return self.tracker.vehicles


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = VehicleTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutdown node on user request.')
        return
    rclpy.shutdown()


if __name__ == '__main__':
    main()
