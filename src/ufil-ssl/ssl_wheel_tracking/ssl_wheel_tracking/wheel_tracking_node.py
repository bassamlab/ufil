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
from datetime import datetime
from itertools import groupby
import logging
from typing import Dict, List, Set

from geometry_msgs.msg import PointStamped
import numpy as np
from numpy import ndarray
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.subscription import Subscription
from rclpy.time import Time
from rclpy.timer import Timer
import ssl_vehicle_tracking_msgs.msg
from ssl_vehicle_tracking_msgs.msg import TrackArray, TrackState, WheelArray
from std_msgs.msg import Header
from stonesoup.types.detection import Detection, MissedDetection
from stonesoup.types.prediction import Prediction
from stonesoup.types.state import GaussianState
from stonesoup.types.track import Track
from stonesoup.types.update import Update
from tf2_geometry_msgs import do_transform_point
import tf2_ros

from .wheel_tracker import WheelTracker


class WheelTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__('wheel_tracking_node')
        self.get_logger().info('Starting wheel tracking node')

        self.tracker: WheelTracker = WheelTracker()
        self.__last_tracks: int = 0
        self.__association_valid_limit: int = 1

        self.__input_topics: List[str] = ['wheel_detection']
        self.declare_parameter('input_topics', self.__input_topics)
        self.__input_topics: List[str] = self.get_parameter('input_topics').value
        self.get_logger().info(
            'Tracking wheels on topics: [%s]'
            % ' '.join(self.__input_topics).strip()
        )

        # --- TF2 Buffer and Listener ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Subscriptions ---
        self.__subscription: Dict[str, Subscription] = {}
        for input_topic in self.__input_topics:
            self.__subscription[input_topic] = self.create_subscription(
                WheelArray, input_topic, self.receive,
                QoSProfile(depth=10,
                           reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           durability=QoSDurabilityPolicy.VOLATILE))

        # --- Publisher ---
        self.publisher: Publisher = self.create_publisher(
            TrackArray,
            'wheel_tracks',
            QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            ),
        )

        # --- Timer ---
        self.timer: Timer = self.create_timer(1 / 100, self.timer_callback)
        self.detections: Set[Detection] = set()

    # -------------------------------------------------------------------------
    # Main periodic callback
    # -------------------------------------------------------------------------
    def timer_callback(self) -> None:
        sorted_detections: List[Detection] = sorted(
            self.detections, key=lambda detection: detection.timestamp
        )
        grouped_detections: List[Set[Detection]] = [
            set(grouped_detection)
            for _, grouped_detection in groupby(
                sorted_detections, lambda detection: detection.timestamp
            )
        ]

        if not grouped_detections:
            return

        # Run intermediate groups (for history)
        for group in grouped_detections[:-1]:
            if group:
                self.tracker.step(
                    next(iter(group)).timestamp,
                    group,
                    self.__association_valid_limit,
                )

        # Run current group
        data: Set[Detection] = grouped_detections[-1]
        current_time: Time = Time(nanoseconds=next(iter(data)).timestamp.timestamp() * 10**9)
        results: List[Track] = self.tracker.step(
            datetime.fromtimestamp(current_time.nanoseconds * 10**-9),
            data,
            self.__association_valid_limit,
        )

        self.detections.clear()

        tracks_n: int = len(results)
        if tracks_n > 0 or self.__last_tracks > 0:
            self.__last_tracks = tracks_n
            packed_data: TrackArray = self.pack_data(results, current_time)
            self.publisher.publish(packed_data)

    # -------------------------------------------------------------------------
    # Incoming detections
    # -------------------------------------------------------------------------
    def receive(self, data: WheelArray) -> None:
        unpacked_data: Set[Detection] = self.unpack_and_transform_data(data)
        self.detections.update(unpacked_data)

    # -------------------------------------------------------------------------
    # Transform detections to map frame
    # -------------------------------------------------------------------------
    def unpack_and_transform_data(self, wheel_array: WheelArray) -> Set[Detection]:
        """Transforms wheel detections from their source frame into the 'map' frame."""
        header_time: datetime = datetime.fromtimestamp(
            Time.from_msg(wheel_array.header.stamp).nanoseconds * 10**-9)

        transformed_detections: Set[Detection] = set()

        for p in wheel_array.wheels:
            point_in = PointStamped()
            point_in.header = wheel_array.header
            point_in.point.x = p.x
            point_in.point.y = p.y
            point_in.point.z = 0.0

            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame='map',
                    source_frame=wheel_array.header.frame_id,
                    time=Time.from_msg(wheel_array.header.stamp),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                point_out = do_transform_point(point_in, transform)
                transformed_detections.add(
                    Detection(
                        [point_out.point.x, point_out.point.y],
                        metadata={'z': p.intensity},
                        timestamp=header_time,
                    )
                )
            except Exception as e:
                self.get_logger().warn(
                    f"TF transform from {wheel_array.header.frame_id} to 'map' failed: {e}"
                )

        return transformed_detections

    # -------------------------------------------------------------------------
    # Track → message conversion
    # -------------------------------------------------------------------------
    def pack_data(self, tracks: List[Track], timestamp: Time) -> TrackArray:
        track_array: TrackArray = TrackArray(
            header=Header(frame_id='map', stamp=timestamp.to_msg()),
            tracks=[
                ssl_vehicle_tracking_msgs.msg.Track(
                    track_id=t.id,
                    states=[
                        TrackState(
                            x=state.mean[0],
                            vx=state.mean[1],
                            y=state.mean[2],
                            vy=state.mean[3],
                            covariance=self.get_covar(state),
                            state=self.get_state(state),
                        )
                        for state in [t.states[i] for i in self.good_states(t)]
                    ],
                    frames_unseen=self.count_missing_detections(t),
                )
                for t in tracks
            ],
        )
        return track_array

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    @staticmethod
    def good_states(track: Track) -> reversed:
        indices = []
        use_all = True
        for i in range(len(track.states) - 1, 0, -1):
            state = track.states[i]
            if isinstance(state, Update) and not isinstance(
                state.hypothesis.measurement, MissedDetection
            ):
                indices.append(i)
                use_all = False
            if use_all:
                indices.append(i)
        return reversed(indices)

    @staticmethod
    def count_missing_detections(track: Track) -> int:
        count: int = 0
        for state in reversed(track.states):
            if isinstance(state, Update):
                if isinstance(state.hypothesis.measurement, MissedDetection):
                    count += 1
                else:
                    break
            elif isinstance(state, Prediction):
                count += 1
            else:
                logging.error('Unexpected state')
        return count

    @staticmethod
    def get_state(state: GaussianState) -> str:
        if isinstance(state, Update):
            return 'matched'
        if isinstance(state, Prediction):
            return 'predicted'
        return 'unknown'

    @staticmethod
    def get_covar(state: GaussianState) -> ndarray:
        covar: np.ndarray = np.identity(4)
        covar[0:2, 0:2] = state.covar[0:2, 0:2]
        covar[2:4, 2:4] = state.covar[2:4, 2:4]
        return covar.flatten()


def main(args=None):
    rclpy.init(args=args)
    try:
        node: Node = WheelTrackingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutdown node on user request.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
