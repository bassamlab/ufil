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

"""
Conversion of deserialized ROS 2 messages into HDF5 records.

Each converter receives the message, its topic and the bag receive time in ns, and returns the
records to append. Topic names start with '/', so ``'imu' + topic`` yields ``imu/<topic>``.
"""

from typing import Any, Callable, Dict, List, Optional

import numpy as np

from . import hdf_layout
from .hdf_layout import UNKNOWN_VARIANCE
from .hdf_writer import GridFrame, Record, Row, Transform

Converter = Callable[[Any, str, int], List[Record]]


def stamp_to_ns(stamp: Any) -> float:
    return stamp.sec * 1e9 + stamp.nanosec


def _xyz(vector: Any) -> tuple:
    return vector.x, vector.y, vector.z


def _xyzw(quaternion: Any) -> tuple:
    return quaternion.x, quaternion.y, quaternion.z, quaternion.w


def convert_imu(msg: Any, topic: str, receive_time: int) -> List[Record]:
    return [Row('imu' + topic, hdf_layout.IMU, {
        'orientation': _xyzw(msg.orientation),
        'angular_velocity': _xyz(msg.angular_velocity),
        'linear_acceleration': _xyz(msg.linear_acceleration),
        'message_time': receive_time,
    })]


def convert_magnetic_field(msg: Any, topic: str, receive_time: int) -> List[Record]:
    return [Row('mag' + topic, hdf_layout.MAGNETIC_FIELD, {
        'magnetic_field': _xyz(msg.magnetic_field),
        'message_time': receive_time,
    })]


def convert_navsatfix(msg: Any, topic: str, receive_time: int) -> List[Record]:
    return [Row('navsatfix' + topic, hdf_layout.NAVSATFIX, {
        'gnss_time': (msg.header.stamp.sec, msg.header.stamp.nanosec),
        'status': (msg.status.status, msg.status.service),
        'position': (msg.latitude, msg.longitude, msg.altitude),
        'position_covariance': msg.position_covariance,
        'position_covariance_type': msg.position_covariance_type,
        'message_time': receive_time,
    })]


def convert_odometry(msg: Any, topic: str, receive_time: int) -> List[Record]:
    return [Row('odometry' + topic, hdf_layout.ODOMETRY, {
        'stamp': (msg.header.stamp.sec, msg.header.stamp.nanosec),
        'pose_position': _xyz(msg.pose.pose.position),
        'pose_orientation': _xyzw(msg.pose.pose.orientation),
        'pose_covariance': msg.pose.covariance,
        'twist_linear': _xyz(msg.twist.twist.linear),
        'twist_angular': _xyz(msg.twist.twist.angular),
        'twist_covariance': msg.twist.covariance,
        'message_time': receive_time,
    })]


def convert_pointcloud2(msg: Any, topic: str, receive_time: int) -> List[Record]:
    return [Row('pointcloud' + topic, hdf_layout.POINTCLOUD, {
        'message_time': receive_time,
        'header_time': stamp_to_ns(msg.header.stamp),
        'number_of_points': msg.height * msg.width,
    })]


def _object_row(group: str, obj: Any, receive_time: int, header_time: float) -> Row:
    state = obj.state.state
    # The state covariance is a row major 8x8 matrix of [x, y, v_x, v_y, a_x, a_y, yaw, yaw_rate].
    variance = obj.state.covariance[::9]
    dimension = obj.dimension.dimension
    return Row(group, hdf_layout.OBJECT, {
        'message_time': receive_time,
        'header_time': header_time,
        'existence_probability': obj.existence_probability,
        'state/position': (state.x, state.y, 0.0),
        'state/position_variance': (variance[0], variance[1], UNKNOWN_VARIANCE),
        'state/linear_velocity': (state.v_x, state.v_y, 0.0),
        'state/linear_velocity_variance': (variance[2], variance[3], UNKNOWN_VARIANCE),
        'state/linear_acceleration': (state.a_x, state.a_y, 0.0),
        'state/linear_acceleration_variance': (variance[4], variance[5], UNKNOWN_VARIANCE),
        'state/orientation': (0.0, 0.0, state.yaw),
        'state/orientation_variance': (UNKNOWN_VARIANCE, UNKNOWN_VARIANCE, variance[6]),
        'state/angular_velocity': (0.0, 0.0, state.yaw_rate),
        'state/angular_velocity_variance': (UNKNOWN_VARIANCE, UNKNOWN_VARIANCE, variance[7]),
        'dimension/dimension': (dimension.length, dimension.width, dimension.height),
        'dimension/dimension_variance': tuple(obj.dimension.covariance[::4]),
        'classification': obj.classification.classification,
    })


def convert_object_list(msg: Any, topic: str, receive_time: int) -> List[Record]:
    group = 'objects' + topic
    header_time = stamp_to_ns(msg.header.stamp)
    records: List[Record] = [Row(group, hdf_layout.OBJECT_LIST, {
        'message_time': receive_time,
        'header_time': header_time,
        'number_of_objects': len(msg.objects),
    })]
    records.extend(_object_row(f'{group}/{obj.id}', obj, receive_time, header_time)
                   for obj in msg.objects)
    return records


def convert_occupancy_grid(msg: Any, topic: str, receive_time: int) -> List[Record]:
    info = msg.info
    cells = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
    return [GridFrame('occgrid' + topic, float(receive_time), info.resolution,
                      (info.origin.position.x, info.origin.position.y), msg.header.frame_id,
                      cells)]


def convert_tf(msg: Any, topic: str, receive_time: int) -> List[Record]:
    return [Transform(t.header.frame_id.strip(), t.child_frame_id.strip(),
                      stamp_to_ns(t.header.stamp) * 1e-9, _xyz(t.transform.translation),
                      _xyzw(t.transform.rotation))
            for t in msg.transforms]


CONVERTERS: Dict[str, Converter] = {
    'sensor_msgs/msg/Imu': convert_imu,
    'sensor_msgs/msg/MagneticField': convert_magnetic_field,
    'sensor_msgs/msg/NavSatFix': convert_navsatfix,
    'sensor_msgs/msg/PointCloud2': convert_pointcloud2,
    'nav_msgs/msg/Odometry': convert_odometry,
    'nav_msgs/msg/OccupancyGrid': convert_occupancy_grid,
    'tf2_msgs/msg/TFMessage': convert_tf,
    'ufil_msgs/msg/ObjectList': convert_object_list,
}


def to_records(msg: Any, topic: str, topic_type: str,
               receive_time: int) -> Optional[List[Record]]:
    """Return the records of a message, or None if the message type is not supported."""
    converter = CONVERTERS.get(topic_type)
    return None if converter is None else converter(msg, topic, receive_time)
