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
Layout of the time series groups written to the HDF5 file.

Every group holds one dataset per signal with one row per message. The attributes follow the
SFB data handbook (name, description, dim, axis, unit, quantity).
"""

from dataclasses import dataclass
from typing import Any, Dict, Tuple

import numpy as np


@dataclass(frozen=True)
class DatasetLayout:
    """Shape of one row, element type and handbook attributes of a dataset."""

    shape: Tuple[int, ...]
    dtype: Any
    attrs: Dict[str, str]


@dataclass(frozen=True)
class GroupLayout:
    """Handbook ``type``/``description`` attributes and datasets of a time series group."""

    group_type: str
    description: str
    datasets: Dict[str, DatasetLayout]


def _dataset(shape, dtype, name, description, dim, axis, unit, quantity) -> DatasetLayout:
    return DatasetLayout(shape, dtype, {
        'name': name, 'description': description, 'dim': dim, 'axis': axis, 'unit': unit,
        'quantity': quantity})


def _message_time() -> DatasetLayout:
    return _dataset((1,), np.float64, 'message reception time',
                    'time stamp in ns at which the message was recorded in the bag',
                    '[message time]', 'x', 'ns', '-')


def _header_time() -> DatasetLayout:
    return _dataset((1,), np.float64, 'message header time',
                    'time stamp in ns of the message header', '[header time]', 'x', 'ns', '-')


def _stamp(name: str, description: str) -> DatasetLayout:
    return _dataset((2,), np.int64, name, description, '[seconds, nanoseconds]', 'x',
                    '[s, ns]', '-')


def _vector(name, description, dim, unit, quantity, size=3) -> DatasetLayout:
    return _dataset((size,), np.float64, name, description, dim, 'y', unit, quantity)


IMU = GroupLayout(
    'experiment',
    'data recorded from an Inertial Measurement Unit (IMU) mounted to some form of vehicle', {
        'orientation': _vector(
            'orientation', 'orientation of the sensor as quaternion',
            '[x quaternion, y quaternion, z quaternion, w quaternion]', 'quaternion units',
            'orientation', size=4),
        'angular_velocity': _vector(
            'angular velocity', 'angular velocity of the sensor',
            '[x angular velocity, y angular velocity, z angular velocity]', 'rad/s',
            'angular velocity'),
        'linear_acceleration': _vector(
            'acceleration', 'linear acceleration given at point in time',
            '[x acceleration, y acceleration, z acceleration]', 'm/s^2', 'acceleration'),
        'message_time': _message_time(),
    })

MAGNETIC_FIELD = GroupLayout(
    'experiment',
    'data recorded from a magnetometer mounted to some form of vehicle', {
        'magnetic_field': _vector(
            'magnetic field', 'magnetic field strength at point in time',
            '[x magnetic field strength, y magnetic field strength, z magnetic field strength]',
            'Tesla', 'magnetic field strength'),
        'message_time': _message_time(),
    })

NAVSATFIX = GroupLayout(
    'experiment',
    'data recorded from a Global Navigation Satellite System (GNSS) receiver mounted to some '
    'form of vehicle', {
        'gnss_time': _stamp('gnss time stamp', 'time stamp provided by the gnss receiver'),
        'status': _dataset((2,), np.int64, 'status of gnss',
                           'fix status and used satellite systems (service) of the receiver',
                           '[status, service]', 'y', '[enum, bitmask]', '-'),
        'position': _dataset((3,), np.float64, 'position', 'position of gnss receiver',
                             '[latitude, longitude, altitude]', 'x, y', '[deg, deg, m]',
                             'position'),
        'position_covariance': _vector(
            'covariance of position', 'covariance of position measurements',
            'row major covariance matrix', 'm^2', 'covariance', size=9),
        'position_covariance_type': _dataset(
            (1,), np.uint8, 'type of position covariance',
            'indicates how the position covariance was obtained (unknown, approximated, '
            'diagonal known, known)', '[type]', 'y', '[enum]', '-'),
        'message_time': _message_time(),
    })

ODOMETRY = GroupLayout(
    'experiment', 'odometry estimate of a vehicle', {
        'stamp': _stamp('header time stamp', 'time stamp of the odometry estimate'),
        'pose_position': _vector('position', 'estimated position',
                                 '[x position, y position, z position]', 'm', 'position'),
        'pose_orientation': _vector(
            'orientation', 'estimated orientation as quaternion',
            '[x quaternion, y quaternion, z quaternion, w quaternion]', 'quaternion units',
            'orientation', size=4),
        'pose_covariance': _vector(
            'pose covariance', 'covariance of the estimated pose',
            'row major 6x6 covariance matrix of [x, y, z, roll, pitch, yaw]', '-',
            'covariance', size=36),
        'twist_linear': _vector('velocity', 'estimated linear velocity',
                                '[x velocity, y velocity, z velocity]', 'm/s', 'velocity'),
        'twist_angular': _vector(
            'angular velocity', 'estimated angular velocity',
            '[x angular velocity, y angular velocity, z angular velocity]', 'rad/s',
            'angular velocity'),
        'twist_covariance': _vector(
            'twist covariance', 'covariance of the estimated twist',
            'row major 6x6 covariance matrix of [v_x, v_y, v_z, w_x, w_y, w_z]', '-',
            'covariance', size=36),
        'message_time': _message_time(),
    })

POINTCLOUD = GroupLayout(
    'experiment', 'statistics of point clouds recorded by a sensor', {
        'message_time': _message_time(),
        'header_time': _header_time(),
        'number_of_points': _dataset((1,), np.int64, 'number of points',
                                     'number of points in cloud', '[number of points]', 'y',
                                     'amount', '-'),
    })

OBJECT_LIST = GroupLayout(
    'simulation', 'data from an object state estimation algorithm', {
        'message_time': _message_time(),
        'header_time': _header_time(),
        'number_of_objects': _dataset((1,), np.int64, 'number of objects',
                                      'number of objects in list', '[number of objects]', 'y',
                                      'amount', '-'),
    })

OBJECT = GroupLayout(
    'simulation', 'data from an object state estimation algorithm', {
        'message_time': _message_time(),
        'header_time': _header_time(),
        'existence_probability': _dataset(
            (1,), np.float64, 'object existence probability',
            'existence probability given at point in time', '[existence probability]', 'y',
            'percent', 'probability'),
        'state/position': _vector('position', 'position given at point in time',
                                  '[x position, y position, z position]', 'm', 'position'),
        'state/position_variance': _vector(
            'position_variance', 'position variance given at point in time',
            '[x position variance, y position variance, z position variance]', 'm^2',
            'variance'),
        'state/linear_velocity': _vector('velocity', 'linear velocity given at point in time',
                                         '[x velocity, y velocity, z velocity]', 'm/s',
                                         'velocity'),
        'state/linear_velocity_variance': _vector(
            'velocity_variance', 'linear velocity variance given at point in time',
            '[x velocity variance, y velocity variance, z velocity variance]', 'm^2/s^2',
            'variance'),
        'state/linear_acceleration': _vector(
            'acceleration', 'linear acceleration given at point in time',
            '[x acceleration, y acceleration, z acceleration]', 'm/s^2', 'acceleration'),
        'state/linear_acceleration_variance': _vector(
            'acceleration_variance', 'linear acceleration variance given at point in time',
            '[x acceleration variance, y acceleration variance, z acceleration variance]',
            'm^2/s^4', 'variance'),
        'state/orientation': _vector('orientation', 'orientation in euler angles',
                                     '[roll, pitch, yaw]', 'rad', 'orientation'),
        'state/orientation_variance': _vector(
            'orientation_variance', 'orientation variance given at point in time',
            '[roll variance, pitch variance, yaw variance]', 'rad^2', 'variance'),
        'state/angular_velocity': _vector(
            'angular velocity', 'angular velocity',
            '[x angular velocity, y angular velocity, z angular velocity]', 'rad/s',
            'angular velocity'),
        'state/angular_velocity_variance': _vector(
            'angular_velocity_variance', 'angular velocity variance given at point in time',
            '[x angular velocity variance, y angular velocity variance, '
            'z angular velocity variance]', 'rad^2/s^2', 'variance'),
        'dimension/dimension': _vector('dimension', 'dimension given at point in time',
                                       '[length, width, height]', 'm', 'dimension'),
        'dimension/dimension_variance': _vector(
            'dimension_variance', 'dimension variance given at point in time',
            '[length variance, width variance, height variance]', 'm^2', 'variance'),
        'classification': _vector(
            'classification', 'classification vector given at point in time',
            '[car, truck, motorcycle, bicycle, pedestrian, stationary, other]', 'percent',
            'confidence', size=7),
    })

UNKNOWN_VARIANCE = -1.0
"""Placeholder for variances the source message does not provide."""
