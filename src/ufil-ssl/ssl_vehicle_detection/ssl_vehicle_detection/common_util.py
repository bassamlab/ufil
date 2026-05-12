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
from typing import List

import numpy as np
from numpy import ndarray

from .vehicle import Vehicle


class COLUMNS:
    """Constant class containing the column indices of the wheel track array. This is advantageous over a pandas data
    frame regarding data access speed"""
    TRACK_ID = 0
    TIME = 1
    X = 2
    Y = 3
    VX = 4
    VY = 5
    FRAMES_UNSEEN = 6
    COVARIANCE = 7


def wheel_map_from_track_array(data: ndarray, vehicle: Vehicle) -> ndarray:
    """For each row in data, finds the corresponding wheel index of the track in a vehicle object. Data must only
    contain tracks assigned to the vehicle"""
    return np.array([vehicle.wheel_tracks.index(t) for t in data[:, COLUMNS.TRACK_ID]])


def get_columns_from_track_array(data: ndarray, columns: List[int], astype=None) -> ndarray:
    """Column selection function for a track array. Only the requested columns are returned. Optionally, the type can be
    converted to a target type"""
    result = data[:, columns].reshape(-1, len(columns))
    if astype is not None:
        result = result.astype(astype)
    return result
