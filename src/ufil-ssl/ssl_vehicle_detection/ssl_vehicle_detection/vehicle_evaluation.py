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
from typing import Callable

import numpy as np
from numpy import ndarray

from .common_util import COLUMNS
from .vehicle import Vehicle


def evaluate_vehicle_tracks_missing(vehicle: Vehicle, track_data: ndarray, vehicle_tracks_missing_model: Callable,
                                    **kwargs) -> float:
    """Evaluates the number of missing wheel tracks. The absolute metric yields a positive number of missing tracks
    :returns vehicle probability respecting wheel assignments"""
    vehicle_tracks: ndarray = track_data[np.isin(track_data[:, COLUMNS.TRACK_ID], vehicle.wheel_tracks) &
                                         (track_data[:, COLUMNS.TIME] == 0)]
    tracks_missing: int = len(vehicle.template.points) - len(vehicle_tracks)
    return vehicle_tracks_missing_model(tracks_missing)


def evaluate_vehicle_wheel_track_probabilities(vehicle: Vehicle, **kwargs) -> float:
    """Evaluates the assigned wheel track probabilities. The average probability yields vehicle probability
    :returns vehicle probability respecting wheel track probabilities"""
    return float(np.nanmean(vehicle.wheel_assignment_probabilities))
