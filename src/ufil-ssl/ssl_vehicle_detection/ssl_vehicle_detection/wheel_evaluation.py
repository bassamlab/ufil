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
from typing import Callable, List

import numpy as np
from numpy import ndarray

from .common_util import COLUMNS, get_columns_from_track_array, wheel_map_from_track_array
from .vehicle import Vehicle
from .vehicle_detection import apply_transformation, compute_rotation_and_offset


def evaluate_track_existence(vehicle: Vehicle, track_data: ndarray, wheel_existence_prob_model: Callable,
                             **kwargs) -> List[float]:
    """Evaluates the probability of track existence. The absolute metric yields a negative number. 0 indicates fresh
    track data, -3 is a track missing since 3 frames.
    :returns list of wheel track existence probabilities"""
    vehicle_tracks: ndarray = track_data[np.isin(track_data[:, COLUMNS.TRACK_ID], vehicle.wheel_tracks) &
                                         (track_data[:, COLUMNS.TIME] == 0)]

    existence_probabilities: List[float] = []
    for idx, track_id in enumerate(vehicle.wheel_tracks):
        # get wheel age
        current_track_data: ndarray = vehicle_tracks[
            vehicle_tracks[:, COLUMNS.TRACK_ID] == track_id
        ]
        if len(current_track_data) > 0:
            wheel_age = -int(current_track_data[:, COLUMNS.FRAMES_UNSEEN])
        else:
            wheel_age = vehicle.wheel_last_seen[idx]

        # use model to calculate probabilities
        wheel_prob = wheel_existence_prob_model(wheel_age)
        existence_probabilities.append(wheel_prob)

    return existence_probabilities


def evaluate_vehicle_wheel_positions(vehicle: Vehicle, track_data: ndarray, search_width: float,
                                     distance_fnc: Callable, considered_rotation_pairs: int,
                                     wheel_correct_position_prob_model: Callable,
                                     **kwargs) -> List[float]:
    """Evaluates the wheel positions in relation to expected vehicle position. The metric yields deviations of wheel to
    its expected position. The value is scaled by the acceptable error `search_width`
    :returns list of probabilities respecting correct track-to-vehicle assignments"""
    vehicle_tracks: ndarray = track_data[np.isin(track_data[:, COLUMNS.TRACK_ID], vehicle.wheel_tracks) &
                                         (track_data[:, COLUMNS.TIME] == 0)]

    # delete vehicle with less than two wheels
    if len(vehicle_tracks) < 2:
        return [0.0 for _ in vehicle.wheel_tracks]

    # compute vehicle position and rotation
    wheel_map = wheel_map_from_track_array(vehicle_tracks, vehicle)
    rotation, offset = compute_rotation_and_offset(
        get_columns_from_track_array(
            vehicle_tracks, [COLUMNS.X, COLUMNS.Y], astype=float
        ),
        wheel_map,
        vehicle.template,
        considered_rotation_pairs,
    )

    # compute target position and deviation of each wheel
    target_positions = apply_transformation(
        vehicle.template.points, rotation, offset=offset
    )
    deviations = np.abs(
        distance_fnc(
            target_positions[wheel_map]
            - get_columns_from_track_array(
                vehicle_tracks,
                [COLUMNS.X, COLUMNS.Y],
                astype=float,
            ),
            axis=1,
        )
    )

    # scale deviations to input error and calculate probability
    deviation_dimension = deviations / search_width

    # use model to compute probabilities
    probabilities: List[float] = []
    for i in range(len(vehicle.wheel_tracks)):
        if i in wheel_map:
            value = deviation_dimension[wheel_map == i][0]
            probability = wheel_correct_position_prob_model(value)
            probabilities.append(probability)
        else:
            # unassigned wheels are not evaluated
            probabilities.append(np.nan)

    return probabilities
