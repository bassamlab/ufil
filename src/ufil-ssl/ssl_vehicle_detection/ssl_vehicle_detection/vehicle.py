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
import json
from typing import Dict, List, Optional, Tuple

import numpy as np
from numpy import ndarray
from scipy.spatial.transform import Rotation


class Axle:
    def __init__(
        self,
        single_track: bool,
        track_width: float,
        track_width_variance: float,
        center_to_axle: float,
        center_to_axle_variance: float,
        wheel_detected: List[bool],
    ):
        self.single_track = single_track
        self.track_width = track_width
        self.track_width_variance = track_width_variance
        self.center_to_axle = center_to_axle
        self.center_to_axle_variance = center_to_axle_variance
        self.wheel_detected = wheel_detected


class Object:
    def __init__(self, length: float, width: float, classification: float, axles: List[Axle]):
        self.length = length
        self.width = width
        self.classification = classification
        self.axles = axles


class VehicleTemplate:
    """Template class for vehicles. Instances of this class define detectable vehicle shapes."""

    def __init__(self, json_str: str) -> None:
        self.object: Object = self.parse_json(json_str)
        self.points: np.ndarray = np.array(self.calc_points())

        """Precomputed pairwise distances of template points"""
        self.distances, self.arg_pairs_by_distance = self.calc_distances()

        """Precomputed angles between template points"""
        self.angles: ndarray = self.calc_angles()

    def axle_from_dict(self, data: Dict) -> Axle:
        return Axle(
            single_track=data['single_track'],
            track_width=data['track_width'],
            track_width_variance=data['track_width_variance'],
            center_to_axle=data['center_to_axle'],
            center_to_axle_variance=data['center_to_axle_variance'],
            wheel_detected=data['wheel_detected']
        )

    def object_from_dict(self, data: Dict) -> Object:
        axles = [self.axle_from_dict(axle_data) for axle_data in data['axles']]
        return Object(
            length=data['length'],
            width=data['width'],
            classification=data['classification'],
            axles=axles
        )

    def parse_json(self, json_str: str) -> Object:
        # Open the file and read its contents
        with open(json_str, 'r') as file:
            json_data = file.read()

            # Parse the JSON data
            try:
                data = json.loads(json_data)
                return self.object_from_dict(data)
            except json.JSONDecodeError as e:
                print(f'JSONDecodeError: {e}')

    def calc_points(self) -> List[Tuple[float, float]]:
        points: List[Tuple[float, float]] = []
        for axle in self.object.axles:
            if axle.single_track:
                points.append((0 + axle.center_to_axle, 0))
            else:
                points.append((0 + axle.center_to_axle, 0 - axle.track_width / 2))  # Left
                points.append((0 + axle.center_to_axle, 0 + axle.track_width / 2))  # Right

        return points

    # Pre-computation function of pairwise wheel distances
    def calc_distances(self) -> Tuple[ndarray, ndarray]:
        """Pre-computation function of pairwise wheel distances"""
        distance_matrix = np.linalg.norm(self.points[:, None] - self.points, axis=-1)
        point_pairs = np.unravel_index(
            np.argsort(distance_matrix, axis=None)[::-1], distance_matrix.shape
        )

        # remove duplicate pairs
        point_pairs_unique = np.empty((0, 2), dtype='int64')
        for a, b in np.transpose(point_pairs):
            if a != b and not any(np.equal(point_pairs_unique, [b, a]).all(1)):
                point_pairs_unique = np.vstack((point_pairs_unique, [[a, b]]))

        return distance_matrix, point_pairs_unique

    # Pre-computation function of pairwise angles between points
    def calc_angles(self) -> ndarray:
        """Pre-computation function of pairwise angles between points"""
        point_diff = -self.points[:, None] + self.points
        angles = np.arctan2(point_diff[..., 1], point_diff[..., 0])
        return angles


# Class of vehicle objects containing all state information
class Vehicle:
    """Class of vehicle objects containing all state information"""
    new_id = itertools.count().__next__

    def __init__(self, template: VehicleTemplate, wheel_tracks: List[str]) -> None:
        super().__init__()
        self.template: VehicleTemplate = template
        """The template the vehicle bases on"""
        self.wheel_tracks: List[Optional[str]] = wheel_tracks
        """List of assigned wheel tracks. If unassigned, the entry is `None`.

        The ordering accords to the template
        """
        self.wheel_last_seen: List[int] = [0 for _ in range(len(template.points))]
        """negative value denoting the amount of frames with no track present for a wheel"""
        self.last_pos: ndarray = np.zeros(2)
        """latest computed value of vehicle position"""
        self.last_rot: Rotation = Rotation.identity()
        """latest computed value of vehicle rotation"""
        self.last_offset: ndarray = np.zeros(2)
        """latest template offset of vehicle to origin"""
        self.wheel_assignment_probabilities: ndarray = np.ones(len(template.points), dtype=float)
        """latest computed values of correct wheel assignments"""
        self.vehicle_existence_probability: float = 1.0
        """latest computed value of vehicle existence probability"""
        self.id = Vehicle.new_id()
        """vehicle speed"""
        self.vx: float = 0.0
        self.vy: float = 0.0
        self.speed: float = 0.0
        """vehicle covariance"""
        self.covariance: ndarray = np.ones(16) * 100
