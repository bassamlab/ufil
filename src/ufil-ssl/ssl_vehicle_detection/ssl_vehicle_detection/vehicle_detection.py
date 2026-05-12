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
from typing import Callable, List, Optional, Tuple

import numpy as np
from numpy import ndarray, nonzero, transpose
from scipy.spatial.transform import Rotation

from .common_util import COLUMNS, get_columns_from_track_array
from .vehicle import Vehicle, VehicleTemplate

"""This file contains a collection of functions for detection of new vehicles. However, some of the functions are also
used for other operations, like vehicle reassembly."""


def find_new_vehicle_candidates(unassigned_tracks: ndarray, search_width: float, distance_fnc: Callable,
                                max_pairs: int, vehicle_templates: List[VehicleTemplate]) -> List[Vehicle]:
    """Initiates the task of detecting vehicle candidates from unassigned wheel track data. Thereby, it tries all
    available vehicle templates and returns resulting candidates."""
    distance_matrix: ndarray = compute_distance_matrix(
        unassigned_tracks, distance_fnc)

    vehicles: List[Vehicle] = []
    for template in vehicle_templates:
        vehicle_candidates: List[Vehicle] = try_fit_template(
            template, unassigned_tracks, distance_matrix, search_width,
            distance_fnc, max_pairs)
        if len(vehicle_candidates) > 0:
            vehicles += vehicle_candidates

    return vehicles


def compute_distance_matrix(data: ndarray, distance_fnc: Callable) -> ndarray:
    """Computes pairwise distances between of a list of wheel tracks. `n` points result in a `n x n` matrix where row
    and column indices preserve the row index of input data"""
    points = get_columns_from_track_array(
        data, [COLUMNS.X, COLUMNS.Y], astype=float)

    distance_matrix = distance_fnc(points[:, None] - points, axis=-1)
    return distance_matrix


def try_fit_template(template: VehicleTemplate, points: ndarray, distance_matrix: ndarray, search_width: float,
                     distance_fnc: Callable, max_pairs: int) -> List[Vehicle]:
    """Tries to fit template on set of points. The pairwise distance matrix is provided to avoid redundant computation.
    First, it detects candidate point pairs with matching target distance. Second, it attempts a vehicle assembly with
    each candidate.
        :returns list of vehicles matching the template"""
    if len(points) < len(template.points):
        return []

    # find possible first match candidates
    candidate_template_index = np.unravel_index(
        np.argmax(template.distances), template.distances.shape
    )
    target_distance = template.distances[candidate_template_index]

    dist_in_range_filter = np.logical_and(
        target_distance - search_width < distance_matrix,
        distance_matrix < target_distance + search_width)
    candidate_point_indices: ndarray = transpose(nonzero(dist_in_range_filter))

    # try every candidate point as (0, 0) of template
    vehicles: List[Vehicle] = []
    for p_idx, q_idx in candidate_point_indices:
        found_vehicle = assemble_candidates(
            p_idx, q_idx, int(candidate_template_index[0]),
            int(candidate_template_index[1]), points, template, search_width,
            distance_fnc, max_pairs)
        if found_vehicle is not None:
            vehicles.append(found_vehicle)

    return vehicles


def assemble_candidates(p_idx: int, q_idx: int, template_p_idx: int, template_q_idx: int, points: ndarray,
                        template: VehicleTemplate, search_width: float, distance_fnc: Callable, max_pairs: int) \
        -> Optional[Vehicle]:
    """Assembles vehicle candidates with a matching initial point pair. This function arranges a wheel map as argument
    for the find_remaining_tracks function which performs the geometric matching.
    The first two parameters are the indices of the two initial points. The two subsequent parameters represent the
    corresponding template point indices."""
    found_tracks: List[str] = [
        points[p_idx, COLUMNS.TRACK_ID], points[q_idx, COLUMNS.TRACK_ID]]
    found_template_point_map: ndarray = np.array(
        [template_p_idx, template_q_idx])

    _, found_tracks = find_remaining_tracks(found_template_point_map, found_tracks,
                                            template, points,
                                            search_width, distance_fnc, max_pairs)
    vehicle = Vehicle(template, found_tracks)
    return vehicle


def find_remaining_tracks(known_template_point_map: ndarray, known_tracks: List[str], template: VehicleTemplate,
                          available_tracks: ndarray, search_width: float, distance_fnc: Callable, max_pairs: int) \
        -> Tuple[int, List[str]]:
    """Performs the action of template fitting and searching of remaining template points in measurement data. The
    `known_template_point_map` parameter must have the same size as `known_tracks`. For each entry of `known_tracks`,
    it indicates the corresponding index of the template. `available_tracks` further must be a superset of
    `known_tracks`.
    :returns number of found tracks; found tracks, including known_tracks"""
    assert len(known_template_point_map) >= 2
    assert len(known_template_point_map) == len(known_tracks)

    # set up manipulatable lists for the following loop
    found_template_point_map: ndarray = known_template_point_map
    found_tracks: List[Optional[str]] = list(known_tracks)
    remaining_template_points_to_find: ndarray = \
        np.setdiff1d(np.arange(len(template.points)), known_template_point_map)

    # try to match all points and adjust template accordingly
    while len(remaining_template_points_to_find) > 0:
        # 1. adjust template
        found_track_points: ndarray = get_points_of_tracks(
            found_tracks, available_tracks)
        wheel_map: ndarray = found_template_point_map[[
            i for i, v in enumerate(found_tracks) if v is not None]]
        template_rotation, template_offset = compute_rotation_and_offset(found_track_points, wheel_map,
                                                                         template, max_pairs)

        # 2. find next point
        next_template_point: int = remaining_template_points_to_find[0:1]
        remaining_template_points_to_find: ndarray = remaining_template_points_to_find[1:]
        next_point: ndarray = apply_transformation(template.points[next_template_point], template_rotation,
                                                   offset=template_offset)
        found, found_track_id = try_find_point(
            available_tracks[~np.isin(
                available_tracks[:, COLUMNS.TRACK_ID], found_tracks)],
            next_point, search_width, distance_fnc)
        if found:
            found_tracks.append(found_track_id)
        else:
            found_tracks.append(None)
        found_template_point_map = np.append(
            found_template_point_map, next_template_point, axis=0)

    found_track_count: int = sum(1 for _ in filter(
        None.__ne__, found_tracks)) - len(known_tracks)
    result_tracks = np.array(found_tracks)[np.argsort(found_template_point_map)].tolist()

    return found_track_count, result_tracks


def get_points_of_tracks(tracks: List[str], points: ndarray) -> ndarray:
    """Extracts points (x, y) from the list of wheel tracks. Only considers points TRACK_ID in `tracks` list."""
    track_points = np.concatenate(list(get_columns_from_track_array(points[points[:, COLUMNS.TRACK_ID] == track_id],
                                                                    [COLUMNS.X, COLUMNS.Y])
                                       for track_id in tracks), axis=0)
    return track_points.astype(float)


def apply_transformation(points: ndarray, rotation: Rotation, rotation_point: ndarray = np.zeros(2),
                         offset: ndarray = np.zeros(2)) -> ndarray:
    """Applies a transformation (rotation + optionally translation) on a set of points. The function supports both 2-d
    and 3-d points. Optionally, a rotation point can be provided. If `rotation_point` is not given, rotation is around
    origin (0, 0)."""
    dim = len(points[0])
    if dim < 3:
        # extend 2-d points to 3-d
        points = np.append(points, np.zeros((len(points), 3 - dim)), axis=1)
        rotation_point = np.append(rotation_point, np.zeros(3 - dim))

    if np.count_nonzero(rotation_point) != 0:
        # perform rotation (if provided, around a rotation point)
        reset_points = apply_translation(points, -rotation_point)
        rotated_reset_points = rotation.apply(reset_points)
        rotated_points = apply_translation(
            rotated_reset_points, rotation_point)
    else:
        rotated_points = rotation.apply(points)

    if dim < 3:
        rotated_points = rotated_points[:, :dim]

    # optionally apply translation with given offset
    if np.count_nonzero(offset) == 0:
        return rotated_points
    else:
        return apply_translation(rotated_points, offset)


def apply_translation(points: ndarray, offset: ndarray) -> ndarray:
    """Applies translation (shift) onto a set of points"""
    for i in range(len(points)):
        points[i] += offset
    return points


def compute_rotation_and_offset(points: ndarray, wheel_map: ndarray, template: VehicleTemplate, max_pairs: int) \
        -> Tuple[Rotation, ndarray]:
    """Computes rotation and offset of thw template according to measurement points. The `wheel_map` must have the same
    length as `points` and indicates the corresponding template wheel index of each entry of `points`. `points` further
    must only contain points that are assigned to the vehicle"""
    # filter point pairs to existing points
    point_pair_indices: ndarray = template.arg_pairs_by_distance[
        np.isin(template.arg_pairs_by_distance, wheel_map).all(1)]
    point_pair_indices = point_pair_indices[:max_pairs]
    point_pair_indices_t = point_pair_indices.T[0], point_pair_indices.T[1]
    # get pairs of points for computations
    orig_indices = wheel_map.argsort()
    pair_to_point_indices = orig_indices[np.searchsorted(
        wheel_map[orig_indices], point_pair_indices)]
    point_pairs = points[pair_to_point_indices, ...]
    # compute angles
    point_diff = point_pairs[:, 1, :] - point_pairs[:, 0, :]
    point_angles = np.arctan2(point_diff[..., 1], point_diff[..., 0])
    # compute angle differences
    template_angles = template.angles[point_pair_indices_t]
    angle_diff = np.mod(point_angles - template_angles, 2 * np.pi)
    # calculate weighted average rotation
    distance_weights = template.distances[point_pair_indices_t]
    rotation = Rotation.from_euler(
        'z', weighted_circular_mean(angle_diff, distance_weights))

    # calc offset of arbitrary existing point to rotated template origin
    point = points[0]
    wheel_id = wheel_map[0]
    point_zero_offset = template.points[wheel_id]
    point_zero_offset_rotated = apply_transformation(
        np.array([point_zero_offset]), rotation)[0]
    offset = point - point_zero_offset_rotated

    return rotation, offset


def weighted_circular_mean(angles: ndarray, weights: ndarray):
    """Computes a weighted mean of angles respecting circular effects like `2 * pi = 0` """
    x = np.nansum(np.cos(angles) * weights)
    y = np.nansum(np.sin(angles) * weights)
    return np.arctan2(y, x)


def try_find_point(points: ndarray, point_to_find: ndarray, search_width: float, distance_fnc: Callable) \
        -> Tuple[bool, str]:
    """Attempts to find a measurement points around a target point. The search radius is `search_width`. For multiple
    hits, the closest point is returned."""
    """input_error is sufficient as error cannot amplify due to longest-distance-first-approach"""
    if len(points) == 0:
        return False, ''

    # compute distances between candidate points and `point_to_find`
    distances: ndarray = distance_fnc(
        get_columns_from_track_array(points, [COLUMNS.X, COLUMNS.Y], astype=float)
        - point_to_find,
        axis=1,
    )
    min_point_index = np.argmin(distances)

    # only return successfully if closest point is within search area
    if distances[min_point_index] > search_width:
        return False, ''
    else:
        min_point = points[min_point_index, COLUMNS.TRACK_ID]
        return True, min_point
