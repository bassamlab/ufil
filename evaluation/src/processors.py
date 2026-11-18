# Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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


from dataclasses import dataclass
from typing import Dict, Any, Tuple
import numpy as np
from copy import deepcopy
from scipy.interpolate import interp1d
from sklearn.metrics import root_mean_squared_error
from utils import class_index_to_class_name, print_dict_structure


def angdiff(a, b):
    """Compute wrapped angular difference (radians) between arrays."""
    diff = np.arctan2(np.sin(a - b), np.cos(a - b))
    return diff


def vecnorm(x, axis=0):
    """Compute Euclidean norm along given axis."""
    return np.linalg.norm(x, axis=axis)


def interp(ref_t, ref_data, t_target):
    f = interp1d(ref_t, ref_data, kind="linear", fill_value="extrapolate")
    return f(t_target)


def compute_plotting_data(match):
    match.update(
        {
            "plotting": {
                "tracking": {
                    "time": np.array(match["header_time"]),
                    "position": np.array(match["state"]["position"]),
                    "ref_position": np.array(match["reference"]["position"]),
                    "linear_velocity": np.array(match["state"]["linear_velocity"]),
                    "ref_linear_velocity": np.array(match["reference"]["linear_velocity"]),
                    "orientation": np.array(match["state"]["orientation"]),
                    "ref_orientation": np.array(match["reference"]["orientation"]),
                    "position_variance": np.array(
                        match["state"].get(
                            "position_variance",
                            np.array(match["state"]["position"]) * 0,
                        )
                    ),
                    "orientation_variance": np.array(
                        match["state"].get(
                            "orientation_variance",
                            np.array(match["state"]["orientation"]) * 0,
                        )
                    ),
                },
                "dimension": {
                    "time": np.array(match["header_time"]),
                    "dimension": np.array(match["dimension"]["dimension"]),
                    "reference_dimension": np.array(match["reference"]["dimension"]),
                    "dimension_variance": np.array(
                        match["dimension"]["dimension_variance"]
                    ),
                    "dimension_rmse": np.array(match["metrics"]["dimension_rmse"]),
                },
                "timing": {"tracker": np.array(match["header_time"] - 0)},
            }
        }
    )


@dataclass
class DataTransformer:
    def get_transformation(
        self, transform: bool, cfg: Dict
    ) -> Tuple[float, np.ndarray, np.ndarray]:
        theta = 0.0
        R = np.eye(3)
        T = np.eye(3)

        if not transform:
            return theta, R, T

        x_shift = cfg["transformation"]["x_shift"]
        y_shift = cfg["transformation"]["y_shift"]
        theta = cfg["transformation"]["theta"]

        R = np.array(
            [
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1],
            ]
        )

        T = np.array([[1, 0, x_shift], [0, 1, y_shift], [0, 0, 1]])

        return theta, R, T


@dataclass
class ObjectProcessor:
    def preprocess_data(
        self, objects: Dict[str, Any], transform: bool, cfg: Dict
    ) -> Dict[str, Any]:
        processed_objects = {}
        transformer = DataTransformer()
        theta, R, T = transformer.get_transformation(transform, cfg)

        for obj_id, obj_data in objects.items():
            processed_dict = {"object_id": obj_id}
            if not isinstance(obj_data, dict):
                continue

            for key, value in obj_data.items():
                if isinstance(value, dict):
                    processed_dict[key] = self.preprocess_data(
                        {key: value}, transform, cfg
                    )[key]
                else:
                    arr = np.array(value).T

                    if key == "position":
                        if arr.shape[0] == 2:
                            arr = np.vstack([arr, np.ones((1, arr.shape[1]))])
                        elif arr.shape[0] == 3:
                            arr[2, :] = 1
                        arr = T @ (R @ arr)
                        arr = arr[0:2, :]
                    elif key == "linear_velocity":
                        if arr.shape[0] == 3:
                            arr = arr[0:2, :]
                        arr = R[:2, :2] @ arr
                    elif key == "orientation":
                        arr = arr[2, :] + theta
                    elif key == "orientation_variance":
                        arr = np.array(value).T
                        arr = arr[2, :]
                    elif key == "position_variance":
                        arr = arr[0:3, :]
                    elif key in ["dimension", "dimension_variance"]:
                        arr = arr[0:3, :]

                    processed_dict[key] = arr

            state = obj_data.get("state", {})
            if "position" in state:
                positions = np.array(obj_data["state"]["position"])
                diffs = np.diff(positions[:, :2].T, axis=0)
                dist_per_step = np.linalg.norm(diffs, axis=1)
                processed_dict["distance"] = np.sum(dist_per_step)

            if "classification" in obj_data:
                classification = np.array(obj_data["classification"]).T
                class_vector = np.sum(classification, axis=1)
                class_index = np.argmax(class_vector)
                class_value = class_vector[class_index]
                processed_dict["class_probability_vector"] = class_vector / np.sum(
                    class_vector
                )
                processed_dict["class_probability"] = class_value / np.sum(class_vector)
                processed_dict["class"] = class_index_to_class_name(class_index)

            if "existence_probability" in obj_data:
                prob_vector = np.array(obj_data["existence_probability"]).flatten()
                processed_dict["probability_vector"] = prob_vector
                processed_dict["probability"] = np.mean(prob_vector)
                processed_dict["exists"] = processed_dict["probability"] > 0.4

            if "header_time" in obj_data:
                time = np.array(obj_data["header_time"])
                processed_dict["start_time"] = time[0]

            processed_objects[obj_id] = processed_dict

        return processed_objects

    def normalize_time(
        self, ref_data: Dict[str, Any], tracked_data: Dict[str, Any]
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """Shift header_time so the earliest reference start is at t=0 and set start_time."""

        # find minimum first timestamp among reference objects
        min_time = min(np.array(obj["header_time"])[0] for obj in ref_data.values())

        for obj in ref_data.values():
            obj["header_time"] = np.array(obj["header_time"]) - min_time
            obj["start_time"] = obj["header_time"][0]

        for obj in tracked_data.values():
            obj["header_time"] = np.array(obj["header_time"]) - min_time
            obj["start_time"] = obj["header_time"][0]

       
        return ref_data, tracked_data

    def interpolate_reference(
        self, ref_objects: Dict[str, Any], track_objects: Dict[str, Any], cfg: Dict
    ) -> Dict[str, Any]:
        eval_objects = {}

        for track_object_id, track_obj in track_objects.items():
            best_rmse = np.inf
            best_obj = None

            for ref_object_id, ref_obj in ref_objects.items():
                t_track = np.array(track_obj["header_time"])
                t_ref = np.array(ref_obj["header_time"])

                # Skip if track time is outside reference time bounds
                if t_track[0] < t_ref[0]-10e9 or t_track[-1] > t_ref[-1]+10e9:
                    continue
                
                obj = deepcopy(track_obj)

                # Indices (front/back) are placeholders from MATLAB
                front = cfg["preprocessing"]["front_padding"]
                back = cfg["preprocessing"]["rear_padding"]
                t_interp = t_track[front : len(t_track) - back]

                if "reference" not in obj:
                    obj["reference"] = {}

                # Position
                obj["reference"]["position"] = np.vstack(
                    [
                        interp(t_ref, ref_obj["state"]["position"][0, :], t_interp),
                        interp(t_ref, ref_obj["state"]["position"][1, :], t_interp),
                    ]
                )
                obj["state"]["position"] = obj["state"]["position"][
                    :, front : len(t_track) - back
                ]
                obj["state"]["position_variance"] = obj["state"]["position_variance"][
                    :, front : len(t_track) - back
                ]

                # Dimension
                obj["reference"]["dimension"] = np.vstack(
                    [
                        interp(
                            t_ref, ref_obj["dimension"]["dimension"][0, :], t_interp
                        ),
                        interp(
                            t_ref, ref_obj["dimension"]["dimension"][1, :], t_interp
                        ),
                        interp(
                            t_ref, ref_obj["dimension"]["dimension"][2, :], t_interp
                        ),
                    ]
                )
                obj["dimension"]["dimension"] = obj["dimension"]["dimension"][
                    :, front : len(t_track) - back
                ]
                obj["dimension"]["dimension_variance"] = obj["dimension"][
                    "dimension_variance"
                ][:, front : len(t_track) - back]

                # # Check for swapped length/width
                # mid_idx = len(obj["dimension"]["dimension"][0, :]) // 2
                # dim_error = abs(
                #     obj["dimension"]["dimension"][0, mid_idx]
                #     - obj["reference"]["dimension"][0, mid_idx]
                # )
                # if dim_error >= 2:
                #     (
                #         obj["dimension"]["dimension"][0, :],
                #         obj["dimension"]["dimension"][1, :],
                #     ) = (
                #         obj["dimension"]["dimension"][1, :],
                #         obj["dimension"]["dimension"][0, :],
                #     )

                # Orientation
                obj["reference"]["orientation"] = interp(
                    t_ref, ref_obj["state"]["orientation"], t_interp
                )
                obj["state"]["orientation"] = obj["state"]["orientation"][
                    front : len(t_track) - back
                ]
                obj["state"]["orientation_variance"] = obj["state"][
                    "orientation_variance"
                ][front : len(t_track) - back]

                # Fix discontinuities around π wrap-around
                for _ in range(cfg["orientation"]["wrap_check_iterations"]):
                    diff = obj["state"]["orientation"] - obj["reference"]["orientation"]
                    over = diff > cfg["orientation"]["wrap_upper"]
                    under = diff < cfg["orientation"]["wrap_lower"]
                    obj["state"]["orientation"][over] -= np.pi
                    obj["state"]["orientation"][under] += np.pi

                # Velocity
                obj["reference"]["linear_velocity"] = np.vstack(
                    [
                        interp(
                            t_ref, ref_obj["state"]["linear_velocity"][0, :], t_interp
                        ),
                        interp(
                            t_ref, ref_obj["state"]["linear_velocity"][1, :], t_interp
                        ),
                    ]
                )
                obj["state"]["linear_velocity"] = obj["state"]["linear_velocity"][
                    :, front : len(t_track) - back
                ]

                # Class info
                obj["reference"]["class"] = ref_obj["class"]
                obj["reference"]["class_probability"] = ref_obj["class_probability"]

                # Finalize
                obj["reference"]["object_id"] = ref_object_id
                obj["header_time"] = t_interp
                obj["length"] = obj["state"]["position"].shape[1]

                if obj["length"] < cfg["thresholds"]["min_track_length"]:
                    continue
                    # eval_objects[track_object_id] = obj

                pos_rmse = root_mean_squared_error(
                    vecnorm(obj["state"]["position"], axis=0),
                    vecnorm(obj["reference"]["position"], axis=0),
                )

                if pos_rmse < best_rmse:
                    best_rmse = pos_rmse
                    best_obj = obj

            if best_obj is not None:
                eval_objects[track_object_id] = best_obj

            if best_obj is None:
                print(track_object_id, " does not belong to any ref object best rmse was", best_rmse)
                pass
            # else:
                # print("Select ref with rmse ", best_rmse, " with class ", best_obj["class"])

        return eval_objects

    def match_objects(
        self, eval_objects: Dict[str, Any], cfg: Dict
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        all_objects = {}
        matched_objects = {}

        for track_id, track in eval_objects.items():
            state = track["state"]
            reference = track["reference"]
            dimension = track["dimension"]

            # Position RMSEs (x, y, and magnitude)
            position_rmse = np.array(
                [
                    root_mean_squared_error(
                        state["position"][0, :], reference["position"][0, :]
                    ),
                    root_mean_squared_error(
                        state["position"][1, :], reference["position"][1, :]
                    ),
                    root_mean_squared_error(
                        vecnorm(state["position"], axis=0),
                        vecnorm(reference["position"], axis=0),
                    ),
                ]
            )

            # Velocity RMSEs (x, y, and magnitude)
            velocity_rmse = np.array(
                [
                    root_mean_squared_error(
                        state["linear_velocity"][0, :], reference["linear_velocity"][0, :]
                    ),
                    root_mean_squared_error(
                        state["linear_velocity"][1, :], reference["linear_velocity"][1, :]
                    ),
                    root_mean_squared_error(
                        vecnorm(state["linear_velocity"], axis=0),
                        vecnorm(reference["linear_velocity"], axis=0),
                    ),
                ]
            )

            # Orientation RMSE (yaw)
            orientation_diff = angdiff(state["orientation"], reference["orientation"])
            orientation_rmse = root_mean_squared_error(
                orientation_diff, np.zeros_like(orientation_diff)
            )

            # Dimension RMSEs (length, width, height, magnitude)
            dimension_rmse = np.array(
                [
                    root_mean_squared_error(
                        dimension["dimension"][0, :], reference["dimension"][0, :]
                    ),
                    root_mean_squared_error(
                        dimension["dimension"][1, :], reference["dimension"][1, :]
                    ),
                    root_mean_squared_error(
                        dimension["dimension"][2, :], reference["dimension"][2, :]
                    ),
                    root_mean_squared_error(
                        vecnorm(dimension["dimension"], axis=0),
                        vecnorm(reference["dimension"], axis=0),
                    ),
                ]
            )

            # Store computed metrics inside the same object
            track["metrics"] = {
                "position_rmse": position_rmse,
                "velocity_rmse": velocity_rmse,
                "orientation_rmse": orientation_rmse,
                "dimension_rmse": dimension_rmse,
            }

            all_objects.update({track_id: track})

            # Filtering
            if (
                position_rmse[2] >= cfg["thresholds"]["max_position_rmse"]
                or position_rmse[0] >= cfg["thresholds"]["max_position_rmse"]
                or position_rmse[1] >= cfg["thresholds"]["max_position_rmse"]
                or track["length"] < cfg["thresholds"]["min_track_length"]
            ):
                continue
            if dimension_rmse[0] >= cfg["thresholds"]["max_dimension_rmse"]:
                continue

            matched_objects.update({track_id: track})

        return all_objects, matched_objects

    def compute_classification_error(
        self, tracks: Dict[str, Any], ref_classes: Dict[str, Any]
    ) -> Dict[str, Any]:
        for track_id, track in tracks.items():
            ref_class = ref_classes.get(track_id)
            track["ref_class"] = str(ref_class)
            track["class_error"] = ref_class is not None and track["class"] == ref_class

        return tracks


@dataclass
class MetricsCalculator:
    def compute_metrics(
        self, matched_objects: Dict[str, Any], all_objects: Dict[str, Any], tracked_data: Dict[str, Any] = None
    ) -> Dict[str, float]:
        metrics = {
            "matches_number": 0,
            "position_rmse": 0,
            "orientation_rmse": 0,
            "dimension_rmse": 0,
            "classification_accuracy": 0,
            "classification_error_rate": 0,
            "velocity_rmse": 0
        }

        position_list = []
        dimension_list = []
        orientation_list = []
        velocity_list = []
        
        if not tracked_data is None:
            message_time_list = []
            header_time_list = []
            for key, value in tracked_data.items() :
                if "message_time" in key:
                    message_time_list.append(np.array(value))
                if "header_time" in key:
                    header_time_list.append(np.array(value))
            message_time = np.concatenate(message_time_list)
            header_time = np.concatenate(header_time_list)
            processing_times = message_time - header_time
            metrics["processing_time"] = processing_times
            metrics["processing_time_mean"] = np.mean(processing_times)
            metrics["processing_time_std"] = np.std(processing_times)
            metrics["processing_time_min"] = np.min(processing_times)
            metrics["processing_time_max"] = np.max(processing_times)
        
        ref_position_list = []
        ref_dimension_list = []
        ref_orientation_list = []
        ref_velocity_list = []

        existence_probability_list = []

        for object in all_objects.values():
            existence_probability_list.append(np.array(object["probability"]))

        for match in matched_objects.values():
            metrics["matches_number"] += match["length"]

            position_list.append(np.array(match["state"]["position"]))
            dimension_list.append(np.array(match["dimension"]["dimension"]))
            orientation_list.append(np.array(match["state"]["orientation"]))
            velocity_list.append(np.array(match["state"]["linear_velocity"]))

            ref_position_list.append(np.array(match["reference"]["position"]))
            ref_dimension_list.append(np.array(match["reference"]["dimension"]))
            ref_orientation_list.append(np.array(match["reference"]["orientation"]))
            ref_velocity_list.append(np.array(match["reference"]["linear_velocity"]))

        position = (
            np.concatenate(position_list, axis=1) if position_list else np.array([])
        )
        dimension = (
            np.concatenate(dimension_list, axis=1) if dimension_list else np.array([])
        )
        orientation = (
            np.concatenate(orientation_list, axis=0)
            if orientation_list
            else np.array([])
        )
        velocity = (
            np.concatenate(velocity_list, axis=1) if velocity_list else np.array([])
        )

        ref_position = (
            np.concatenate(ref_position_list, axis=1)
            if ref_position_list
            else np.array([])
        )
        ref_dimension = (
            np.concatenate(ref_dimension_list, axis=1)
            if ref_dimension_list
            else np.array([])
        )
        ref_orientation = (
            np.concatenate(ref_orientation_list, axis=0)
            if ref_orientation_list
            else np.array([])
        )
        ref_velocity = (
            np.concatenate(ref_velocity_list, axis=1) if ref_velocity_list else np.array([])
        )

        metrics["classification_accuracy"] = round(
            sum(t["class_error"] for t in all_objects.values())
            / len(all_objects.keys()) if len(all_objects.keys()) > 0 else 1,
            3,
        )
        metrics["classification_error_rate"] = round(
            1 - metrics["classification_accuracy"], 3
        )

        angdiff = (orientation - ref_orientation + np.pi) % (2 * np.pi) - np.pi
        metrics["position_error"] = position - ref_position
        metrics["orientation_error"] = angdiff
        metrics["dimension_error"] = dimension - ref_dimension
        metrics["velocity_error"] = velocity - ref_velocity
        metrics["existence_probability"] = existence_probability_list

        metrics["position_rmse"] = root_mean_squared_error(
            np.linalg.norm(position, axis=0), np.linalg.norm(ref_position, axis=0)
        )

        metrics["orientation_rmse"] = root_mean_squared_error(
            angdiff, np.zeros(metrics["matches_number"])
        )
        metrics["dimension_rmse"] = root_mean_squared_error(
            np.linalg.norm(dimension, axis=0), np.linalg.norm(ref_dimension, axis=0)
        )
        metrics["velocity_rmse"] = root_mean_squared_error(
            np.linalg.norm(velocity, axis=0), np.linalg.norm(ref_velocity, axis=0)
        )

        return metrics
