#!/usr/bin/env python3

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


import os
import argparse
from config import cfg
from pathlib import Path
from typing import Dict, Any, Tuple, List
from files import read_objects_hdf, read_ref_classes, write_objects_hdf5
from plotting import plot_tracking_error, plot_dimension_error, plot_timing_error
from loggers import MatchesMetricsLogger, GlobalMetricsLogger
from pipeline import EvaluationPipeline
from models import EvaluationResult
from processors import compute_plotting_data
from utils import print_summary_metrics, print_dict_structure
import matplotlib.pyplot as plt
from concurrent.futures import ProcessPoolExecutor, as_completed
from copy import deepcopy
import multiprocessing
import numpy as np


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline HDF evaluator with ObjectState-like data"
    )
    parser.add_argument(
        "--hdf", required=True, nargs="+", help="Path(s) to HDF5 file(s)"
    )
    parser.add_argument(
        "--classes",
        required=False,
        nargs="+",
        help="Path(s) to CSV file(s) containing a reference label for each object",
    )
    parser.add_argument(
        "--output", default="evaluation_metrics.csv", help="CSV output file"
    )
    return parser.parse_args()


def load_reference_classes(csv_paths: List[str]) -> Dict[str, Dict]:
    ref_classes_by_type = {}

    for csv_path in csv_paths:
        curr_ref_raw_classes = read_ref_classes(csv_path)
        prefix = os.path.splitext(os.path.basename(csv_path))[0]

        for track_type, obj_data in curr_ref_raw_classes.items():
            if track_type not in ref_classes_by_type:
                ref_classes_by_type[track_type] = {}

            ref_classes_by_type[track_type].update(
                {f"{prefix}_{k}": v for k, v in obj_data.items()}
            )

    return ref_classes_by_type


def load_tracked_data(hdf_paths: List[str]) -> Tuple[Dict, Dict]:
    ref_raw_data = {}
    tracked_raw_data_by_type = {}

    for hdf_path in hdf_paths:
        curr_ref_raw_data, curr_tracked_raw_data = read_objects_hdf(hdf_path)
        prefix = os.path.splitext(os.path.basename(hdf_path))[0]

        ref_raw_data.update({f"{prefix}_{k}": v for k, v in curr_ref_raw_data.items()})

        for track_type, obj_data in curr_tracked_raw_data.items():
            if track_type not in tracked_raw_data_by_type:
                tracked_raw_data_by_type[track_type] = {}

            tracked_raw_data_by_type[track_type].update(
                {f"{prefix}_{k}": v for k, v in obj_data.items()}
            )

    return ref_raw_data, tracked_raw_data_by_type


def setup_output_directories(track_type: str) -> None:
    Path(f"./tmp/{track_type}/dimension").mkdir(parents=True, exist_ok=True)
    Path(f"./tmp/{track_type}/tracking").mkdir(parents=True, exist_ok=True)
    # Path(f"./tmp/{track_type}/timing").mkdir(parents=True, exist_ok=True)


def _save_plots_worker(args):
    # worker runs in separate process: import inside function to ensure fresh matplotlib backend
    track_type, index, match_id, obj_data = args
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from processors import compute_plotting_data
    from plotting import plot_tracking_error, plot_dimension_error

    # prepare data and output dirs
    compute_plotting_data(obj_data)
    os.makedirs(f"tmp/{track_type}/tracking", exist_ok=True)
    os.makedirs(f"tmp/{track_type}/dimension", exist_ok=True)

    # tracking plot
    fig = plot_tracking_error(obj_data)
    fig.savefig(f"tmp/{track_type}/tracking/{index}.png", dpi=300)
    plt.close(fig)

    # dimension plot
    fig = plot_dimension_error(obj_data)
    fig.savefig(f"tmp/{track_type}/dimension/{index}.png", dpi=300)
    plt.close(fig)

    return index


def save_plots(track_type: str, matched_objects: Dict[str, Any]) -> None:
    # prepare tasks
    keys = list(matched_objects.keys())
    if not keys:
        return

    max_workers = min((multiprocessing.cpu_count() or 1), 8)
    tasks = []
    for index, match_id in enumerate(keys):
        # deepcopy ensures no shared-state issues when pickling complex objects
        obj_data = deepcopy(matched_objects[match_id])
        tasks.append((track_type, index, match_id, obj_data))

    # run workers in processes to bypass GIL and allow matplotlib Agg backend per process
    with ProcessPoolExecutor(max_workers=max_workers) as exc:
        futures = {exc.submit(_save_plots_worker, task): task[1] for task in tasks}
        for fut in as_completed(futures):
            try:
                _ = fut.result()
            except Exception as e:
                # keep short: print minimal info
                print(f"Warning: failed to save plot index={futures[fut]}: {e}")


def save_metrics(track_type: str, result: EvaluationResult) -> None:
    matches_logger = MatchesMetricsLogger(track_type)
    global_logger = GlobalMetricsLogger(track_type)

    for _, obj_data in result.matched_objects.items():
        matches_logger.log(obj_data)

    global_logger.log(result.metrics)


def save_objects(track_type: str, result: EvaluationResult) -> None:
    write_objects_hdf5(result.matched_objects, f"tmp/{track_type}/matched_objects.hdf5")
    write_objects_hdf5(result.all_objects, f"tmp/{track_type}/all_objects.hdf5")


def process_track_results(track_type: str, result: EvaluationResult) -> None:
    print(f"\nWriting results and objects for {track_type}...")
    setup_output_directories(track_type)
    if cfg["output"].get("save_plots", False):
        save_plots(track_type, result.matched_objects)
    if cfg["output"].get("save_metrics", False):
        save_metrics(track_type, result)
    if cfg["output"].get("save_objects", False):
        save_objects(track_type, result)


def process_global_results(matched_all: Dict, objects_all: Dict) -> None:
    print("\nProcessing all tracks")
    pipeline = EvaluationPipeline(cfg)
    global_metrics = pipeline.calculator.compute_metrics(matched_all, objects_all)
    print_summary_metrics(global_metrics)

    print("\nWriting global results and objects...")
    global_logger = GlobalMetricsLogger(None)
    global_logger.log(global_metrics)

    write_objects_hdf5(matched_all, "tmp/matched_objects.hdf5")
    write_objects_hdf5(objects_all, "tmp/all_objects.hdf5")


def _find_time_array(obj: Dict) -> Tuple[str, np.ndarray]:
    # try common keys first
    candidates = ["header_times", "header_time", "timestamps", "times", "time_stamps", "frame_times"]
    for k in candidates:
        if k in obj:
            return k, np.asarray(obj[k])
    return None, None


def split_tracks_on_time_gap(tracked_raw_data_by_type: Dict[str, Dict]) -> Dict[str, Dict]:
    max_gap = cfg["preprocessing"].get("max_time_gap", 2.0)

    new_by_type: Dict[str, Dict] = {}

    for track_type, objs in tracked_raw_data_by_type.items():
        new_by_type[track_type] = {}
        for obj_id, obj in objs.items():
            time_key, times = _find_time_array(obj)
            if times is None or len(times) <= 1:
                # nothing to split
                new_by_type[track_type][obj_id] = obj
                continue

            times = np.asarray(times)
            # compute gaps between consecutive times
            diffs = np.diff(times)
            # indices where a new segment should start (start of new segment)
            split_indices = np.where(diffs > max_gap)[0] 
            print(split_indices)
            if split_indices.size == 0:
                new_by_type[track_type][obj_id] = obj
                continue

            # segment boundaries (start:end) pairs
            boundaries = []
            prev = 0
            for si in split_indices:
                boundaries.append((prev, si))
                prev = si 
            boundaries.append((prev, len(times)))

            # for each segment, slice arrays that have length == len(times)
            for seg_idx, (s, e) in enumerate(boundaries):
                # create a shallow deepcopy then replace per-frame arrays
                seg_obj = deepcopy(obj)
                for k, v in list(obj.items()):
                    try:
                        # treat numpy arrays and lists/tuples with matching length as frame-aligned
                        if isinstance(v, np.ndarray) and v.shape[0] == times.shape[0]:
                            seg_obj[k] = v[s:e]
                        elif isinstance(v, (list, tuple)) and len(v) == times.shape[0]:
                            seg_obj[k] = list(v)[s:e]
                    except Exception:
                        # keep original for keys that error or are not sliceable
                        pass
                if len(seg_obj["header_time"]) > 1:
                    print(len(seg_obj["header_time"]))
                    new_id = f"{obj_id}_part{seg_idx}"
                    new_by_type[track_type][new_id] = seg_obj

        # preserve original mapping order where possible
    return new_by_type


def main():
    args = parse_arguments()
    pipeline = EvaluationPipeline(cfg)

    ref_classes_by_type = load_reference_classes(args.classes) if args.classes else {}
    ref_raw_data, tracked_raw_data_by_type = load_tracked_data(args.hdf)

    # split long gaps if requested
    # tracked_raw_data_by_type = split_tracks_on_time_gap(tracked_raw_data_by_type)
    
    objects_all = {}
    matched_all = {}

    for track_type, tracked_raw_data in tracked_raw_data_by_type.items():
        print(f"\nProcessing tracks: {track_type}")

        result = pipeline.process_track_type(
            ref_raw_data, tracked_raw_data, ref_classes_by_type.get(track_type, {})
        )
        print_summary_metrics(result.metrics)

        objects_all.update(result.all_objects)
        matched_all.update(result.matched_objects)

        process_track_results(track_type, result)

    process_global_results(matched_all, objects_all)

    MatchesMetricsLogger.close_all()
    GlobalMetricsLogger.close_all()

    print(f"\nAll tasks finished, the program will now exit.")


if __name__ == "__main__":
    main()
