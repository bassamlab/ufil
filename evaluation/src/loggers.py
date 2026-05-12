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


# results_logger.py
import csv
from pathlib import Path
from threading import Lock
import numpy as np

class MatchesMetricsLogger:
    _instances = {}
    _lock = Lock()

    def __new__(cls, track_type=None, output_dir="tmp"):
        with cls._lock:
            key = track_type or "global"
            if key not in cls._instances:
                instance = super().__new__(cls)
                instance._init_logger(track_type, output_dir)
                cls._instances[key] = instance
            return cls._instances[key]

    def _init_logger(self, track_type, output_dir):
        self.object_type = track_type
        self.output_dir = (
            Path(output_dir) if track_type is None else Path(output_dir) / track_type
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.file_path = self.output_dir / "matches_metrics.csv"

        self._file = open(self.file_path, mode="w", newline="")
        self._writer = csv.DictWriter(
            self._file,
            fieldnames=[
                "object_id",
                "reference_object_id",
                "points",
                "position_rmse",
                "orientation_rmse",
                "dimension_rmse",
                "class",
                "class_probability",
            ],
        )
        self._writer.writeheader()

    def log(self, track):
        self._writer.writerow(
            {
                "object_id": track["object_id"],
                "reference_object_id": track["reference"]["object_id"],
                "points": track["length"],
                "position_rmse": f"{track['metrics']['position_rmse'][2]:.3f}",
                "orientation_rmse": f"{track['metrics']['orientation_rmse']:.3f}",
                "dimension_rmse": f"{track['metrics']['dimension_rmse'][3]:.3f}",
                "class": track["class"],
                "class_probability": f"{track['class_probability']:.3f}",
            }
        )
        self._file.flush()


    def close(self):
        self._file.close()

    @classmethod
    def close_all(cls):
        for instance in cls._instances.values():
            instance.close()
        cls._instances.clear()


class GlobalMetricsLogger:
    _instances = {}
    _lock = Lock()

    def __new__(cls, track_type=None, output_dir="tmp"):
        with cls._lock:
            key = track_type or "global"
            if key not in cls._instances:
                instance = super().__new__(cls)
                instance._init_logger(track_type, output_dir)
                cls._instances[key] = instance
            return cls._instances[key]

    def _init_logger(self, track_type, output_dir):
        self.object_type = track_type
        self.output_dir = (
            Path(output_dir) if track_type is None else Path(output_dir) / track_type
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.file_path = self.output_dir / "global_metrics.csv"

        self._file = open(self.file_path, mode="w", newline="")
        self._writer = csv.DictWriter(
            self._file,
            fieldnames=[
                "matches_number",
                "position_rmse",
                "orientation_rmse",
                "dimension_rmse",
                "classification_accuracy",
                "classification_error_rate",
            ],
        )
        self._writer.writeheader()

    def log(self, metrics):
        self._writer.writerow(
            {
                "matches_number": f"{metrics['matches_number']}",
                "position_rmse": f"{metrics['position_rmse']:.3f}",
                "orientation_rmse": f"{np.rad2deg(metrics['orientation_rmse']):.3f}",
                "dimension_rmse": f"{metrics['dimension_rmse']:.3f}",
                "classification_accuracy": f"{metrics['classification_accuracy']:.3f}",
                "classification_error_rate": f"{metrics['classification_error_rate']:.3f}",
            }
        )
        self._file.flush()

        track_position_error_file_path = self.output_dir / "position_error.csv"
        track_position_error_mat = np.matrix(metrics['position_error']).transpose()
        track_orientation_error_mat = np.rad2deg(np.matrix(metrics['orientation_error']).transpose())
        tracking_error_mat = np.hstack((track_position_error_mat,track_orientation_error_mat))
        with open(track_position_error_file_path,'wb') as f:
            for line in tracking_error_mat:
                np.savetxt(f, line, fmt='%10.10f')

        track_dimension_error_file_path = self.output_dir / "dimension_error.csv"
        track_dimension_error_mat = np.matrix(metrics['dimension_error']).transpose()
        with open(track_dimension_error_file_path,'wb') as f:
            for line in track_dimension_error_mat:
                np.savetxt(f, line, fmt='%10.10f')

        track_dexitence_probability_file_path = self.output_dir / "exitence_probability.csv"
        track_exitence_probability_mat = np.matrix(metrics['existence_probability']).transpose()
        with open(track_dexitence_probability_file_path,'wb') as f:
            for line in track_exitence_probability_mat:
                np.savetxt(f, line, fmt='%10.10f')

        if 'processing_time' in metrics:
            track_processing_time_file_path = self.output_dir / "processing_time.csv"
            track_processing_time_mat = np.matrix(metrics['processing_time']*1e-9).transpose()
            with open(track_processing_time_file_path,'wb') as f:
                for line in track_processing_time_mat:
                    np.savetxt(f, line, fmt='%10.10f')
                    
    def close(self):
        self._file.close()

    @classmethod
    def close_all(cls):
        for instance in cls._instances.values():
            instance.close()
        cls._instances.clear()
