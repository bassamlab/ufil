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
from typing import Dict, Any, Optional
import numpy as np


@dataclass
class EvaluationResult:
    position_rmse: float
    orientation_rmse: float
    dimension_rmse: float
    position_error: np.ndarray
    orientation_error: np.ndarray
    dimension_error: np.ndarray
    classification_accuracy: float
    matches_number: int
    metrics: Dict[str, float]
    all_objects: Dict[str, Any]
    matched_objects: Dict[str, Any]


@dataclass
class ProcessedTrack:
    object_id: str
    position: np.ndarray
    orientation: np.ndarray
    dimension: np.ndarray
    class_info: Optional[Dict[str, Any]]
    metrics: Dict[str, float]
    reference: Dict[str, Any]
