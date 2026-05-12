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


from typing import Dict, Any, List
from processors import ObjectProcessor, MetricsCalculator
from models import EvaluationResult
from utils import sort_objects, print_dict_structure


class EvaluationPipeline:
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.processor = ObjectProcessor()
        self.calculator = MetricsCalculator()

    def process_track_type(
        self,
        ref_data: Dict[str, Any],
        tracked_data: Dict[str, Any],
        ref_classes: Dict[str, Any],
    ) -> EvaluationResult:
        processed_ref = self.processor.preprocess_data(ref_data, True, self.config)
        processed_tracked = self.processor.preprocess_data(
            tracked_data, False, self.config
        )

        (normalized_ref, normalized_tracked) = self.processor.normalize_time(
            processed_ref, processed_tracked
        )

        evaluated = self.processor.interpolate_reference(
            normalized_ref, normalized_tracked, self.config
        )
        evaluated = sort_objects(evaluated, "start_time")
        evaluated = self.processor.compute_classification_error(evaluated, ref_classes)

        all_objects, matched_objects = self.processor.match_objects(
            evaluated, self.config
        )
        metrics = self.calculator.compute_metrics(matched_objects, all_objects, tracked_data)

        return EvaluationResult(
            position_rmse=metrics["position_rmse"],
            orientation_rmse=metrics["orientation_rmse"],
            dimension_rmse=metrics["dimension_rmse"],
            position_error=metrics["position_error"],
            orientation_error=metrics["orientation_error"],
            dimension_error=metrics["dimension_error"],
            classification_accuracy=metrics["classification_accuracy"],
            matches_number=metrics["matches_number"],
            metrics=metrics,
            all_objects=all_objects,
            matched_objects=matched_objects,
        )
