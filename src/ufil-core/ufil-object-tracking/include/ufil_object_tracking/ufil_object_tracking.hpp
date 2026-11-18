// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef UFIL_OBJECT_TRACKING__UFIL_OBJECT_TRACKING_HPP_
#define UFIL_OBJECT_TRACKING__UFIL_OBJECT_TRACKING_HPP_

#include "ufil_object_tracking/types/classification.hpp"
#include "ufil_object_tracking/types/control.hpp"
#include "ufil_object_tracking/types/dimension.hpp"
#include "ufil_object_tracking/types/existence_probability.hpp"
#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/measurement.hpp"
#include "ufil_object_tracking/types/occupancy_grid.hpp"
#include "ufil_object_tracking/types/scalar.hpp"
#include "ufil_object_tracking/types/sensor_fov.hpp"
#include "ufil_object_tracking/types/state.hpp"
#include "ufil_object_tracking/types/time.hpp"
#include "ufil_object_tracking/types/track.hpp"
#include "ufil_object_tracking/association/association_functions.hpp"
#include "ufil_object_tracking/association/associator.hpp"
#include "ufil_object_tracking/association/hypothesize_functions.hpp"
#include "ufil_object_tracking/association/hypothesizer.hpp"
#include "ufil_object_tracking/detection/detector.hpp"
#include "ufil_object_tracking/detection/simple_detector.hpp"
#include "ufil_object_tracking/management/deleter/deleter.hpp"
#include "ufil_object_tracking/management/deleter/json_history_deleter.hpp"
#include "ufil_object_tracking/serialization/json.hpp"
#include "ufil_object_tracking/management/deleter/timed_deleter.hpp"
#include "ufil_object_tracking/management/deleter/existence_deleter.hpp"
#include "ufil_object_tracking/management/initiator/function_initiator.hpp"
#include "ufil_object_tracking/management/initiator/initiator.hpp"
#include "ufil_object_tracking/management/initiator/simple_initiator.hpp"
#include "ufil_object_tracking/management/pruner/length_pruner.hpp"
#include "ufil_object_tracking/management/pruner/pruner.hpp"
#include "ufil_object_tracking/management/pruner/timed_pruner.hpp"
#include "ufil_object_tracking/models/covariance_model.hpp"
#include "ufil_object_tracking/models/measurement/linear_kalman_measurement_model.hpp"
#include "ufil_object_tracking/models/measurement/linearized_measurement_model.hpp"
#include "ufil_object_tracking/models/measurement/measurement_model.hpp"
#include "ufil_object_tracking/models/transition/no_transition_model.hpp"
#include "ufil_object_tracking/models/transition/random_walk_transition_model.hpp"
#include "ufil_object_tracking/models/transition/cv_transition_model.hpp"
#include "ufil_object_tracking/models/transition/ca_transition_model.hpp"
#include "ufil_object_tracking/models/transition/extended_cv_transition_model.hpp"
#include "ufil_object_tracking/models/transition/extended_ca_transition_model.hpp"
#include "ufil_object_tracking/models/transition/non_linear_kalman_transition_model.hpp"
#include "ufil_object_tracking/models/transition/linear_kalman_transition_model.hpp"
#include "ufil_object_tracking/models/transition/linearized_transition_model.hpp"
#include "ufil_object_tracking/models/transition/transition_model.hpp"
#include "ufil_object_tracking/predict/classification/classification_predictor.hpp"
#include "ufil_object_tracking/predict/composite_predictor.hpp"
#include "ufil_object_tracking/predict/dimension/dimension_predictor.hpp"
#include "ufil_object_tracking/predict/predictor.hpp"
#include "ufil_object_tracking/predict/probability/existence_predictor.hpp"
#include "ufil_object_tracking/predict/selective_predictor.hpp"
#include "ufil_object_tracking/predict/state/model_state_predictor.hpp"
#include "ufil_object_tracking/predict/state/state_predictor.hpp"
#include "ufil_object_tracking/tracking/multi_target_tracker.hpp"
#include "ufil_object_tracking/tracking/tracker.hpp"
#include "ufil_object_tracking/update/composite_updater.hpp"
#include "ufil_object_tracking/update/classification/classification_updater.hpp"
#include "ufil_object_tracking/update/classification/dimension_classification_updater.hpp"
#include "ufil_object_tracking/update/dimension/dimension_gridmap_updater.hpp"
#include "ufil_object_tracking/update/dimension/dimension_updater.hpp"
#include "ufil_object_tracking/update/probability/existence_estimator.hpp"
#include "ufil_object_tracking/update/probability/existence_updater.hpp"
#include "ufil_object_tracking/update/state/model_state_updater.hpp"
#include "ufil_object_tracking/update/state/state_updater.hpp"
#include "ufil_object_tracking/update/updater.hpp"
#include "ufil_object_tracking/utility_functions.hpp"

#endif  // UFIL_OBJECT_TRACKING__UFIL_OBJECT_TRACKING_HPP_
