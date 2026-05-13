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


#ifndef UFIL_EXAMPLES_PEDESTRIAN_TRACKER__UFIL_EXAMPLES_PEDESTRIAN_TRACKER_HPP_
#define UFIL_EXAMPLES_PEDESTRIAN_TRACKER__UFIL_EXAMPLES_PEDESTRIAN_TRACKER_HPP_

#include <memory>
#include <set>
#include <map>

#include <ufil_object_tracking/association/association_functions.hpp>
#include <ufil_object_tracking/association/associator.hpp>
#include <ufil_object_tracking/detection/simple_detector.hpp>
#include <ufil_object_tracking/management/deleter/timed_deleter.hpp>
#include <ufil_object_tracking/management/initiator/function_initiator.hpp>
#include <ufil_object_tracking/management/pruner/timed_pruner.hpp>
#include <ufil_object_tracking/models/measurement/linear_kalman_measurement_model.hpp>
#include <ufil_object_tracking/models/transition/random_walk_transition_model.hpp>
#include <ufil_object_tracking/predict/state/model_state_predictor.hpp>
#include <ufil_object_tracking/tracking/multi_target_tracker.hpp>
#include <ufil_object_tracking/types/classification.hpp>
#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/existence_probability.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_object_tracking/types/track.hpp>
#include <ufil_object_tracking/update/composite_updater.hpp>
#include <ufil_object_tracking/update/dimension/dimension_gridmap_updater.hpp>
#include <ufil_object_tracking/update/state/model_state_updater.hpp>

#include "ufil_examples_pedestrian_tracker/visibility_control.h"

namespace ufil_examples_pedestrian_tracker
{

// Type definitions for better readability
using State = ufil::type::state::PoseVelocity2D;
using Dimension = ufil::type::dimension::Dimension3D;
using Classification = ufil::type::classification::ObjectClassification;
using Control = ufil::type::control::None;
using Measurement = ufil::type::measurement::Pose2DWithDimension3D;
using ExistenceProbability = ufil::type::ExistenceProbability;
using Detection = Measurement;
using Track =
  ufil::type::Track<State, Dimension, Classification, ExistenceProbability, Control, Measurement>;
using TransitionModel = ufil::model::transition::RandomWalkTransitionModel<State, Control>;
using MeasurementModel = ufil::model::LinearKalmanMeasurementModel<State, Measurement>;

/**
 * @class PedestrianTracker
 * @brief Implements a pedestrian tracking system using UFIL framework.
 */
class PedestrianTracker
{
private:
  ufil::tracking::MultiTargetTracker<Track, Detection>::UniquePtr tracker_;

public:
  using SharedPtr = std::shared_ptr<PedestrianTracker>;
  using UniquePtr = std::unique_ptr<PedestrianTracker>;

  /**
   * @brief Constructor to initialize the pedestrian tracker.
   */
  PedestrianTracker();

  /**
   * @brief Updates the tracker with new detections.
   * @param detections The detected objects.
   * @param timestamp The timestamp of the detections.
   */
  void update(std::set<Detection> && detections, ufil::type::Timestamp timestamp);

  /**
   * @brief Retrieves the current tracks.
   * @return A map of tracked objects with their unique IDs.
   */
  const std::map<ufil::type::Id, Track> & tracks() const;
};

}  // namespace ufil_examples_pedestrian_tracker

#endif  // UFIL_EXAMPLES_PEDESTRIAN_TRACKER__UFIL_EXAMPLES_PEDESTRIAN_TRACKER_HPP_
