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

#include "ufil_examples_pedestrian_tracker/pedestrian_tracker.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <ufil_object_tracking/association/association_functions.hpp>
#include <ufil_object_tracking/association/associator.hpp>
#include <ufil_object_tracking/detection/simple_detector.hpp>
#include <ufil_object_tracking/management/deleter/timed_deleter.hpp>
#include <ufil_object_tracking/management/initiator/function_initiator.hpp>
#include <ufil_object_tracking/management/pruner/timed_pruner.hpp>
#include <ufil_object_tracking/models/measurement/linear_kalman_measurement_model.hpp>
#include <ufil_object_tracking/models/transition/random_walk_transition_model.hpp>
#include <ufil_object_tracking/predict/state/model_state_predictor.hpp>
#include <ufil_object_tracking/update/composite_updater.hpp>
#include <ufil_object_tracking/update/dimension/dimension_gridmap_updater.hpp>
#include <ufil_object_tracking/update/state/model_state_updater.hpp>

namespace ufil_examples_pedestrian_tracker
{

namespace
{

using TransitionModel = ufil::model::transition::RandomWalkTransitionModel<State, Control>;
using MeasurementModel = ufil::model::LinearKalmanMeasurementModel<State, Measurement>;

/// Variance of the state and dimension of a new track.
constexpr float kInitialVariance = 100.0F;
/// The multi-target tracker stores this threshold but does not use it.
constexpr float kExistenceThreshold = 0.7F;
/// History of a track kept by the pruner.
constexpr auto kHistoryDuration = std::chrono::seconds(1);

/// Initializes a new pedestrian track from an unassociated measurement.
void initiateNewTrack(
  const Measurement & measurement, State & new_track_state, Dimension & new_track_dimension,
  Classification & new_track_classification, ExistenceProbability & new_existence_probability)
{
  new_track_state.x() = measurement.x();
  new_track_state.y() = measurement.y();
  new_track_state.yaw() = measurement.yaw();
  new_track_state.covariance() = State::CovarianceMatrixType::Identity() * kInitialVariance;

  new_track_dimension.length() = measurement.dimension().length();
  new_track_dimension.width() = measurement.dimension().width();
  new_track_dimension.covariance() = Dimension::CovarianceMatrixType::Identity() * kInitialVariance;

  new_track_classification.pedestrian() = 1.0;
  new_track_classification.normalize();

  new_existence_probability.existence() = measurement.existenceProbability();
}

}  // namespace

std::optional<std::string> validate(const TrackerConfig & config)
{
  if (!(config.process_noise > 0.0)) {
    return "process_noise must be > 0";
  }
  if (!(config.track_timeout > 0.0)) {
    return "track_timeout must be > 0";
  }
  if (!(config.max_time_gap > 0.0)) {
    return "max_time_gap must be > 0";
  }
  return std::nullopt;
}

PedestrianTracker::PedestrianTracker(const TrackerConfig & config)
: config_(config)
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument("invalid tracker configuration: " + *error);
  }
  reset();
}

void PedestrianTracker::reset()
{
  auto detector = std::make_shared<ufil::detection::SimpleDetector<Track, Measurement>>();

  const auto noise = static_cast<float>(config_.process_noise);
  TransitionModel::NoiseMatrixType model_noise;
  model_noise << noise, 0.0F, 0.0F, noise;
  auto state_transition_model = std::make_shared<TransitionModel>(model_noise);
  auto predictor =
    std::make_shared<ufil::predict::ModelStatePredictor<Track>>(state_transition_model);

  auto associator = std::make_shared<ufil::association::FunctionAssociator<Track>>(
    &ufil::association::wasserstein<Track>, &ufil::association::hungarian);

  auto updater = std::make_shared<ufil::update::ParallelUpdater<Track>>();
  auto state_measurement_model = std::make_shared<MeasurementModel>();
  updater->addUpdater(
    std::make_shared<ufil::update::state::ModelStateUpdater<Track>>(state_measurement_model));
  updater->addUpdater(std::make_shared<ufil::update::dimension::DimensionGridmapUpdater<Track>>());

  auto initiator = std::make_shared<ufil::management::FunctionInitiator<Track>>(&initiateNewTrack);
  auto deleter = std::make_shared<ufil::management::TimedDeleter<Track>>(
    ufil::from_seconds<ufil::type::Duration>(config_.track_timeout));
  auto pruner = std::make_shared<ufil::management::TimedPruner<Track>>(kHistoryDuration);

  tracker_ = std::make_unique<ufil::tracking::MultiTargetTracker<Track, Detection>>(
    kExistenceThreshold, std::move(predictor), std::move(detector), std::move(associator),
    std::move(updater), std::move(initiator), std::move(deleter), std::move(pruner));
}

UpdateEvent PedestrianTracker::update(
  std::set<Detection> && detections, const ufil::type::Timestamp timestamp)
{
  auto event = UpdateEvent::kUpdated;
  if (last_update_) {
    const double gap = ufil::to_seconds(timestamp - *last_update_);
    if (gap < 0.0 || gap > config_.max_time_gap) {
      reset();
      ++time_jump_resets_;
      event = UpdateEvent::kResetAfterTimeJump;
    }
  }
  last_update_ = timestamp;
  ++updates_;
  tracker_->update(std::move(detections), timestamp);
  return event;
}

const std::map<ufil::type::Id, Track> & PedestrianTracker::tracks() const
{
  return tracker_->tracks();
}

const TrackerConfig & PedestrianTracker::config() const
{
  return config_;
}

TrackerStatus PedestrianTracker::status() const
{
  TrackerStatus status;
  status.active_tracks = tracker_->tracks().size();
  status.updates = updates_;
  status.time_jump_resets = time_jump_resets_;
  status.last_update = last_update_;
  return status;
}

}  // namespace ufil_examples_pedestrian_tracker
