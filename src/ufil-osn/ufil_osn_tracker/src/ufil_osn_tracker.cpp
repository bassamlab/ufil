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


#include "ufil_osn_tracker/ufil_osn_tracker.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>

#include <ufil_object_tracking/association/association_functions.hpp>
#include <ufil_object_tracking/association/associator.hpp>
#include <ufil_object_tracking/detection/point_cloud_detector.hpp>
#include <ufil_object_tracking/management/deleter/existence_deleter.hpp>
#include <ufil_object_tracking/management/initiator/function_initiator.hpp>
#include <ufil_object_tracking/management/pruner/timed_pruner.hpp>
#include <ufil_object_tracking/models/measurement/linear_kalman_measurement_model.hpp>
#include <ufil_object_tracking/models/transition/extended_cv_transition_model.hpp>
#include <ufil_object_tracking/predict/composite_predictor.hpp>
#include <ufil_object_tracking/predict/state/model_state_predictor.hpp>
#include <ufil_object_tracking/tracking/multi_target_tracker.hpp>
#include <ufil_object_tracking/update/classification/dimension_classification_updater.hpp>
#include <ufil_object_tracking/update/composite_updater.hpp>
#include <ufil_object_tracking/update/dimension/dimension_gridmap_updater.hpp>
#include <ufil_object_tracking/update/probability/existence_estimator.hpp>
#include <ufil_object_tracking/update/state/model_state_updater.hpp>

#include "occlusion_predictor.hpp"
#include "orientation_checker.hpp"

namespace ufil_osn_tracker
{

namespace
{

using TransitionModel =
  ufil::model::transition::ExtendedConstantVelocityTransitionModel<State, Control>;
using MeasurementModel = ufil::model::LinearKalmanMeasurementModel<State, Measurement>;

/**
 * @brief Initializes a new track from a given measurement.
 */
void initiateNewTrack(
  const Measurement & measurement, State & new_track_state, Dimension & new_track_dimension,
  Classification & new_track_classification, ExistenceProbability & new_existence_probability)
{
  new_track_state.x() = measurement.x();
  new_track_state.y() = measurement.y();
  new_track_state.yaw() = measurement.yaw();

  new_track_state.covariance() = State::CovarianceMatrixType::Identity() * 100;

  new_track_dimension.length() = measurement.dimension().length();
  new_track_dimension.width() = measurement.dimension().width();
  new_track_dimension.height() = measurement.dimension().height();
  new_track_dimension.covariance() = Dimension::CovarianceMatrixType::Identity() * 100;

  new_track_classification.other() = 1.0;
  new_track_classification.normalize();

  new_existence_probability.existence() = 0.15;
  new_existence_probability.nonExistence() = 0.75;
}

}  // namespace

/// The tracking pipeline. Recreated on reset().
struct OsnObjectTracker::Pipeline
{
  ufil::association::FunctionAssociator<Track>::SharedPtr associator;
  ufil::management::TimedPruner<Track>::SharedPtr pruner;
  ufil::detection::PointCloudDetector<Track>::SharedPtr detector;
  OcclusionPredictor::SharedPtr occlusion_predictor;
  ufil::tracking::MultiTargetTracker<Track, Detection>::UniquePtr tracker;

  Pipeline()
  {
    TransitionModel::NoiseMatrixType W = TransitionModel::NoiseMatrixType::Zero();
    W(0, 0) = 10;
    W(1, 1) = 10;
    W(2, 2) = 1;
    auto state_transition_model = std::make_shared<TransitionModel>(W);

    auto predictor = std::make_shared<ufil::predict::SequentialPredictor<Track>>();
    predictor->addPredictor(
      std::make_shared<ufil::predict::ModelStatePredictor<Track>>(state_transition_model));
    occlusion_predictor = std::make_shared<OcclusionPredictor>();
    predictor->addPredictor(occlusion_predictor);

    detector = std::make_shared<ufil::detection::PointCloudDetector<Track>>();

    associator = std::make_shared<ufil::association::FunctionAssociator<Track>>(
      &ufil::association::euclidean<Track>, &ufil::association::hungarian);

    auto updater_phase_one = std::make_shared<ufil::update::ParallelUpdater<Track>>();
    updater_phase_one->addUpdater(
      std::make_shared<ufil::update::state::ModelStateUpdater<Track>>(
        std::make_shared<MeasurementModel>()));
    updater_phase_one->addUpdater(
      std::make_shared<ufil::update::dimension::DimensionGridmapUpdater<Track>>(
        ufil::type::Vector3(0.1, 0.1, 0.1), 1.0, 0.3, 0.5, 0.8));

    auto updater = std::make_shared<ufil::update::SequentialUpdater<Track>>();
    updater->addUpdater(updater_phase_one);
    updater->addUpdater(std::make_shared<OrientationChecker>());
    updater->addUpdater(std::make_shared<ufil::update::probability::ExistenceEstimator<Track>>());
    updater->addUpdater(
      std::make_shared<ufil::update::classification::DimensionClassificationUpdater<Track>>());

    auto existence_deleter = std::make_shared<ufil::management::ExistenceDeleter<Track>>(0.1);
    auto initiator =
      std::make_shared<ufil::management::FunctionInitiator<Track>>(&initiateNewTrack);
    pruner = std::make_shared<ufil::management::TimedPruner<Track>>(std::chrono::seconds(1));

    tracker = std::make_unique<ufil::tracking::MultiTargetTracker<Track, Detection>>(
      0.7, std::move(predictor), detector, associator, std::move(updater), initiator,
      std::move(existence_deleter), pruner);
  }

  void configure(const TrackerConfig & config)
  {
    associator->setAssociationThreshold(config.association_threshold);
    pruner->setPruneTime(
      std::chrono::nanoseconds(static_cast<std::int64_t>(config.history_duration * 1e9)));
    detector->setClusterPointRange(config.cluster_min_points, config.cluster_max_points);
    detector->setClusterTolerance(config.cluster_tolerance);
    detector->setDimensionCovarianceMultiplier(config.dimension_covariance_multiplier);
    detector->setPositionCovarianceMultiplier(config.position_covariance_multiplier);
    detector->setOrientationCovarianceMultiplier(config.orientation_covariance_multiplier);
  }
};

OsnObjectTracker::OsnObjectTracker(const TrackerConfig & config)
{
  if (const auto error = validate(config)) {
    throw std::invalid_argument("invalid tracker configuration: " + *error);
  }
  config_ = config;
  reset();
}

OsnObjectTracker::~OsnObjectTracker() = default;

void OsnObjectTracker::configure(const TrackerConfig & config)
{
  if (const auto error = validate(config)) {
    throw std::invalid_argument("invalid tracker configuration: " + *error);
  }
  config_ = config;
  pipeline_->configure(config_);
}

const TrackerConfig & OsnObjectTracker::config() const
{
  return config_;
}

void OsnObjectTracker::reset()
{
  pipeline_ = std::make_unique<Pipeline>();
  pipeline_->configure(config_);
  pipeline_->occlusion_predictor->setGrid(occlusion_grid_);
  last_update_time_.reset();
}

UpdateResult OsnObjectTracker::update(
  std::set<Detection> && detections, ufil::type::Timestamp timestamp)
{
  UpdateResult result = UpdateResult::kUpdated;
  if (last_update_time_) {
    last_time_delta_ = std::chrono::duration<double>(timestamp - *last_update_time_).count();
    if (last_time_delta_ < 0.0) {
      ++rejected_out_of_order_;
      return UpdateResult::kRejectedOutOfOrder;
    }
    if (last_time_delta_ > config_.time_jump_threshold) {
      reset();
      ++time_jump_resets_;
      result = UpdateResult::kResetAfterTimeJump;
    }
  }

  pipeline_->tracker->update(std::move(detections), timestamp);
  last_update_time_ = timestamp;
  ++processed_updates_;
  return result;
}

void OsnObjectTracker::setCurrentFOV(const ufil::type::SensorFOV & fov)
{
  // Kept for monitoring; the current pipeline does not model the field of view.
  fov_ = fov;
}

void OsnObjectTracker::setOcclusionGrid(std::shared_ptr<const ufil::type::OccupancyGrid> grid)
{
  occlusion_grid_ = std::move(grid);
  pipeline_->occlusion_predictor->setGrid(occlusion_grid_);
}

const std::map<ufil::type::Id, Track> & OsnObjectTracker::tracks() const
{
  return pipeline_->tracker->tracks();
}

const std::set<Measurement> & OsnObjectTracker::unassociatedMeasurements() const
{
  return pipeline_->tracker->unassociatedMeasurements();
}

const std::map<ufil::type::Id, Measurement> & OsnObjectTracker::associations() const
{
  return pipeline_->tracker->associations();
}

TrackerStatus OsnObjectTracker::status() const
{
  // The association counters of the upstream tracker are not const-qualified.
  auto & tracker = *pipeline_->tracker;
  TrackerStatus status;
  status.processed_updates = processed_updates_;
  status.time_jump_resets = time_jump_resets_;
  status.rejected_out_of_order = rejected_out_of_order_;
  status.last_time_delta = last_time_delta_;
  status.active_tracks = tracker.tracks().size();
  status.associations = tracker.associations().size();
  status.unassociated_measurements = tracker.unassociatedMeasurements().size();
  status.total_detector_associations = tracker.totalDetectorAssociationCount();
  status.total_associator_associations = tracker.totalAssociatorAssociationCount();
  status.occlusion_grid_available = occlusion_grid_ != nullptr;
  status.fov_available = fov_.has_value();
  return status;
}

}  // namespace ufil_osn_tracker
