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

#include <ufil_object_tracking/predict/composite_predictor.hpp>
#include <ufil_object_tracking/predict/state/model_state_predictor.hpp>
#include <ufil_object_tracking/update/composite_updater.hpp>
#include <ufil_object_tracking/update/dimension/dimension_gridmap_updater.hpp>
#include <ufil_object_tracking/update/probability/existence_estimator.hpp>
#include <ufil_object_tracking/update/state/model_state_updater.hpp>
#include <ufil_object_tracking/management/deleter/existence_deleter.hpp>

#include "classifier.hpp"
#include "orientation_checker.hpp"
#include "existence_estimator.hpp"
#include "existence_updater_bayesian.hpp"

namespace ufil_osn_tracker
{

/**
 * @brief Initializes a new pedestrian track from a given measurement.
 * @param measurement The initial measurement.
 * @param new_track_state The initialized state of the track.
 * @param new_track_dimension The dimension of the new track.
 * @param new_track_classification The classification of the new track.
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

OsnObjectTracker::OsnObjectTracker()
{
  TransitionModel::NoiseMatrixType W = TransitionModel::NoiseMatrixType::Zero();
  W(0, 0) = 10;
  W(1, 1) = 10;
  W(2, 2) = 1;

  auto state_transition_model = std::make_shared<TransitionModel>(W);

  auto predictor = std::make_shared<ufil::predict::SequentialPredictor<Track>>();
  auto state_predictor = std::make_shared<ufil::predict::ModelStatePredictor<Track>>(
    state_transition_model);

  // this->occlusion_predictor_ = std::make_shared<ufil_osn_tracker::OcclusionPredictor>();

  // this->existence_predictor_ = std::make_shared<ufil_osn_tracker::BayesianExistencePredictor>();
  predictor->addPredictor(state_predictor);
  // predictor->addPredictor(occlusion_predictor_);
  // predictor->addPredictor(existence_predictor_);

  this->detector_ = std::make_shared<Detector>();

  this->associator_ = std::make_shared<ufil::association::FunctionAssociator<Track>>(
    &ufil::association::euclidean<Track>, &ufil::association::hungarian);

  auto updater_phase_one = std::make_shared<ufil::update::ParallelUpdater<Track>>();
  auto state_measurement_model = std::make_shared<MeasurementModel>();
  auto state_updater =
    std::make_shared<ufil::update::state::ModelStateUpdater<Track>>(state_measurement_model);
  updater_phase_one->addUpdater(state_updater);
  auto dimension_updater =
    std::make_shared<ufil::update::dimension::DimensionGridmapUpdater<Track>>(
    ufil::type::Vector3(
      0.1, 0.1, 0.1), 1.0, 0.3, 0.5, 0.8);
  updater_phase_one->addUpdater(dimension_updater);

  auto updater = std::make_shared<ufil::update::SequentialUpdater<Track>>();
  updater->addUpdater(updater_phase_one);
  updater->addUpdater(std::make_shared<ufil_osn_tracker::OrientationChecker>());
  // updater->addUpdater(
  // std::make_shared<ufil_osn_tracker::BayesianExistenceUpdater>(
  // this->existence_predictor_));
  updater->addUpdater(std::make_shared<ufil::update::probability::ExistenceEstimator<Track>>());
  updater->addUpdater(std::make_shared<ufil_osn_tracker::Classifier>());

  auto existence_deleter = std::make_shared<ufil::management::ExistenceDeleter<Track>>(0.1);
  // this->occlusion_initiator_ = std::make_shared<ufil_osn_tracker::OcclusionInitiator>(
  // &initiateNewTrack);
  auto initiator = std::make_shared<ufil::management::FunctionInitiator<Track>>(&initiateNewTrack);
  this->pruner_ = std::make_shared<ufil::management::TimedPruner<Track>>(std::chrono::seconds(1));

  this->tracker_ = std::make_unique<ufil::tracking::MultiTargetTracker<Track, Detection>>(
    0.7, std::move(predictor), this->detector_, this->associator_, std::move(updater), initiator,
    std::move(existence_deleter), this->pruner_);
}

bool OsnObjectTracker::setAssociationThreshold(const ufil::type::Scalar threshold)
{
  if (!this->associator_) {
    return false;
  }
  if (threshold < 0) {
    return false;
  }
  this->associator_->setAssociationThreshold(threshold);
  return true;
}

bool OsnObjectTracker::setDimensionCovarianceMultiplier(
  const ufil::type::Scalar dimension_covariance_multiplier)
{
  this->detector_->setDimensionCovarianceMultiplier(dimension_covariance_multiplier);
  return true;
}

bool OsnObjectTracker::setPositionCovarianceMultiplier(
  const ufil::type::Scalar position_covariance_multiplier)
{
  this->detector_->setPositionCovarianceMultiplier(position_covariance_multiplier);
  return true;
}

bool OsnObjectTracker::setOrientationCovarianceMultiplier(
  const ufil::type::Scalar orientation_covariance_multiplier)
{
  this->detector_->setOrientationCovarianceMultiplier(orientation_covariance_multiplier);
  return true;
}

bool OsnObjectTracker::setHistoryLength(const ufil::type::Scalar history_length)
{
  if (!this->pruner_) {
    return false;
  }
  if (history_length < 0) {
    return false;
  }
  this->pruner_->setPruneTime(
    std::chrono::nanoseconds(
      static_cast<int64_t>(history_length *
      1e9)));
  return true;
}

void OsnObjectTracker::setCurrentFOV(const ufil::type::SensorFOV & fov)
{
  if (this->existence_predictor_) {
    this->existence_predictor_->setCurrentFOV(fov);
  }
}

void OsnObjectTracker::setOcclusionGrid(std::shared_ptr<const nav_msgs::msg::OccupancyGrid> grid)
{
  if (this->occlusion_predictor_) {
    this->occlusion_predictor_->setGrid(grid);
  }
  if (this->occlusion_initiator_) {
    this->occlusion_initiator_->setGrid(grid);
  }
}

bool OsnObjectTracker::setClusterPointRange(
  const ufil::type::Scalar min_points,
  const ufil::type::Scalar max_points)
{
  if(!this->detector_) {
    throw std::runtime_error("Detector pointer is not initialized.");
  }
  this->detector_->setClusterPointRange(min_points, max_points);
  return true;
}

bool OsnObjectTracker::setClusterTolerance(const ufil::type::Scalar tolerance)
{
  this->detector_->setClusterTolerance(tolerance);
  return true;
}

const std::map<ufil::type::Id,
  ufil::type::measurement::Pose2DWithDimension3D> & OsnObjectTracker::associations() const
{
  return this->tracker_->associations();
}

const std::set<ufil::type::measurement::Pose2DWithDimension3D> & OsnObjectTracker::
unassociatedMeasurements() const
{
  return this->tracker_->unassociatedMeasurements();
}

std::size_t OsnObjectTracker::detectorAssociationCount() const
{
  return this->tracker_->detectorAssociationCount();
}

std::size_t OsnObjectTracker::associatorAssociationCount() const
{
  return this->tracker_->associatorAssociationCount();
}

std::size_t OsnObjectTracker::totalDetectorAssociationCount() const
{
  return this->tracker_->totalDetectorAssociationCount();
}

std::size_t OsnObjectTracker::totalAssociatorAssociationCount() const
{
  return this->tracker_->totalAssociatorAssociationCount();
}

const std::map<ufil::type::Id, Track> & OsnObjectTracker::tracks() const
{
  return this->tracker_->tracks();
}

void OsnObjectTracker::update(std::set<Detection> && detections, ufil::type::Timestamp timestamp)
{
  this->tracker_->update(std::move(detections), timestamp);
}

}  // namespace ufil_osn_tracker
