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


#include "ufil_central_fusion/object_tracker.hpp"
#include "ufil_central_fusion/classification_updater.hpp"
#include "ufil_central_fusion/existence_probability_updater.hpp"
#include "measurement_model.hpp"
#include "orientation_checker.hpp"

#include <ufil_object_tracking/association/association_functions.hpp>
#include <ufil_object_tracking/association/associator.hpp>
#include <ufil_object_tracking/detection/simple_detector.hpp>
#include <ufil_object_tracking/management/deleter/existence_deleter.hpp>
#include <ufil_object_tracking/management/initiator/function_initiator.hpp>
#include <ufil_object_tracking/management/pruner/timed_pruner.hpp>
#include <ufil_object_tracking/predict/composite_predictor.hpp>
#include <ufil_object_tracking/predict/selective_predictor.hpp>
#include <ufil_object_tracking/update/composite_updater.hpp>
#include <ufil_object_tracking/tracking/multi_target_tracker.hpp>
#include <ufil_object_tracking/update/dimension/dimension_gridmap_updater.hpp>
#include <ufil_object_tracking/update/state/model_state_updater.hpp>

namespace ufil_central_fusion
{

ObjectTracker::~ObjectTracker() = default;

void initiateNewTrack(
  const Track::MeasurementType & measurement,
  Track::StateType & state,
  Track::DimensionType & dimension,
  Track::ClassificationType & classification,
  Track::ExistenceProbabilityType & existence)
{
  if(measurement.hasX()) {
    state.x() = measurement.x();
  }
  if(measurement.hasY()) {
    state.y() = measurement.y();
  }
  if(measurement.hasVx()) {
    state.vx() = measurement.vx();
  }
  if(measurement.hasVy()) {
    state.vy() = measurement.vy();
  }
  if(measurement.hasYaw()) {
    state.yaw() = measurement.yaw();
  }
  if(measurement.hasYawRate()) {
    state.yawRate() = measurement.yawRate();
  }
  state.covariance() = Track::StateType::CovarianceMatrixType::Identity() * 100;

  if (measurement.dimension().hasLength()) {
    dimension.length() = measurement.dimension().length();
  }
  if (measurement.dimension().hasWidth()) {
    dimension.width() = measurement.dimension().width();
  }
  if (measurement.dimension().hasHeight()) {
    dimension.height() = measurement.dimension().height();
  }
  dimension.covariance() = Track::DimensionType::CovarianceMatrixType::Identity() * 100;

  existence.existence() = measurement.existenceProbability();

  if(measurement.isCam()) {
    existence.existence() = 0.0f;
  }

  classification.classificationVector() = measurement.classification().classificationVector();
}

ObjectTracker::ObjectTracker(
  ufil::type::Scalar min_existence_weight, ufil::type::Scalar max_existence_weight,
  ufil::type::Scalar decay_factor, ufil::type::Scalar delta_d,
  ufil::type::Scalar alpha, ufil::type::Scalar p_min,
  ufil::type::Scalar p_ref, ufil::type::Scalar p_max,
  ufil::type::Scalar association_threshold)
: min_existence_weight_(min_existence_weight),
  max_existence_weight_(max_existence_weight),
  decay_factor_(decay_factor),
  delta_d_(delta_d),
  alpha_(alpha),
  p_min_(p_min),
  p_ref_(p_ref),
  p_max_(p_max),
  association_threshold_(association_threshold)
{
  PedestrianTransitionModel::NoiseMatrixType noise_pedestrian;
  noise_pedestrian(0, 0) = 2;
  noise_pedestrian(1, 1) = 2;
  auto transition_model_pedestrian = std::make_shared<PedestrianTransitionModel>(noise_pedestrian);

  OtherTransitionModel::NoiseMatrixType noise_other;
  noise_other(0, 0) = 20;
  noise_other(1, 1) = 20;
  noise_other(2, 2) = 2;
  auto transition_model_others = std::make_shared<OtherTransitionModel>(noise_other);

  auto measurement_model = std::make_shared<ufil_central_fusion::DynamicMeasurementModel>();
  predictor_ = std::make_shared<ufil::predict::ParallelPredictor<Track>>();
  auto pedestrian_predictor = std::make_shared<ufil::predict::SelectivePredictor<Track>>(
                                                                transition_model_pedestrian);
  auto others_predictor = std::make_shared<ufil::predict::SelectivePredictor<Track>>(
                                                                    transition_model_others);
  pedestrian_predictor->addClassification(
      ufil::type::classification::ObjectClassification::PEDESTRIAN);
  others_predictor->addClassification(ufil::type::classification::ObjectClassification::BICYCLE);
  others_predictor->addClassification(ufil::type::classification::ObjectClassification::MOTORCYCLE);
  others_predictor->addClassification(ufil::type::classification::ObjectClassification::STATIONARY);
  others_predictor->addClassification(ufil::type::classification::ObjectClassification::TRUCK);
  others_predictor->addClassification(ufil::type::classification::ObjectClassification::CAR);
  others_predictor->addClassification(ufil::type::classification::ObjectClassification::OTHER);
  predictor_->addPredictor(pedestrian_predictor);
  predictor_->addPredictor(others_predictor);

  auto detector = std::make_shared<ufil::detection::SimpleDetector<Track, DynamicMeasurement>>();

  auto associator = std::make_shared<ufil::association::FunctionAssociator<Track>>(
      &ufil::association::euclidean<Track>, &ufil::association::hungarian, association_threshold_);

  auto model_updater =
    std::make_shared<ufil::update::state::ModelStateUpdater<Track>>(std::move(measurement_model));

  auto dimension_updater =
    std::make_shared<ufil::update::dimension::DimensionGridmapUpdater<Track>>(
          ufil::type::Vector3(delta_d_, delta_d_, delta_d_), alpha_, p_min_, p_ref_, p_max_);

  classification_updater_ =
    std::make_shared<ClassificationDempsterShaferUpdater>();

  existence_probability_updater_ =
    std::make_shared<ufil_central_fusion::ProbDempsterShaferUpdater>(min_existence_weight_,
      max_existence_weight_, decay_factor_);

  auto updater = std::make_shared<ufil::update::SequentialUpdater<Track>>();
  updater->addUpdater(std::make_shared<ufil_central_fusion::OrientationChecker>());
  updater->addUpdater(model_updater);
  updater->addUpdater(dimension_updater);
  updater->addUpdater(existence_probability_updater_);
  updater->addUpdater(classification_updater_);

  auto initiator = std::make_shared<ufil::management::FunctionInitiator<Track>>(&initiateNewTrack);
  auto deleter = std::make_shared<ufil::management::ExistenceDeleter<Track>>(0.1);
  auto pruner = std::make_shared<ufil::management::TimedPruner<Track>>(std::chrono::seconds(2));

  this->tracker_ = std::make_unique<ufil::tracking::MultiTargetTracker<Track, DynamicMeasurement>>(
      0.7, predictor_, std::move(detector), std::move(associator), std::move(updater),
      std::move(initiator), std::move(deleter), std::move(pruner));
}

const std::map<ufil::type::Id, Track> & ObjectTracker::tracks() const
{
  return this->tracker_->tracks();
}

void ObjectTracker::update(
  std::set<DynamicMeasurement> && detections,
  ufil::type::Timestamp timestamp)
{
  this->tracker_->update(std::move(detections), timestamp);
}

void ObjectTracker::predict(const ufil::type::Timestamp timestamp)
{
  this->tracker_->update({}, timestamp);
}

void ObjectTracker::rollbackHistory(const ufil::type::Timestamp timestamp)
{
  this->tracker_->rollbackHistory(timestamp);
}

void ObjectTracker::setCurrentSensorData(const SensorData & sensor_data)
{
  this->existence_probability_updater_->setCurrentSensorData(sensor_data);
  this->classification_updater_->setCurrentSensorData(sensor_data);
}


}  // namespace ufil_central_fusion
