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


#include <chrono>

#include "ufil_examples_pedestrian_tracker/ufil_examples_pedestrian_tracker.hpp"

namespace ufil_examples_pedestrian_tracker
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
  new_track_dimension.covariance() = Dimension::CovarianceMatrixType::Identity() * 100;

  new_track_classification.pedestrian() = 1.0;
  new_track_classification.normalize();

  new_existence_probability.existence() = measurement.existenceProbability();
}

  /**
   * @brief Constructs the PedestrianTracker and initializes its components.
   */
PedestrianTracker::PedestrianTracker()
{
  auto detector = std::make_shared<ufil::detection::SimpleDetector<Track, Measurement>>();

  TransitionModel::NoiseMatrixType model_noise;
  model_noise << 10.0f, 0.0f, 0.0f, 10.0f;
  auto state_transition_model = std::make_shared<TransitionModel>(model_noise);
  auto predictor =
    std::make_shared<ufil::predict::ModelStatePredictor<Track>>(state_transition_model);

  auto associator = std::make_shared<ufil::association::FunctionAssociator<Track>>(
        &ufil::association::wasserstein<Track>, &ufil::association::hungarian);

  auto updater = std::make_shared<ufil::update::ParallelUpdater<Track>>();
  auto state_measurement_model = std::make_shared<MeasurementModel>();
  auto state_updater =
    std::make_shared<ufil::update::state::ModelStateUpdater<Track>>(state_measurement_model);
  updater->addUpdater(state_updater);
  auto dimension_updater =
    std::make_shared<ufil::update::dimension::DimensionGridmapUpdater<Track>>();
  updater->addUpdater(dimension_updater);

  auto initiator = std::make_shared<ufil::management::FunctionInitiator<Track>>(&initiateNewTrack);
  auto deleter = std::make_shared<ufil::management::TimedDeleter<Track>>(std::chrono::seconds(1));
  auto pruner = std::make_shared<ufil::management::TimedPruner<Track>>(std::chrono::seconds(1));

  tracker_ = std::make_unique<ufil::tracking::MultiTargetTracker<Track, Detection>>(
        0.7, std::move(predictor), std::move(detector), std::move(associator), std::move(updater),
      std::move(initiator),
        std::move(deleter), std::move(pruner));
}

  /**
   * @brief Retrieves the currently tracked objects.
   * @return A map of tracked objects.
   */
const std::map<ufil::type::Id, Track> & PedestrianTracker::tracks() const
{
  return tracker_->tracks();
}

  /**
   * @brief Updates the pedestrian tracker with new detections.
   * @param detections A set of new detections.
   * @param timestamp The timestamp of the detections.
   */
void PedestrianTracker::update(std::set<Detection> && detections, ufil::type::Timestamp timestamp)
{
  tracker_->update(std::move(detections), timestamp);
}

}  // namespace ufil_examples_pedestrian_tracker
