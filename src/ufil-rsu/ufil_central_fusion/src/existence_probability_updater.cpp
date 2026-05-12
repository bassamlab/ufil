// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWHT Aachen University
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

#include <Eigen/Dense>

#include <ranges>
#include <filesystem>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "ufil_central_fusion/existence_probability_updater.hpp"

namespace ufil_central_fusion
{

// -----------------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------------

void ProbDempsterShaferUpdater::predict_track(
  const TrackType & track,
  const ufil::type::Timestamp & timestamp,
  ufil::type::Scalar & exist_prediction,
  ufil::type::Scalar & non_exist_prediction,
  ufil::type::Scalar & uncertainty_prediction)
{
  // time since last state update
  const auto & prior_state = track.previousState();
  ufil::type::Duration delta_t = (timestamp - prior_state.timestamp());
  double delta_seconds = ufil::to_seconds(delta_t);

  // weight factor (decays exponentially with time, clamped between configured bounds)
  ufil::type::Scalar weight = (1.0f - std::exp(-3.0f * delta_seconds));
  weight = std::clamp(weight, weight_min_, weight_max_);

  const auto & prior_bba = track.previousExistenceProbability().basicBelief();

  exist_prediction = (1.0f - weight) * prior_bba.existence();
  non_exist_prediction = (1.0f - weight) * prior_bba.nonExistence();
  uncertainty_prediction = prior_bba.uncertainty() +
    weight * (prior_bba.existence() + prior_bba.nonExistence());
}

namespace
{
/// Convert Cartesian coordinates to polar (radius, angle)
inline std::tuple<ufil::type::Scalar, ufil::type::Scalar> cartesian_to_polar(
  ufil::type::Scalar x,
  ufil::type::Scalar y)
{
  ufil::type::Scalar radius = std::sqrt(x * x + y * y);
  ufil::type::Scalar angle = std::atan2(y, x);
  return {radius, angle};
}
}  // namespace

ufil::type::Scalar ProbDempsterShaferUpdater::calculate_persistence_prob(
  const TrackType & track,
  ufil::type::SensorFOV sensor_fov)
{
  auto state = track.currentState();
  ufil::type::Scalar x = state.position().x() - sensor_fov.originX();
  ufil::type::Scalar y = state.position().y() - sensor_fov.originY();

  ufil::type::Scalar radius_min = sensor_fov.rMin();
  ufil::type::Scalar radius_max = sensor_fov.rMax();
  ufil::type::Scalar phi_min = sensor_fov.phiMin();
  ufil::type::Scalar phi_max = sensor_fov.phiMax();

  const double r = std::sqrt(x * x + y * y);
  const double phi = std::atan2(y, x);

  const double r_margin = 3.0;
  const double phi_margin = 0.17;

  constexpr double log_alpha = std::log(0.1);

  auto logisticEdge = [&](double value, double inner, double outer) {
      const double decay = -log_alpha * (value - outer) / (outer - inner);
      return 0.5 * (1.0 + std::exp(decay));
    };

  double p_r = 0.0;
  if (r < radius_min || r > radius_max) {
    p_r = 0.0;
  } else if (r >= (radius_min + r_margin) && r <= (radius_max - r_margin)) {
    p_r = 1.0;
  } else if (r < (radius_min + r_margin)) {
    p_r = logisticEdge(r, radius_min + r_margin, radius_min);
  } else if (r > (radius_max - r_margin)) {
    p_r = logisticEdge(r, radius_max, radius_max - r_margin);
  }

  double p_phi = 0.0;
  if (phi < phi_min || phi > phi_max) {
    p_phi = 0.0;
  } else if (phi >= (phi_min + phi_margin) && phi <= (phi_max - phi_margin)) {
    p_phi = 1.0;
  } else if (phi < (phi_min + phi_margin)) {
    p_phi = logisticEdge(phi, phi_min + phi_margin, phi_min);
  } else if (phi > (phi_max - phi_margin)) {
    p_phi = logisticEdge(phi, phi_max, phi_max - phi_margin);
  }

  return std::clamp(p_r * p_phi, 0.0, 1.0);
}


ufil::type::Scalar ProbDempsterShaferUpdater::decay_probability(
  const TrackType & track,
  const double dt)
{
  size_t consecutive_misses = 0;
  for (const auto & entry : std::ranges::reverse_view{track.history()}) {
    if (entry.second.hasAssociatedMeasurement()) {
      break;
    }
    consecutive_misses++;
  }

  ufil::type::Scalar probability = track.previousExistenceProbability().existence();
  ufil::type::Scalar decay = 0.0f;
  if(consecutive_misses > 1) {
    decay = probability * std::min(1.0, consecutive_misses * decay_factor_ * dt);
  }
  return probability - decay;
}

// -----------------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------------

ProbDempsterShaferUpdater::ProbDempsterShaferUpdater(
  ufil::type::Scalar weight_min,
  ufil::type::Scalar weight_max,
  ufil::type::Scalar decay_factor)
{
  assert(weight_min >= 0.0 && weight_min <= 1.0);
  assert(weight_max >= 0.0 && weight_max <= 1.0);
  assert(weight_min <= weight_max);

  weight_min_ = weight_min;
  weight_max_ = weight_max;
  decay_factor_ = decay_factor;
}

// -----------------------------------------------------------------------------
// Public update()
// -----------------------------------------------------------------------------

void ProbDempsterShaferUpdater::update(
  const TrackType & track,
  const std::optional<MeasurementType> & measurement,
  const ufil::type::Timestamp & timestamp,
  ExistenceProbabilityType & resulting_existence_probability)
{
  const auto & prior_state = track.previousState();
  const ufil::type::Duration delta_t = (timestamp - prior_state.timestamp());
  const double dt = ufil::to_seconds(delta_t);

  // We are just predicting there is no sensor or measurement connected to this iteration.
  SensorData chosen_sensor = this->sensor_data_;
  if(chosen_sensor.isPredictionSensor()) {
    ufil::type::Scalar decayed_probability = decay_probability(track, dt);
    resulting_existence_probability.existence() = std::clamp(decayed_probability,
        static_cast<ufil::type::Scalar>(0.0), static_cast<ufil::type::Scalar>(1.0));
    return;
  }

  ufil::type::Scalar sensor_trust_prob = chosen_sensor.existenceProbTrust();
  ufil::type::Scalar sensor_persistence_prob = 0.0f;

  if (chosen_sensor.persistencePossible()) {
    sensor_persistence_prob = calculate_persistence_prob(track, chosen_sensor.sensorFOV());
  }

  // ---------------------------------------------------------------------------
  // Case 1: object inside FOV / persistence region
  // ---------------------------------------------------------------------------
  if (sensor_persistence_prob > 0.0) {
    ufil::type::Scalar exist_prediction = 0.0f;
    ufil::type::Scalar non_exist_prediction = 0.0f;
    ufil::type::Scalar uncertainty_prediction = 0.0f;
    predict_track(track, timestamp, exist_prediction, non_exist_prediction, uncertainty_prediction);

    ufil::type::Scalar bba_exist_sensor = 0.0f;
    ufil::type::Scalar bba_non_exist_sensor = 0.0f;
    ufil::type::Scalar bba_uncertainty_sensor = 0.0f;
    ufil::type::Scalar denominator = 0.0f;

    ufil::type::Scalar sensor_existence_prob = 0.0f;
    if (measurement) {
      sensor_existence_prob = measurement->existenceProbability();
    }

    bba_exist_sensor = sensor_persistence_prob * sensor_trust_prob * sensor_existence_prob;
    bba_non_exist_sensor = sensor_persistence_prob * sensor_trust_prob *
      (1.0f - sensor_existence_prob);
    bba_uncertainty_sensor = 1.0f - (bba_exist_sensor + bba_non_exist_sensor);

    denominator = (1.0f - (exist_prediction * bba_non_exist_sensor +
      non_exist_prediction * bba_exist_sensor));

      // if (denominator != 0.0f) {
    resulting_existence_probability.basicBelief().existence() =
      (exist_prediction * bba_exist_sensor +
      exist_prediction * bba_uncertainty_sensor +
      uncertainty_prediction * bba_exist_sensor) /
      denominator;
      // }
      // } else {
      //   bba_exist_sensor = 0.0f;
      //   bba_non_exist_sensor = sensor_persistence_prob * sensor_trust_prob;
      //   bba_uncertainty_sensor = 1.0f - sensor_persistence_prob * sensor_trust_prob;

    //   denominator = 1.0f - (exist_prediction * bba_non_exist_sensor);
    //   if (denominator != 0.0f) {
    //     resulting_existence_probability.basicBelief().existence() =
    //       (exist_prediction * bba_uncertainty_sensor) / denominator;
    //   } else {
    //     std::cerr << "denominator equals 0" << std::endl;
    //   }
    // }

    // denominator = (1.0f - (exist_prediction * bba_non_exist_sensor +
    // non_exist_prediction * bba_exist_sensor));

    resulting_existence_probability.basicBelief().nonExistence() =
      (non_exist_prediction * bba_non_exist_sensor +
      non_exist_prediction * bba_uncertainty_sensor +
      uncertainty_prediction * bba_non_exist_sensor) /
      denominator;

    resulting_existence_probability.basicBelief().uncertainty() =
      (uncertainty_prediction * bba_uncertainty_sensor) / denominator;


    // Convert BBA into existence probability
    ufil::type::Scalar calculated_existence_prob =
      resulting_existence_probability.basicBelief().existence() +
      0.5f * resulting_existence_probability.basicBelief().uncertainty();

    resulting_existence_probability.existence() = std::clamp(
      calculated_existence_prob, static_cast<ufil::type::Scalar>(0.0),
        static_cast<ufil::type::Scalar>(1.0));
  } else {
    // ---------------------------------------------------------------------------
    // Case 2: object outside persistence region
    // ---------------------------------------------------------------------------
    if (measurement) {
      resulting_existence_probability.existence() = std::clamp(
        measurement->existenceProbability() * sensor_trust_prob * 0.5f +
        (2.0f - sensor_trust_prob) * 0.5f * track.previousExistenceProbability().existence(),
         static_cast<ufil::type::Scalar>(0.0), static_cast<ufil::type::Scalar>(1.0));
    } else {
      resulting_existence_probability.existence() = std::clamp(
        decay_probability(track, dt), static_cast<ufil::type::Scalar>(0.01),
          static_cast<ufil::type::Scalar>(0.975));
    }
  }
}

void ProbDempsterShaferUpdater::setCurrentSensorData(const SensorData & sensor_data)
{
  this->sensor_data_ = sensor_data;
}

}  // namespace ufil_central_fusion
