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

#include "ufil_osn_tracker/existence_predictor_bayesian.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <iomanip>
#include <iostream>

#include <boost/uuid/uuid_io.hpp>

namespace ufil_osn_tracker
{

// ---------------------------------------------------------------------------
// PREDICTION UPDATE
// ---------------------------------------------------------------------------

void BayesianExistencePredictor::predict(
  const TrackType & track,
  const std::optional<ControlType> & /*control*/,
  const ufil::type::Timestamp & /*timestamp*/,
  ExistenceProbabilityType & prediction)
{
  const double prev_exist = track.previousExistenceProbability().existence();
  const double prev_non_exist = track.previousExistenceProbability().nonExistence();

  const double p_persist = computePersistenceProbability(track.currentState().position());

  const double p_birth = computeBirthProbability(track.currentState().position());

  // Bayesian update
  const double pred_exist = prev_exist * p_persist + prev_non_exist * p_birth;
  const double pred_non_exist = prev_exist * (1.0 - p_persist) + prev_non_exist * (1.0 - p_birth);

  // Write back to track with clamping to avoid numerical issues
  constexpr double kEps = 1e-3;

  prediction.existence() = std::clamp(pred_exist, kEps, 1.0 - kEps);
  prediction.nonExistence() = std::clamp(pred_non_exist, kEps, 1.0 - kEps);

  // renormalize
  double sum = prediction.existence() + prediction.nonExistence();
  prediction.existence() /= sum;
  prediction.nonExistence() /= sum;
}


// ---------------------------------------------------------------------------
// PERSISTENCE PROBABILITY
// ---------------------------------------------------------------------------

double BayesianExistencePredictor::computePersistenceProbability(
  const ufil::type::Vector2 & pos)
{
  const double x = pos.x();
  const double y = pos.y();

  const double r = std::sqrt(x * x + y * y);
  const double phi = std::atan2(y, x);

  const double r_min = current_fov_.rMin();
  const double r_max = current_fov_.rMax();
  const double phi_min = current_fov_.phiMin();
  const double phi_max = current_fov_.phiMax();

  const double r_margin = 2.0;
  const double phi_margin = 0.10;

    // ----------------- SIGMOID EDGE -----------------
  auto logisticEdge = [](double value, double start, double end, double sharpness = 10.0)
    {
      double t = (value - start) / (end - start);

        // clamp to [0, 1]
      t = std::clamp(t, 0.0, 1.0);

      return 1.0 / (1.0 + std::exp(-sharpness * (t - 0.5)));
    };

    // ----------------- RANGE -----------------
  double p_r = 0.0;

  if (r < r_min || r > r_max) {
    p_r = 0.0;
  } else if (r <= (r_min + r_margin)) {
    p_r = logisticEdge(r, r_min, r_min + r_margin);
  } else if (r >= (r_max - r_margin)) {
    p_r = logisticEdge(r, r_max, r_max - r_margin);
  } else {
    p_r = 1.0;
  }

    // ----------------- ANGLE -----------------
  double p_phi = 0.0;

  if (phi < phi_min - phi_margin || phi > phi_max + phi_margin) {
    p_phi = 0.0;
  } else if (phi <= (phi_min + phi_margin)) {
    p_phi = logisticEdge(phi, phi_min - phi_margin, phi_min + phi_margin);
  } else if (phi >= (phi_max - phi_margin)) {
    p_phi = logisticEdge(phi, phi_max + phi_margin, phi_max - phi_margin);
  } else {
    p_phi = 1.0;
  }

  return std::clamp(p_r * p_phi, 0.05, 0.95);
}

// ---------------------------------------------------------------------------
// BIRTH PROBABILITY
// ---------------------------------------------------------------------------

double BayesianExistencePredictor::computeBirthProbability(
  const ufil::type::Vector2 & pos)
{
  constexpr double dx = 0.5;

  // Numerical gradient of persistence probability
  const double grad_x =
    (computePersistenceProbability(pos + ufil::type::Vector2(dx, 0.0)) -
    computePersistenceProbability(pos - ufil::type::Vector2(dx, 0.0))) /
    (2.0 * dx);

  const double grad_y =
    (computePersistenceProbability(pos + ufil::type::Vector2(0.0, dx)) -
    computePersistenceProbability(pos - ufil::type::Vector2(0.0, dx))) /
    (2.0 * dx);

  const double grad_norm = std::hypot(grad_x, grad_y);

  return std::clamp(grad_norm * 0.3, 0.0, 0.5);
}


// ---------------------------------------------------------------------------
// OCCLUSION PROBABILITY (SIMPLE WEIGHTED HISTORY)
// ---------------------------------------------------------------------------

double BayesianExistencePredictor::computeOcclusionProbability(const Track & track)
{
  const auto & history = track.history();
  if (history.empty()) {
    return 0.0;
  }

  const int N = history.size();

  double weighted_sum = 0.0;
  double total_weight = 0.0;
  int age = 0;

  // Newest → oldest, weights = N, N-1, ..., 1
  for (auto it = history.rbegin(); it != history.rend(); ++it, ++age) {
    const bool occluded = it->second.occluded();
    const double weight = static_cast<double>(N - age);

    weighted_sum += weight * (occluded ? 1.0 : 0.0);
    total_weight += weight;
  }

  const double p_occ =
    (total_weight > 0.0) ? (weighted_sum / total_weight) : 0.0;

  return std::clamp(p_occ, 0.0, 0.95);
}


// ---------------------------------------------------------------------------
// FOV UPDATE
// ---------------------------------------------------------------------------

void BayesianExistencePredictor::setCurrentFOV(const ufil::type::SensorFOV & fov)
{
  current_fov_ = fov;
}

}  // namespace ufil_osn_tracker
