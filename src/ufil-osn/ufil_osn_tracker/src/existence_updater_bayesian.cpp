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

#include "existence_updater_bayesian.hpp"

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <utility>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace ufil_osn_tracker
{
BayesianExistenceUpdater::BayesianExistenceUpdater(
  std::shared_ptr<BayesianExistencePredictor> predictor)
: predictor_(std::move(predictor))
{
}

void BayesianExistenceUpdater::update_parameter(
  const std::map<ufil::type::Id, Track> & /*tracks*/,
  const std::map<ufil::type::Id, ufil::type::measurement::Pose2DWithDimension3D> & associations,
  const std::set<ufil::type::measurement::Pose2DWithDimension3D> & unassociated_measurements)
{
  total_measurements_ = associations.size() + unassociated_measurements.size();
  associated_measurements_ = associations.size();

  double birth_prob_sum = 0.0;
  for (const auto & meas : unassociated_measurements) {
    birth_prob_sum += predictor_->computeBirthProbability(meas.position());
  }

  // Round expected births → integer number of birth measurements
  birth_measurements_ = birth_prob_sum;

  // Clutter rate update for the entire scene
  updateClutterRate(total_measurements_, associated_measurements_, birth_measurements_);
}

void BayesianExistenceUpdater::update(
  Track & track,
  const std::optional<ufil::type::measurement::Pose2DWithDimension3D> & /*measurement*/,
  const ufil::type::Timestamp & /*timestamp*/)
{
    // Get prior existence probabilities from predictor
  double prior_exist = track.currentExistenceProbability().existence();
  double prior_not_exist = track.currentExistenceProbability().nonExistence();

    // Detection probability given the current state & occlusion model
  double p_det = predictor_->computePersistenceProbability(track.currentState().position()) *
    (1.0 - predictor_->computeOcclusionProbability(track));
  p_det = std::clamp(p_det, 0.05, 0.95);

    // Count clutter (measurements not associated with any track and not birth)
  int num_clutter = total_measurements_ - associated_measurements_ - birth_measurements_;
  double p_clutter = computeClutterProbability(num_clutter);

    // --- Determine likelihood depending on measurement ---
  double p_meas_given_exist = 0.0;
  double p_meas_given_not_exist = 0.0;

  if (track.currentHistoryEntry().hasAssociatedMeasurement()) {
        // Track has associated measurement -> existence reinforced
    p_meas_given_exist = p_det;
    p_meas_given_not_exist = p_clutter;
  } else {
        // Reduce penalty if occluded
    double occ = predictor_->computeOcclusionProbability(track);
    double miss_penalty = std::max(0.4, p_det * (1 - occ));
    p_meas_given_exist = std::clamp(1.0 - miss_penalty, 0.05, 0.95);

        // Track has no associated measurement → existence weakened
    p_meas_given_not_exist = std::clamp(1.0 - p_clutter, 0.2, 0.8);
  }

    // Bayesian update
  double norm = 1.0 / (p_meas_given_exist * prior_exist + p_meas_given_not_exist * prior_not_exist);

  double posterior_exist = norm * p_meas_given_exist * prior_exist;
  double posterior_not_exist = norm * p_meas_given_not_exist * prior_not_exist;

    // Write back to track with clamping to avoid numerical issues
  constexpr double kEps = 1e-3;
  auto & cur = track.currentExistenceProbability();

  cur.existence() = std::clamp(posterior_exist, kEps, 1.0 - kEps);
  cur.nonExistence() = std::clamp(posterior_not_exist, kEps, 1.0 - kEps);

    // renormalize
  double sum = cur.existence() + cur.nonExistence();
  cur.existence() /= sum;
  cur.nonExistence() /= sum;
}

double BayesianExistenceUpdater::computeDetectionProbability(const Track & track)
{
  // Occlusion reduces detection probability
  double not_occluded = 1.0 - predictor_->computeOcclusionProbability(track);

  // Persistence: model probability the object still exists
  double persistence = predictor_->computePersistenceProbability(track.currentState().position());

  double mod_prob = persistence * not_occluded;

  // Measurement quality factor
  double meas_factor = 1.0;
  double track_factor = 1.0;

  if (track.currentHistoryEntry().hasAssociatedMeasurement()) {
    // Penalize classes with high "other" probability
    meas_factor = std::clamp(1.0 - track.currentClassification().other(),
                             0.1, 1.0);

    // Evaluate NIS-based track quality
    track_factor = computeTrackProbability(track);
  }

  return std::clamp(mod_prob * meas_factor * track_factor, 0.00, 1.0);
}

double BayesianExistenceUpdater::computeTrackProbability(const Track & track)
{
  // Use up to K recent entries
  int K = std::min<int>(15, track.history().size());

  if (K == 0) {
    return 0.0;
  }

  const int measurement_dim = Track::MeasurementType::Size;

  double sum_nis = 0.0;
  int count = 0;

  for (const auto &[timestamp, entry] : track.history()) {
    if (count == K) {
      break;
    }
    if (entry.hasNIS()) {
      sum_nis += *entry.nis();
      ++count;
    }
  }

  if (count == 0) {
    return 1.0;  // No NIS information available
  }

  double anis = sum_nis / count;
  int dof = K * measurement_dim;

  // Compute tail probability of NIS under chi^2 distribution
  boost::math::chi_squared_distribution<double> chi2(dof);
  double cdf = boost::math::cdf(chi2, anis * K);

  // Higher NIS → worse track → lower probability
  return std::clamp(1.0 - cdf, 0.0, 1.0);
}

void BayesianExistenceUpdater::updateClutterRate(
  int total_measurements,
  int num_associated,
  int num_births)
{
  int num_false = std::max(0, total_measurements - num_associated - num_births);
  double false_meas_double = static_cast<double>(num_false);

  // Exponential smoothing:
  //   λ_new = (1 - w) * new_value + w * old_value
  lambda_c_ = (1.0 - lambda_w_) * false_meas_double +
    lambda_w_ * lambda_c_;
}

double BayesianExistenceUpdater::computeClutterProbability(int m) const
{
  if (m < 0 || lambda_c_ <= 0.0) {
    return 0.0;
  }

  // Poisson CDF up to m:
  //   P(k ≤ m) = Σ exp(k*log(λ) - λ - log(k!))
  double p_c = 0.0;

  for (int k = 0; k <= m; ++k) {
    double logp =
      k * std::log(lambda_c_) -
      lambda_c_ -
      std::lgamma(k + 1);

    p_c += std::exp(logp);
  }

  return std::clamp(p_c, 0.00, 1.0);
}


}   // namespace ufil_osn_tracker
