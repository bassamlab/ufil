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

#ifndef EXISTENCE_UPDATER_BAYESIAN_HPP_
#define EXISTENCE_UPDATER_BAYESIAN_HPP_

#include "ufil_osn_tracker/existence_predictor_bayesian.hpp"

#include <memory>
#include <map>
#include <set>

#include <ufil_osn_tracker/definitions.hpp>
#include <ufil_object_tracking/update/updater.hpp>

namespace ufil_osn_tracker
{
class BayesianExistenceUpdater : public ufil::update::Updater<Track>
{
public:
  explicit BayesianExistenceUpdater(std::shared_ptr<BayesianExistencePredictor> predictor);

  void update(
    Track & track,
    const std::optional<ufil::type::measurement::Pose2DWithDimension3D> & measurement,
    const ufil::type::Timestamp & timestamp) override;

  void update_parameter(
    const std::map<ufil::type::Id, Track> & tracks,
    const std::map<ufil::type::Id, ufil::type::measurement::Pose2DWithDimension3D> & associations,
    const std::set<ufil::type::measurement::Pose2DWithDimension3D> & unassociated_measurements)
  override;

private:
  BayesianExistencePredictor::SharedPtr predictor_;

  double computeDetectionProbability(const Track & track);  // Detection probability
  double computeTrackProbability(const Track & track);
  void updateClutterRate(
    int total_measurements,
    int num_associated, int num_births);  // Update clutter rate with new observation
  double computeClutterProbability(int m) const;  // Compute Poisson clutter probability

  // Constants or tunable values
  double lambda_c_ = 0.1;  // clutter rate
  double lambda_w_ = 0.9;  // weight factor

  int total_measurements_ = 0;
  int associated_measurements_ = 0;
  int birth_measurements_ = 0;
};
}  // namespace ufil_osn_tracker
#endif  // EXISTENCE_UPDATER_BAYESIAN_HPP_
