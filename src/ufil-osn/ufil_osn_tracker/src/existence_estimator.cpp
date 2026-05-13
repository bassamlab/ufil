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

#include <ranges>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <map>
#include <string>

#include "existence_estimator.hpp"

namespace ufil_osn_tracker
{

double calculate_std(const std::vector<double> & data)
{
  double sum = 0.0;
  double mean, standardDeviation = 0.0;

  for (double value : data) {
    sum += value;
  }
  mean = sum / static_cast<double>(data.size());

  for (double value : data) {
    standardDeviation += std::pow(value - mean, 2);
  }

  return std::sqrt(standardDeviation / static_cast<double>(data.size()));
}

void ExistenceEstimator::update(
  Track & track,
  const std::optional<ufil::type::measurement::Pose2DWithDimension3D> & /*measurement*/,
  const ufil::type::Timestamp & timestamp)
{
  const std::map<std::string, double> weights{
    {"association", 0.3}, {"velocity", 0.1}, {"yaw_rate", 0.1}, {"age", 0.3}, {"aspect_ratio", 0.1},
    {"volume", 0.1}
  };

  const double vel_threshold = 2.0;       // m/s variation
  const double yaw_rate_threshold = 1.0;  // rad/s variation
  const double age_k = 0.4;
  const double age_offset = 5;
  const double aspect_ratio_threshold = 1.0;
  const double volume_threshold = 1.0;
  const double decay_factor = 0.08;

  // auto& history_entry = track.currentHistoryEntry();

  const size_t N = track.history().size();
  std::map<std::string, double> scores;

  double age = ufil::to_seconds(timestamp - track.creationTime());
  scores["age"] = 1.0 / (1.0 + std::exp(-age_k * (age - age_offset)));

  size_t number_of_associations = 0;
  for (const auto & entry : track.history()) {
    if (entry.second.hasAssociatedMeasurement()) {
      number_of_associations++;
    }
  }
  scores["association"] = static_cast<double>(number_of_associations) / static_cast<double>(N);

  std::vector<double> velocity_mag;
  for (const auto & entry : track.history()) {
    velocity_mag.push_back(entry.second.state().velocity().norm());
  }
  double vel_std = calculate_std(velocity_mag);
  scores["velocity"] = 1.0 - std::min(1.0, vel_std / vel_threshold);

  std::vector<double> yaw_rates;
  for (const auto & entry : track.history()) {
    yaw_rates.push_back(entry.second.state().yawRate());
  }
  double yaw_rate_std = calculate_std(yaw_rates);
  scores["yaw_rate"] = 1.0 - std::min(1.0, yaw_rate_std / yaw_rate_threshold);

  std::vector<double> volumes;
  std::vector<double> aspect_ratios;
  for (const auto & entry : track.history()) {
    const auto & dimension = entry.second.dimension();
    volumes.push_back(dimension.length() * dimension.width() * dimension.height());
    aspect_ratios.push_back((dimension.length() >
      0) ? dimension.width() / dimension.length() : 0.0);
  }
  double volumes_std = calculate_std(volumes);
  double aspect_ratios_std = calculate_std(aspect_ratios);
  scores["aspect_ratio"] = 1.0 - std::min(1.0, (aspect_ratios_std) / aspect_ratio_threshold);
  scores["volume"] = 1.0 - std::min(1.0, (volumes_std) / volume_threshold);


  double probability = 0.0;
  for (const auto & score : scores) {
    if (!weights.contains(score.first)) {
      throw std::runtime_error("Missing weight in existence probability estimator.");
    }
    double weight = weights.at(score.first);
    double value = score.second;

    probability += weight * value;
  }


  size_t consecutive_misses = 0;
  std::ranges::reverse_view rhistory {track.history()};
  for (const auto & entry : rhistory) {
    if (!entry.second.hasAssociatedMeasurement()) {
      consecutive_misses++;
    } else {
      break;
    }
  }
  probability *= 1.0 - std::min(1.0, consecutive_misses * decay_factor);

  size_t history_length = track.history().size();
  probability *= std::min(1.0, history_length * decay_factor);

  probability = std::clamp(probability, 0.0, 1.0);

  track.currentExistenceProbability().existence() = probability;

  // std::cout << ufil::hashId<uint32_t>(track.uuid()) << ": " <<
  // track.currentExistenceProbability().existence() << std::endl;
  // for (const auto & [key, value] : scores) {
  // std::cout << key << ": " << value << std::endl;
  // }
}

}  // namespace ufil_osn_tracker
