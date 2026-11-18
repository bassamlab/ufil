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

#ifndef UFIL_OBJECT_TRACKING__UPDATE__PROBABILITY__EXISTENCE_ESTIMATOR_HPP_
#define UFIL_OBJECT_TRACKING__UPDATE__PROBABILITY__EXISTENCE_ESTIMATOR_HPP_

#include <ranges>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "ufil_object_tracking/update/probability/existence_updater.hpp"

namespace ufil
{
namespace update
{
namespace probability
{
namespace detail
{

inline double calculate_std(const std::vector<double> & data)
{
  if (data.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (double value : data) {
    sum += value;
  }

  const double mean = sum / static_cast<double>(data.size());
  double variance = 0.0;
  for (double value : data) {
    variance += std::pow(value - mean, 2.0);
  }

  return std::sqrt(variance / static_cast<double>(data.size()));
}

}  // namespace detail

template<typename T>
class ExistenceEstimator : public ExistenceUpdater<T>
{
protected:
  using Base = ExistenceUpdater<T>;
  using TrackType = typename Base::TrackType;
  using MeasurementType = typename Base::MeasurementType;
  using ExistenceProbabilityType = typename Base::ExistenceProbabilityType;
  using StateType = typename TrackType::StateType;
  using DimensionType = typename TrackType::DimensionType;

public:
  using Base::update;

  void update(
    const TrackType & track, const std::optional<MeasurementType> & /*measurement*/,
    const ufil::type::Timestamp & timestamp,
    ExistenceProbabilityType & resulting_existence) override
  {
    const size_t history_length = track.history().size();
    if (history_length == 0U) {
      return;
    }

    constexpr double association_weight = 0.3;
    constexpr double velocity_weight = 0.1;
    constexpr double yaw_rate_weight = 0.1;
    constexpr double age_weight = 0.3;
    constexpr double aspect_ratio_weight = 0.1;
    constexpr double volume_weight = 0.1;

    constexpr double velocity_threshold = 2.0;
    constexpr double yaw_rate_threshold = 1.0;
    constexpr double age_k = 0.4;
    constexpr double age_offset = 5.0;
    constexpr double aspect_ratio_threshold = 1.0;
    constexpr double volume_threshold = 1.0;
    constexpr double decay_factor = 0.1;

    double weighted_probability = 0.0;
    double active_weight_sum = 0.0;

    auto add_score = [&weighted_probability, &active_weight_sum](const double weight,
      const std::optional<double> & score) {
        if (!score.has_value()) {
          return;
        }
        weighted_probability += weight * score.value();
        active_weight_sum += weight;
      };

    add_score(association_weight, compute_association_score(track));
    add_score(velocity_weight, compute_velocity_score(track, velocity_threshold));
    add_score(yaw_rate_weight, compute_yaw_rate_score(track, yaw_rate_threshold));
    add_score(age_weight, compute_age_score(track, timestamp, age_k, age_offset));
    add_score(aspect_ratio_weight, compute_aspect_ratio_score(track, aspect_ratio_threshold));
    add_score(volume_weight, compute_volume_score(track, volume_threshold));

    if (active_weight_sum <= 0.0) {
      resulting_existence.existence() = 0.0;
      return;
    }

    double probability = weighted_probability / active_weight_sum;
    probability *= 1.0 - std::min(1.0, static_cast<double>(compute_consecutive_misses(track)) *
      decay_factor);
    probability *= std::min(1.0, static_cast<double>(history_length) * decay_factor);
    probability = std::clamp(probability, 0.0, 1.0);

    resulting_existence.existence() = probability;
  }

private:
  static std::optional<double> compute_association_score(const TrackType & track)
  {
    const size_t history_length = track.history().size();
    if (history_length == 0U) {
      return std::nullopt;
    }

    size_t number_of_associations = 0U;
    for (const auto & entry : track.history()) {
      if (entry.second.hasAssociatedMeasurement()) {
        number_of_associations++;
      }
    }

    return static_cast<double>(number_of_associations) / static_cast<double>(history_length);
  }

  static std::optional<double> compute_age_score(
    const TrackType & track, const ufil::type::Timestamp & timestamp,
    const double age_k, const double age_offset)
  {
    const double age = ufil::to_seconds(timestamp - track.creationTime());
    return 1.0 / (1.0 + std::exp(-age_k * (age - age_offset)));
  }

  static std::optional<double> compute_velocity_score(
    const TrackType & track, const double velocity_threshold)
  {
    if constexpr (requires(const StateType & state) {state.velocity();}) {
      std::vector<double> velocity_magnitudes;
      velocity_magnitudes.reserve(track.history().size());

      for (const auto & entry : track.history()) {
        velocity_magnitudes.push_back(entry.second.state().velocity().norm());
      }

      const double velocity_std = detail::calculate_std(velocity_magnitudes);
      return 1.0 - std::min(1.0, velocity_std / velocity_threshold);
    } else {
      return std::nullopt;
    }
  }

  static std::optional<double> compute_yaw_rate_score(
    const TrackType & track, const double yaw_rate_threshold)
  {
    if constexpr (requires(const StateType & state) {state.yawRate();}) {
      std::vector<double> yaw_rates;
      yaw_rates.reserve(track.history().size());

      for (const auto & entry : track.history()) {
        yaw_rates.push_back(entry.second.state().yawRate());
      }

      const double yaw_rate_std = detail::calculate_std(yaw_rates);
      return 1.0 - std::min(1.0, yaw_rate_std / yaw_rate_threshold);
    } else {
      return std::nullopt;
    }
  }

  static std::optional<double> compute_aspect_ratio_score(
    const TrackType & track, const double aspect_ratio_threshold)
  {
    if constexpr (requires(const DimensionType & dimension) {
      dimension.length();
      dimension.width();
          })
    {
      std::vector<double> aspect_ratios;
      aspect_ratios.reserve(track.history().size());

      for (const auto & entry : track.history()) {
        const auto & dimension = entry.second.dimension();
        aspect_ratios.push_back(
          dimension.length() > 0.0 ? dimension.width() / dimension.length() : 0.0);
      }

      const double aspect_ratio_std = detail::calculate_std(aspect_ratios);
      return 1.0 - std::min(1.0, aspect_ratio_std / aspect_ratio_threshold);
    } else {
      return std::nullopt;
    }
  }

  static std::optional<double> compute_volume_score(
    const TrackType & track, const double volume_threshold)
  {
    if constexpr (requires(const DimensionType & dimension) {
      dimension.length();
      dimension.width();
      dimension.height();
          })
    {
      std::vector<double> volumes;
      volumes.reserve(track.history().size());

      for (const auto & entry : track.history()) {
        const auto & dimension = entry.second.dimension();
        volumes.push_back(dimension.length() * dimension.width() * dimension.height());
      }

      const double volume_std = detail::calculate_std(volumes);
      return 1.0 - std::min(1.0, volume_std / volume_threshold);
    } else {
      return std::nullopt;
    }
  }

  static size_t compute_consecutive_misses(const TrackType & track)
  {
    size_t consecutive_misses = 0U;
    std::ranges::reverse_view reverse_history {track.history()};
    for (const auto & entry : reverse_history) {
      if (!entry.second.hasAssociatedMeasurement()) {
        consecutive_misses++;
      } else {
        break;
      }
    }
    return consecutive_misses;
  }
};

}  // namespace probability
}  // namespace update
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__UPDATE__PROBABILITY__EXISTENCE_ESTIMATOR_HPP_
