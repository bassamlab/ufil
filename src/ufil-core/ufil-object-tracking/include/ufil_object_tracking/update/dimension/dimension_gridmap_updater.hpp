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

#ifndef UFIL_OBJECT_TRACKING__UPDATE__DIMENSION__DIMENSION_GRIDMAP_UPDATER_HPP_
#define UFIL_OBJECT_TRACKING__UPDATE__DIMENSION__DIMENSION_GRIDMAP_UPDATER_HPP_

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <numeric>
#include <optional>
#include <vector>

#include "ufil_object_tracking/models/measurement/linearized_measurement_model.hpp"
#include "ufil_object_tracking/update/dimension/dimension_updater.hpp"
namespace ufil
{
namespace update
{
namespace dimension
{

template<typename T>
class DimensionGridmapUpdater : public DimensionUpdater<T>
{
protected:
  using TrackType = typename DimensionUpdater<T>::TrackType;
  using MeasurementType = typename DimensionUpdater<T>::MeasurementType;
  using DimensionType = typename DimensionUpdater<T>::DimensionType;

public:
  DimensionGridmapUpdater(
    const ufil::type::Vector3 delta_d = {0.1, 0.1, 0.1}, const ufil::type::Scalar alpha = 0.95,
    const ufil::type::Scalar P_min = 0.1, const ufil::type::Scalar P_ref = 0.5,
    const ufil::type::Scalar P_max = 0.9)
  : alpha_(alpha), P_min_(P_min), P_ref_(P_ref), P_max_(P_max), delta_d_(delta_d)
  {
    // Check if parameter values are valid and throw exceptions if not
    validate_parameter();

    // Precompute prior log-odds for efficiency
    prior_log_odds_ = std::log(P_ref / (1.0 - P_ref));
  }

  void update(
    const TrackType & /*track*/, const std::optional<MeasurementType> & measurement,
    const ufil::type::Timestamp & /*timestamp*/, DimensionType & resulting_dimension) override
  {
    if (!measurement) {
      return;
    }
    // Validate measurement and throw exceptions if invalid
    validate_measurement(*measurement);

    for (size_t i = 0; i < measurement->dimension().numberOfElements(); ++i) {
      // Read input parameters
      const ufil::type::Scalar measured = measurement->dimension().dimensionVector()(i);
      const ufil::type::Scalar sig_measured = std::sqrt(measurement->dimension().covariance()(i,
              i));
      const ufil::type::Scalar delta_d = delta_d_(i);

      // Get working buffer for current dimension
      std::vector<ufil::type::DimensionGridmapCell> & working_buffer =
            resulting_dimension.dimensionGridmapBuffer().at(i);

      // Update probabilities in the gridmap based on measurement
      update_probability(working_buffer, measured, sig_measured, delta_d);

      // Normalize probabilities to ensure they sum to 1
      normalize_probabilities(working_buffer, measured, sig_measured);

      // Compute estimate and variance
      const ufil::type::Scalar estimate = std::accumulate(working_buffer.begin(),
              working_buffer.end(), 0.0,
          [](ufil::type::Scalar result, const auto & cell) {
            return result + cell.probability * cell.position;
      });

      const ufil::type::Scalar variance = std::accumulate(working_buffer.begin(),
              working_buffer.end(), 0.0,
          [estimate](ufil::type::Scalar result, const auto & cell) {
            const ufil::type::Scalar diff = cell.position - estimate;
            return result + cell.probability * diff * diff;
      });

      // Write back estimate and variance to resulting dimension
      resulting_dimension.dimensionVector()(i) = std::max(estimate, 0.0);
      resulting_dimension.covariance()(i, i) = std::max(variance, 1e-6);
    }

    // Final validation of resulting dimension to catch any potential issues
    validate_result(resulting_dimension);
  }

private:
  const ufil::type::Scalar alpha_;
  const ufil::type::Scalar P_min_;
  const ufil::type::Scalar P_ref_;
  const ufil::type::Scalar P_max_;
  const ufil::type::Vector3 delta_d_;

  ufil::type::Scalar prior_log_odds_;

  inline void validate_parameter()
  {
    if (delta_d_.x() <= 0.0 || delta_d_.y() <= 0.0 || delta_d_.z() <= 0.0) {
      throw std::invalid_argument("Delta_d must be positive in all dimensions");
    }

    if (alpha_ < 0.0 || alpha_ > 1.0) {
      throw std::invalid_argument("Alpha must be in the range [0, 1]");
    }

    if (P_min_ <= 0.0 || P_min_ >= 1.0 || P_ref_ <= 0.0 || P_ref_ >= 1.0 || P_max_ <= 0.0 ||
      P_max_ >= 1.0)
    {
      throw std::invalid_argument("P_min, P_ref, and P_max must be in the range (0, 1)");
    }

    if (!(P_min_ < P_ref_ && P_ref_ < P_max_)) {
      throw std::invalid_argument("Must hold that P_min < P_ref < P_max");
    }
  }

  inline void validate_measurement(const MeasurementType & measurement) const
  {
    for (size_t i = 0; i < measurement.dimension().numberOfElements(); ++i) {
      const ufil::type::Scalar measured = measurement.dimension().dimensionVector()(i);
      const ufil::type::Scalar sig_measured = std::sqrt(measurement.dimension().covariance()(i, i));

      if (std::isnan(measured) || std::isnan(sig_measured)) {
              throw std::runtime_error("NaN measurement received in DimensionGridmapUpdater");
      }

      if (sig_measured <= 0.0) {
              throw std::runtime_error(
                "Non-positive measurement variance received in DimensionGridmapUpdater");
      }
    }
  }

  inline void validate_result(const DimensionType & result) const
  {
    for (size_t i = 0; i < result.numberOfElements(); ++i) {
      const ufil::type::Scalar estimate = result.dimensionVector()(i);
      const ufil::type::Scalar variance = result.covariance()(i, i);

      if (std::isnan(estimate) || std::isnan(variance)) {
        throw std::runtime_error("NaN estimate or variance computed in DimensionGridmapUpdater");
      }

      if (estimate < 0.0) {
        throw std::runtime_error("Negative estimate computed in DimensionGridmapUpdater");
      }

      if (variance <= 0.0) {
        throw std::runtime_error("Non-positive variance computed in DimensionGridmapUpdater");
      }
    }
  }

  inline ufil::type::Scalar conditional_probability(
    const ufil::type::Scalar x, const ufil::type::Scalar d, const ufil::type::Scalar sig) const
  {
    ufil::type::Scalar p_d = 0.0;

    if (x < d) {
      p_d = -2.0 * (P_max_ - P_ref_) / (1.0 + std::exp((d - x) / sig)) + P_max_;
    } else if (x == d) {
      p_d = P_ref_;
    } else if (x > d) {
      p_d = -2.0 * (P_ref_ - P_min_) / (1.0 + std::exp((d - x) / sig)) + 2 * P_ref_ - P_min_;
    }

    return p_d;
  }

  inline void update_probability(
    std::vector<ufil::type::DimensionGridmapCell> & working_buffer,
    const ufil::type::Scalar d_measured,
    const ufil::type::Scalar sig_measured, const ufil::type::Scalar delta_d)
  {
    // Initialize last_cd at x = 0 to handle truncation
    const ufil::type::Scalar p0 = std::clamp(
      conditional_probability(0.0, d_measured, sig_measured),
      1e-6, 1.0 - 1e-6);

    const ufil::type::Scalar L0 = std::log(p0 / (1 - p0)) + alpha_ * prior_log_odds_ -
      prior_log_odds_;
    const ufil::type::Scalar exp_L0 = std::exp(L0);
    ufil::type::Scalar last_cd = exp_L0 / (1.0 + exp_L0);

    constexpr size_t kMaxCells = 1000;
    for (size_t i = 0; i < kMaxCells; i++) {
      if (i >= working_buffer.size()) {
        const ufil::type::Scalar initial_log_odds = prior_log_odds_;
        const ufil::type::Scalar cell_position = (i + 0.5) * delta_d;
        working_buffer.push_back({cell_position, initial_log_odds, 0.0});
      }

      auto & cell = working_buffer[i];

      // Reset log_odds for distant cells before applying update
      // if (std::abs(cell.position - d_measured) > 5.0 * sig_measured) {
      //     cell.log_odds = prior_log_odds_;
      // }

      const ufil::type::Scalar p_cell_d = conditional_probability(cell.position, d_measured,
              sig_measured);

      static constexpr ufil::type::Scalar kEps = 1e-6;
      const ufil::type::Scalar p = std::clamp(p_cell_d, kEps, 1.0 - kEps);
      cell.log_odds = std::log(p / (1 - p)) + alpha_ * cell.log_odds - prior_log_odds_;

      const ufil::type::Scalar exp_L = std::exp(cell.log_odds);
      ufil::type::Scalar cd = exp_L / (1.0 + exp_L);
      cd = std::min(cd, last_cd);
      const ufil::type::Scalar delta = last_cd - cd;
      cell.probability = (delta > 1e-12) ? delta : 0.0;

      last_cd = cd;
    }
  }

  void normalize_probabilities(
    std::vector<ufil::type::DimensionGridmapCell> & buffer,
    const ufil::type::Scalar measured, const ufil::type::Scalar sig_measured) const
  {
    ufil::type::Scalar sum_p = std::accumulate(
        buffer.begin(), buffer.end(), 0.0,
      [](ufil::type::Scalar s, const auto & cell) {return s + cell.probability;});

    if (sum_p > 1e-8) {
      for (auto & cell : buffer) {
        cell.probability /= sum_p;
      }
    } else {
        // instead: put all mass near measurement
      for (auto & cell : buffer) {
        double dist = std::abs(cell.position - measured);
        cell.probability = std::exp(-dist * dist / (2 * sig_measured * sig_measured));
      }
        // renormalize
      sum_p = std::accumulate(
          buffer.begin(), buffer.end(), 0.0,
        [](ufil::type::Scalar s, const auto & cell) {return s + cell.probability;});
      if (sum_p > 1e-8) {
        for (auto & cell : buffer) {
          cell.probability /= sum_p;
        }
      }
    }
  }
};
}  // namespace dimension
}  // namespace update
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__UPDATE__DIMENSION__DIMENSION_GRIDMAP_UPDATER_HPP_
