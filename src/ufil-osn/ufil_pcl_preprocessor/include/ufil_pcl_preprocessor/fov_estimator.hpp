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

#ifndef UFIL_PCL_PREPROCESSOR__FOV_ESTIMATOR_HPP_
#define UFIL_PCL_PREPROCESSOR__FOV_ESTIMATOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <numbers>  // listed with the C headers, cpplint does not know this C++20 header

#include <array>
#include <deque>

#include <ufil_object_tracking/types/sensor_fov.hpp>

#include "ufil_pcl_preprocessor/visibility_control.hpp"

namespace ufil_pcl_preprocessor
{

/// Estimates the sensor field of view from the extent of the observed points.
class UFIL_PCL_PREPROCESSOR_PUBLIC FOVEstimator
{
public:
  FOVEstimator() = default;
  explicit FOVEstimator(int num_bins);

  /**
   * @brief Add the estimate of a cloud given in the sensor frame to the history.
   */
  void update(const pcl::PointCloud<pcl::PointXYZ> & cloud);

  /**
   * @brief Averaged field of view over the history.
   */
  ufil::type::SensorFOV getFOV() const;

  /**
   * @brief Whether the most recent cloud contained enough points for an estimate.
   *
   * Clouds without a valid estimate contribute an empty field of view to the history.
   */
  bool lastEstimateValid() const;

  /// @throws std::invalid_argument if num_bins <= 0
  void setNumberOfBins(int num_bins);
  int getNumberOfBins() const;

  /// @throws std::invalid_argument if history_length <= 0
  void setHistoryLength(int history_length);
  int getHistoryLength() const;

  /// @throws std::invalid_argument if max_range <= 0
  void setMaxRange(float max_range);
  float getMaxRange() const;

  /// Minimum points required per bin to consider it valid.
  static constexpr int MIN_POINTS_PER_BIN = 500;

private:
  /// Half range of the azimuth covered by the bins.
  static constexpr float FOV_HALF_RANGE = std::numbers::pi_v<float>/ 2.0f;

  ufil::type::SensorFOV estimateFOV(const pcl::PointCloud<pcl::PointXYZ> & cloud);
  void updateAveragedFOV();

  int num_bins_ = 360;
  int history_length_ = 100;
  float max_range_ = 0.0f;
  bool last_estimate_valid_ = false;

  std::deque<ufil::type::SensorFOV> fov_history_;
  ufil::type::SensorFOV averaged_fov_;
  /// Rolling sum of r_min, r_max, phi_min, phi_max, theta_min, theta_max over the history.
  std::array<float, 6> fov_sum_{};
};

}  // namespace ufil_pcl_preprocessor

#endif  // UFIL_PCL_PREPROCESSOR__FOV_ESTIMATOR_HPP_
