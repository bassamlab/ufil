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

#include <deque>
#include <limits>
#include <array>
#include <vector>

#include <ufil_object_tracking/types/matrix.hpp>
#include <ufil_object_tracking/types/vector.hpp>
#include <ufil_object_tracking/types/sensor_fov.hpp>

#include "ufil_pcl_preprocessor/definitions.hpp"
#include "ufil_pcl_preprocessor/visibility_control.h"

namespace ufil_pcl_preprocessor
{

class OSN_PCL_PREPROCESSING_PUBLIC FOVEstimator
{
public:
  FOVEstimator();
  explicit FOVEstimator(int num_bins);
  ~FOVEstimator() = default;

  /**
   * @brief Update the field of view estimation from a point cloud
   * @param cloud Input point cloud to estimate FOV from
   */
  void update(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  // /**
  //  * @brief Set sensor position used for ray projection.
  //  * @param origin sensor position(x,y,z in meters).
  //  */
  // void setSensorOrigin(const Eigen::Vector3d & origin);

  /**
   * @brief Get the averaged field of view from the history
   * @return SensorFOV struct with averaged radial, azimuth, and elevation ranges
   */
  ufil::type::SensorFOV getFOV() const;

  /**
   * @brief Set the number of azimuthal bins for FOV estimation
   * @param num_bins Number of bins (must be > 0)
   * @throws std::invalid_argument if num_bins <= 0
   */
  void setNumberOfBins(int num_bins);

  /**
   * @brief Get the current number of bins
   * @return Number of bins
   */
  int getNumberOfBins() const;

  /**
   * @brief Set the history length for averaging FOV estimates
   * @param history_length Number of FOV estimates to maintain (must be > 0)
   * @throws std::invalid_argument if history_length <= 0
   */
  void setHistoryLength(int history_length);

  /**
   * @brief Get the current history length
   * @return History length
   */
  int getHistoryLength() const;

  /**
   * @brief Set the maximum range for FOV estimation
   * @param max_range Maximum range in meters
   */
  void setMaxRange(float max_range);

  /**
   * @brief Get the current maximum range
   * @return Maximum range in meters
   */
  float getMaxRange() const;

private:
  int num_bins_ = 360;  // Default: 1 degree per bin
  int history_length_ = 100;  // Default: no averaging
  std::deque<ufil::type::SensorFOV> fov_history_;
  ufil::type::SensorFOV averaged_fov_;
  std::array<float, 6> fov_sum_{};

  float max_range_ = 0.0f;

  /**
   * @brief Minimum points required per bin to consider it valid
   */
  static constexpr int MIN_POINTS_PER_BIN = 500;

  /**
   * @brief Half range of FOV for angular binning
   */
  static constexpr float FOV_HALF_RANGE = M_PI_2;

  /**
   * @brief Estimate FOV from a single point cloud
   * @param cloud Input point cloud
   * @return SensorFOV struct with estimated ranges
   */
  ufil::type::SensorFOV estimateFOV(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  /**
   * @brief Calculate and update averaged FOV from history
   */
  void updateAveragedFOV();
};

}  // namespace ufil_pcl_preprocessor

#endif  // UFIL_PCL_PREPROCESSOR__FOV_ESTIMATOR_HPP_
