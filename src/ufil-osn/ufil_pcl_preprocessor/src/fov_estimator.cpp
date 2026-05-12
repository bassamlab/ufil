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

#include "ufil_pcl_preprocessor/fov_estimator.hpp"

#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace ufil_pcl_preprocessor
{

FOVEstimator::FOVEstimator()
{
}

FOVEstimator::FOVEstimator(int num_bins)
: num_bins_(num_bins)
{
  if (num_bins <= 0) {
    throw std::invalid_argument("Number of bins must be greater than zero.");
  }
}

void FOVEstimator::setNumberOfBins(int num_bins)
{
  if (num_bins <= 0) {
    throw std::invalid_argument("Number of bins must be greater than zero.");
  }
  num_bins_ = num_bins;
}

int FOVEstimator::getNumberOfBins() const
{
  return num_bins_;
}

void FOVEstimator::setHistoryLength(int history_length)
{
  if (history_length <= 0) {
    throw std::invalid_argument("History length must be greater than zero.");
  }
  history_length_ = history_length;
}

int FOVEstimator::getHistoryLength() const
{
  return history_length_;
}

void FOVEstimator::setMaxRange(float max_range)
{
  if (max_range <= 0.0f) {
    throw std::invalid_argument("Max range must be greater than zero.");
  }
  max_range_ = max_range;
}

float FOVEstimator::getMaxRange() const
{
  return max_range_;
}

void FOVEstimator::update(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  ufil::type::SensorFOV fov = estimateFOV(cloud);

  // Maintain a rolling sum so averaging stays O(1) instead of re-scanning history.
  if (static_cast<int>(fov_history_.size()) >= history_length_) {
    const auto & oldest = fov_history_.front();
    fov_sum_[0] -= oldest.rMin();
    fov_sum_[1] -= oldest.rMax();
    fov_sum_[2] -= oldest.phiMin();
    fov_sum_[3] -= oldest.phiMax();
    fov_sum_[4] -= oldest.thetaMin();
    fov_sum_[5] -= oldest.thetaMax();
    fov_history_.pop_front();
  }

  fov_history_.push_back(fov);
  fov_sum_[0] += fov.rMin();
  fov_sum_[1] += fov.rMax();
  fov_sum_[2] += fov.phiMin();
  fov_sum_[3] += fov.phiMax();
  fov_sum_[4] += fov.thetaMin();
  fov_sum_[5] += fov.thetaMax();

  updateAveragedFOV();
}

ufil::type::SensorFOV FOVEstimator::getFOV() const
{
  return averaged_fov_;
}

void FOVEstimator::updateAveragedFOV()
{
  averaged_fov_ = ufil::type::SensorFOV();
  if (fov_history_.empty()) {
    return;
  }

  float history_size = static_cast<float>(fov_history_.size());
  averaged_fov_.rMin() = fov_sum_[0] / history_size;
  averaged_fov_.rMax() = fov_sum_[1] / history_size;
  averaged_fov_.phiMin() = fov_sum_[2] / history_size;
  averaged_fov_.phiMax() = fov_sum_[3] / history_size;
  averaged_fov_.thetaMin() = fov_sum_[4] / history_size;
  averaged_fov_.thetaMax() = fov_sum_[5] / history_size;
}

ufil::type::SensorFOV FOVEstimator::estimateFOV(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  float angle_min = -FOV_HALF_RANGE;
  float angle_max = FOV_HALF_RANGE;
  float angle_bin_width = (angle_max - angle_min) / num_bins_;

  struct BinStats
  {
    int count = 0;
    float r_min = std::numeric_limits<float>::max();
    float r_max = 0.0f;
    float phi_min = std::numeric_limits<float>::max();
    float phi_max = std::numeric_limits<float>::lowest();
    float theta_min = std::numeric_limits<float>::max();
    float theta_max = std::numeric_limits<float>::lowest();
  };

  std::vector<BinStats> bins(num_bins_);

  for (const auto & pt : *cloud) {
    const float r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    const float phi = std::atan2(pt.y, pt.x);
    const float theta = std::atan2(pt.z, r);

    int bin_index = static_cast<int>((phi - angle_min) / angle_bin_width);
    bin_index = std::clamp(bin_index, 0, num_bins_ - 1);

    auto & bin = bins[bin_index];
    ++bin.count;
    bin.r_min = std::min(bin.r_min, r);
    bin.r_max = std::max(bin.r_max, r);
    bin.phi_min = std::min(bin.phi_min, phi);
    bin.phi_max = std::max(bin.phi_max, phi);
    bin.theta_min = std::min(bin.theta_min, theta);
    bin.theta_max = std::max(bin.theta_max, theta);
  }

  float r_min = std::numeric_limits<float>::max();
  float r_max = 0.0f;
  float phi_min = std::numeric_limits<float>::max();
  float phi_max = std::numeric_limits<float>::lowest();
  float theta_min = std::numeric_limits<float>::max();
  float theta_max = std::numeric_limits<float>::lowest();
  bool has_valid_bin = false;

  for (const auto & bin : bins) {
    if (bin.count < MIN_POINTS_PER_BIN) {
      continue;
    }

    has_valid_bin = true;
    r_min = std::min(r_min, bin.r_min);
    r_max = std::max(r_max, bin.r_max);
    phi_min = std::min(phi_min, bin.phi_min);
    phi_max = std::max(phi_max, bin.phi_max);
    theta_min = std::min(theta_min, bin.theta_min);
    theta_max = std::max(theta_max, bin.theta_max);
  }

  if (!has_valid_bin) {
    return ufil::type::SensorFOV();
  }

  const int bin_index_max = static_cast<int>((phi_max - angle_min) / angle_bin_width);
  phi_max = angle_min + static_cast<float>(bin_index_max + 1) * angle_bin_width;
  const int bin_index_min = static_cast<int>((phi_min - angle_min) / angle_bin_width);
  phi_min = angle_min + static_cast<float>(bin_index_min) * angle_bin_width;

  ufil::type::SensorFOV fov;
  fov.rMin() = r_min;
  fov.rMax() = std::clamp(r_max, 0.0f, max_range_);
  fov.phiMin() = phi_min;
  fov.phiMax() = phi_max;
  fov.thetaMin() = theta_min;
  fov.thetaMax() = theta_max;
  return fov;
}

}  // namespace ufil_pcl_preprocessor
