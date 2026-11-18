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

#include "ufil_pcl_preprocessor/depth_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace ufil_pcl_preprocessor
{

namespace
{

/// Growth rate of cells observed occupied in consecutive updates.
constexpr double kOccupiedGrowthRate = 0.1;
/// Exponent applied to the decay rate for free cells.
constexpr double kFreeDecayExponent = 0.3;
/// Height at which the grid is published [m].
constexpr double kGridHeight = 0.15;

}  // namespace

DepthBuffer::DepthBuffer()
: DepthBuffer(OcclusionGridConfig{})
{
}

DepthBuffer::DepthBuffer(const OcclusionGridConfig & config)
: config_(config)
{
  resizeGrid();
}

void DepthBuffer::configure(const OcclusionGridConfig & config)
{
  const bool geometry_changed =
    config.resolution != config_.resolution ||
    config.size_x != config_.size_x ||
    config.size_y != config_.size_y;
  config_ = config;
  if (geometry_changed) {
    resizeGrid();
  }
}

void DepthBuffer::setSensorOrigin(const Eigen::Vector3d & origin)
{
  sensor_origin_ = origin;
}

void DepthBuffer::update(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  std::vector<bool> observed(grid_.size(), false);

  for (const auto & pt : cloud) {
    const Eigen::Vector3d point(pt.x, pt.y, pt.z);
    const double range = point.norm();
    if (!std::isfinite(range) || range < config_.min_range || range > config_.max_range) {
      continue;
    }

    // Project along the sensor ray onto the ground plane.
    const Eigen::Vector3d direction = point - sensor_origin_;
    const double t = -sensor_origin_.z() / direction.z();
    if (!(t > 0.0)) {
      continue;
    }
    const Eigen::Vector3d ground = sensor_origin_ + t * direction;

    const int gx = static_cast<int>((ground.x() + config_.size_x * 0.5) / config_.resolution);
    const int gy = static_cast<int>((ground.y() + config_.size_y * 0.5) / config_.resolution);
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
      continue;
    }
    observed[static_cast<std::size_t>(gy) * width_ + gx] = true;
  }

  const double free_decay = std::pow(config_.decay_rate, kFreeDecayExponent);
  for (std::size_t i = 0; i < grid_.size(); ++i) {
    if (observed[i]) {
      // Ramp towards 1, faster the longer the cell is observed consecutively.
      ++persistence_[i];
      const double gain = 1.0 - std::exp(-persistence_[i] * kOccupiedGrowthRate);
      grid_[i] += gain * (1.0 - grid_[i]);
    } else {
      persistence_[i] = 0;
      grid_[i] *= free_decay;
    }
    grid_[i] = std::clamp(grid_[i], 0.0, 1.0);
  }
}

ufil::type::OccupancyGrid DepthBuffer::getOccupancyGrid() const
{
  ufil::type::OccupancyGrid grid;
  grid.width() = width_;
  grid.height() = height_;
  grid.resolution() = config_.resolution;
  grid.originX() = -config_.size_x * 0.5;
  grid.originY() = -config_.size_y * 0.5;
  grid.originZ() = kGridHeight;

  grid.data().resize(grid_.size());
  for (std::size_t i = 0; i < grid_.size(); ++i) {
    grid.data()[i] = static_cast<std::int8_t>(std::round(grid_[i] * 100.0));
  }
  return grid;
}

void DepthBuffer::reset()
{
  std::fill(grid_.begin(), grid_.end(), 0.0);
  std::fill(persistence_.begin(), persistence_.end(), 0);
}

void DepthBuffer::resizeGrid()
{
  width_ = static_cast<int>(std::floor(config_.size_x / config_.resolution));
  height_ = static_cast<int>(std::floor(config_.size_y / config_.resolution));
  const auto cells = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  grid_.assign(cells, 0.0);
  persistence_.assign(cells, 0);
}

}  // namespace ufil_pcl_preprocessor
