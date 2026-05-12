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

#ifndef UFIL_PCL_PREPROCESSOR__DEPTH_BUFFER_HPP_
#define UFIL_PCL_PREPROCESSOR__DEPTH_BUFFER_HPP_

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>
#include <algorithm>

#include "ufil_pcl_preprocessor/definitions.hpp"
#include "ufil_object_tracking/types/occupancy_grid.hpp"

namespace ufil_pcl_preprocessor
{

class DepthBuffer
{
public:
  DepthBuffer()
  {
    setGridSize(grid_size_x_m_, grid_size_y_m_, grid_resolution_m_);
  }
  ~DepthBuffer() = default;

    // ----------------------------
    // Configuration
    // ----------------------------
  void setGridSize(double size_x_m, double size_y_m, double resolution_m)
  {
    grid_size_x_m_ = size_x_m;
    grid_size_y_m_ = size_y_m;
    grid_resolution_m_ = resolution_m;

    width_ = static_cast<int>(std::floor(grid_size_x_m_ / grid_resolution_m_));
    height_ = static_cast<int>(std::floor(grid_size_y_m_ / grid_resolution_m_));
    grid_.assign(width_ * height_, 0.0);
    persistence_.assign(width_ * height_, 0);
  }

  void setSensorOrigin(const Eigen::Vector3d & origin) {sensor_origin_ = origin;}

    /// decay_rate controls how quickly occupancy decays for fluctuating cells
  void setDecayRate(double decay_rate) {decay_rate_ = decay_rate;}

  void setMinRange(double rmin) {min_range_m_ = rmin;}
  void setMaxRange(double rmax) {max_range_m_ = rmax;}

    // ----------------------------
    // Public API
    // ----------------------------
  void update(const pcl::PointCloud<pcl::PointXYZ> & cloud)
  {
        // create a temporary measurement grid
    std::vector<double> meas_grid(width_ * height_, 0.0);

        // integrate each point
    for (const auto & pt : cloud) {
      const Eigen::Vector3d vec(pt.x, pt.y, pt.z);
      const double r = vec.norm();
      if (!std::isfinite(r) || r < min_range_m_ || r > max_range_m_) {continue;}

            // project onto ground plane
      Eigen::Vector3d dir = vec - sensor_origin_;
      double t = -sensor_origin_.z() / dir.z();
      if (t <= 0) {continue;}
      Eigen::Vector3d ground = sensor_origin_ + t * dir;

      int gx = static_cast<int>((ground.x() + grid_size_x_m_ * 0.5) / grid_resolution_m_);
      int gy = static_cast<int>((ground.y() + grid_size_y_m_ * 0.5) / grid_resolution_m_);

      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {continue;}

      meas_grid[gy * width_ + gx] = 1.0;       // occupied
    }

    // -----------------------------
    // Update main grid with temporal smoothing
    // -----------------------------
    for (size_t i = 0; i < grid_.size(); ++i) {
      // cells consistently observed as free decay faster
      // cells consistently occupied stay high
      if (meas_grid[i] > 0.5) {
        // cell is observed occupied
        persistence_[i] += 1;         // count consecutive occupied frames
        double alpha = 0.1;           // base growth rate
        double gain = 1.0 - std::exp(-persistence_[i] * alpha);
        grid_[i] += gain * (1.0 - grid_[i]);         // ramp toward 1 non-linearly
      } else {
                // cell is free
        persistence_[i] = 0;
        double beta = 0.3;         // faster decay for free cells
        grid_[i] *= std::pow(decay_rate_, beta);         // decay faster for free cells
      }
      // grid_[i] = grid_[i] * decay_rate_ +  meas_grid[i] * (1.0 - decay_rate_);
      grid_[i] = std::clamp(grid_[i], 0.0, 1.0);
    }
  }

  ufil::type::OccupancyGrid getOccupancyGrid() const
  {
    ufil::type::OccupancyGrid g;
    g.width() = width_;
    g.height() = height_;
    g.resolution() = grid_resolution_m_;
    g.originX() = -grid_size_x_m_ * 0.5;
    g.originY() = -grid_size_y_m_ * 0.5;
    g.originZ() = 0.15;

    g.data().resize(width_ * height_);
    for (size_t i = 0; i < grid_.size(); ++i) {
      g.data()[i] = static_cast<int8_t>(std::round(std::clamp(grid_[i], 0.0, 1.0) * 100.0));
    }
    return g;
  }

  void reset()
  {
    std::fill(grid_.begin(), grid_.end(), 0.0);
  }

private:
  Eigen::Vector3d sensor_origin_{0.2, -0.2, 5.9};

  double grid_size_x_m_ = 80.0;
  double grid_size_y_m_ = 80.0;
  double grid_resolution_m_ = 0.4;
  int width_ = 200;
  int height_ = 200;

  double min_range_m_ = 5.0;
  double max_range_m_ = 44.0;

  double decay_rate_ = 0.95;   // temporal smoothing factor

  std::vector<int> persistence_;        // number of consecutive occupied observations
  std::vector<double> grid_;   // main occupancy grid [0..1]
};

}  // namespace ufil_pcl_preprocessor

#endif  // UFIL_PCL_PREPROCESSOR__DEPTH_BUFFER_HPP_
