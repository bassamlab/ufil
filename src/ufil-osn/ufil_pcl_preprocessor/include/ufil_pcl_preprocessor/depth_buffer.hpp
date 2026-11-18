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

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include <ufil_object_tracking/types/occupancy_grid.hpp>

#include "ufil_pcl_preprocessor/config.hpp"
#include "ufil_pcl_preprocessor/visibility_control.hpp"

namespace ufil_pcl_preprocessor
{

/**
 * @brief Ground-plane occupancy grid of the regions occluded by static scene content.
 *
 * Every point is projected along its sensor ray onto the ground plane (z = 0). Cells hit
 * repeatedly ramp up towards fully occupied, cells that are not hit decay.
 */
class UFIL_PCL_PREPROCESSOR_PUBLIC DepthBuffer
{
public:
  DepthBuffer();
  explicit DepthBuffer(const OcclusionGridConfig & config);

  /// Apply a new configuration. The grid content is reset if its geometry changes.
  void configure(const OcclusionGridConfig & config);

  /// Sensor position in the frame of the points passed to update().
  void setSensorOrigin(const Eigen::Vector3d & origin);

  void update(const pcl::PointCloud<pcl::PointXYZ> & cloud);

  ufil::type::OccupancyGrid getOccupancyGrid() const;

  void reset();

private:
  void resizeGrid();

  OcclusionGridConfig config_;
  Eigen::Vector3d sensor_origin_{0.0, 0.0, 0.0};
  int width_ = 0;
  int height_ = 0;
  /// Number of consecutive updates a cell was observed occupied.
  std::vector<int> persistence_;
  /// Occupancy in [0, 1].
  std::vector<double> grid_;
};

}  // namespace ufil_pcl_preprocessor

#endif  // UFIL_PCL_PREPROCESSOR__DEPTH_BUFFER_HPP_
