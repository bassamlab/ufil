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

#ifndef UFIL_PCL_PREPROCESSOR__UFIL_PCL_PREPROCESSOR_HPP_
#define UFIL_PCL_PREPROCESSOR__UFIL_PCL_PREPROCESSOR_HPP_

#include <Eigen/Geometry>
#include <pcl/PCLPointCloud2.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include <ufil_object_tracking/types/occupancy_grid.hpp>
#include <ufil_object_tracking/types/sensor_fov.hpp>

#include "ufil_pcl_preprocessor/clipped_area.hpp"
#include "ufil_pcl_preprocessor/config.hpp"
#include "ufil_pcl_preprocessor/depth_buffer.hpp"
#include "ufil_pcl_preprocessor/fov_estimator.hpp"
#include "ufil_pcl_preprocessor/visibility_control.hpp"

namespace ufil_pcl_preprocessor
{

/// Where the points of a single cloud ended up. Each input point is counted exactly once.
struct FilterStatistics
{
  std::size_t input_points = 0;
  /// Outside [z_min, z_max] or not finite.
  std::size_t out_of_height_points = 0;
  /// Inside a clipped area; these feed the occlusion grid.
  std::size_t static_points = 0;
  /// Removed by the radius outlier filter.
  std::size_t outlier_points = 0;
  /// Remaining dynamic points before voxel downsampling.
  std::size_t dynamic_points = 0;
  /// Points in the output cloud after voxel downsampling.
  std::size_t output_points = 0;
};

/// Snapshot of the preprocessor state, intended for monitoring.
struct PreprocessorStatus
{
  std::uint64_t processed_clouds = 0;
  FilterStatistics last_cloud;
  std::size_t clipped_area_count = 0;
  bool fov_valid = false;
  ufil::type::SensorFOV fov;
};

/**
 * @brief Splits a lidar point cloud into dynamic points and static scene content.
 *
 * Per cloud, the field of view is estimated in the sensor frame. The cloud is then
 * transformed into the target frame, height filtered and split by the clipped areas. Points
 * inside clipped areas update the occlusion grid; the remaining points are outlier filtered,
 * voxel downsampled and returned.
 */
class UFIL_PCL_PREPROCESSOR_PUBLIC PclPreprocessor
{
public:
  explicit PclPreprocessor(const PreprocessorConfig & config = PreprocessorConfig{});

  /// @throws std::invalid_argument if the configuration is invalid; the previous one is kept.
  void configure(const PreprocessorConfig & config);
  const PreprocessorConfig & config() const;

  void setClippedAreas(std::vector<ClippedArea> clipped_areas);
  const std::vector<ClippedArea> & clippedAreas() const;

  /**
   * @param cloud_to_sensor Transform from the input cloud frame into the sensor frame.
   * @param sensor_to_target Transform from the sensor frame into the target frame.
   */
  void setTransforms(
    const Eigen::Isometry3d & cloud_to_sensor,
    const Eigen::Isometry3d & sensor_to_target);

  /// Filter a cloud given in the input frame; the output is expressed in the target frame.
  FilterStatistics filter(const pcl::PCLPointCloud2 & input, pcl::PCLPointCloud2 & output);

  ufil::type::OccupancyGrid occlusionGrid() const;
  /// Field of view in the sensor frame.
  ufil::type::SensorFOV fov() const;
  PreprocessorStatus status() const;

private:
  PreprocessorConfig config_;
  std::vector<ClippedArea> clipped_areas_;
  Eigen::Isometry3d cloud_to_sensor_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d sensor_to_target_ = Eigen::Isometry3d::Identity();

  DepthBuffer depth_buffer_;
  FOVEstimator fov_estimator_;

  std::uint64_t processed_clouds_ = 0;
  FilterStatistics last_statistics_;
};

}  // namespace ufil_pcl_preprocessor

#endif  // UFIL_PCL_PREPROCESSOR__UFIL_PCL_PREPROCESSOR_HPP_
