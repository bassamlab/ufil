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

#include "ufil_pcl_preprocessor/ufil_pcl_preprocessor.hpp"

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ufil_pcl_preprocessor
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

PclPreprocessor::PclPreprocessor(const PreprocessorConfig & config)
{
  configure(config);
}

void PclPreprocessor::configure(const PreprocessorConfig & config)
{
  if (const auto error = validate(config)) {
    throw std::invalid_argument("invalid preprocessor configuration: " + *error);
  }
  config_ = config;
  fov_estimator_.setNumberOfBins(config.fov.num_bins);
  fov_estimator_.setHistoryLength(config.fov.history_length);
  fov_estimator_.setMaxRange(static_cast<float>(config.fov.max_range));
  depth_buffer_.configure(config.occlusion_grid);
}

const PreprocessorConfig & PclPreprocessor::config() const
{
  return config_;
}

void PclPreprocessor::setClippedAreas(std::vector<ClippedArea> clipped_areas)
{
  clipped_areas_ = std::move(clipped_areas);
}

const std::vector<ClippedArea> & PclPreprocessor::clippedAreas() const
{
  return clipped_areas_;
}

void PclPreprocessor::setTransforms(
  const Eigen::Isometry3d & cloud_to_sensor,
  const Eigen::Isometry3d & sensor_to_target)
{
  cloud_to_sensor_ = cloud_to_sensor;
  sensor_to_target_ = sensor_to_target;
  depth_buffer_.setSensorOrigin(sensor_to_target_.translation());
}

FilterStatistics PclPreprocessor::filter(
  const pcl::PCLPointCloud2 & input,
  pcl::PCLPointCloud2 & output)
{
  FilterStatistics statistics;

  auto cloud = std::make_shared<Cloud>();
  pcl::fromPCLPointCloud2(input, *cloud);
  statistics.input_points = cloud->size();

  if (!cloud->empty()) {
    // Estimate the field of view in the sensor frame, then continue in the target frame.
    pcl::transformPointCloud(*cloud, *cloud, Eigen::Affine3d(cloud_to_sensor_.matrix()));
    fov_estimator_.update(*cloud);
    pcl::transformPointCloud(*cloud, *cloud, Eigen::Affine3d(sensor_to_target_.matrix()));
  }

  // Split into out-of-height, static (inside a clipped area) and dynamic candidates.
  Cloud static_cloud;
  auto candidates = std::make_shared<Cloud>();
  candidates->reserve(cloud->size());
  for (const auto & pt : *cloud) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
      !(pt.z >= config_.filter.z_min && pt.z <= config_.filter.z_max))
    {
      ++statistics.out_of_height_points;
      continue;
    }
    const bool is_static = std::any_of(
      clipped_areas_.begin(), clipped_areas_.end(),
      [&pt](const ClippedArea & area) {return area.contains(pt.x, pt.y, pt.z);});
    if (is_static) {
      static_cloud.push_back(pt);
    } else {
      candidates->push_back(pt);
    }
  }
  statistics.static_points = static_cloud.size();

  if (!static_cloud.empty()) {
    depth_buffer_.update(static_cloud);
  }

  auto dynamic_cloud = std::make_shared<Cloud>();
  if (!candidates->empty()) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(candidates);
    outlier_filter.setRadiusSearch(config_.filter.outlier_radius);
    outlier_filter.setMinNeighborsInRadius(config_.filter.min_neighbors);
    outlier_filter.filter(*dynamic_cloud);
  }
  statistics.outlier_points = candidates->size() - dynamic_cloud->size();
  statistics.dynamic_points = dynamic_cloud->size();

  Cloud downsampled;
  if (!dynamic_cloud->empty()) {
    const auto leaf_size = static_cast<float>(config_.filter.voxel_size);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(dynamic_cloud);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(downsampled);
  }
  statistics.output_points = downsampled.size();
  pcl::toPCLPointCloud2(downsampled, output);

  ++processed_clouds_;
  last_statistics_ = statistics;
  return statistics;
}

ufil::type::OccupancyGrid PclPreprocessor::occlusionGrid() const
{
  return depth_buffer_.getOccupancyGrid();
}

ufil::type::SensorFOV PclPreprocessor::fov() const
{
  return fov_estimator_.getFOV();
}

PreprocessorStatus PclPreprocessor::status() const
{
  PreprocessorStatus status;
  status.processed_clouds = processed_clouds_;
  status.last_cloud = last_statistics_;
  status.clipped_area_count = clipped_areas_.size();
  status.fov_valid = processed_clouds_ > 0 && fov_estimator_.lastEstimateValid();
  status.fov = fov_estimator_.getFOV();
  return status;
}

}  // namespace ufil_pcl_preprocessor
