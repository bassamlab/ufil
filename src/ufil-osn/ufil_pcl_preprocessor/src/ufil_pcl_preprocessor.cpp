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

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <stdexcept>
#include <iostream>
#include <unordered_set>

#include "ufil_pcl_preprocessor/ufil_pcl_preprocessor.hpp"

#if PCL_VERSION_COMPARE(>=, 1, 14, 0)
#include <pcl/filters/box_clipper3D.h>
#else
#include "box_clipper_3d.hpp"
#endif

namespace ufil_pcl_preprocessor
{

void PclProcessing::filter(
  const pcl::PCLPointCloud2::Ptr & input_cloud,
  pcl::PCLPointCloud2 & output_cloud)
{
  using CloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

  CloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(*input_cloud, *cloud);
  if (cloud->empty()) {
    output_cloud = *input_cloud;
    return;
  }

  const auto point_in_clipped_area = [](const pcl::PointXYZ & pt, const ClippedArea & area) {
      const float dx = pt.x - area.x;
      const float dy = pt.y - area.y;
      const float dz = pt.z - area.z;

      const float c = std::cos(area.yaw);
      const float s = std::sin(area.yaw);

      const float local_x = c * dx + s * dy;
      const float local_y = -s * dx + c * dy;

      return std::abs(local_x) <= area.length * 0.5f &&
             std::abs(local_y) <= area.width * 0.5f &&
             std::abs(dz) <= area.height * 0.5f;
    };

  // Apply FOV transforms
  pcl::transformPointCloud(*cloud, *cloud, this->fov_transform_);
  this->fov_estimator_.update(cloud);
  pcl::transformPointCloud(*cloud, *cloud, this->transform_);

  size_t N = cloud->size();
  std::vector<bool> is_ground(N, false);
  std::vector<bool> is_static(N, false);

  // ---------------------------------
  // 1. Z-filter (ground removal)
  // ---------------------------------
  CloudPtr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  cloud_filtered->reserve(N);
  std::vector<int> z_filtered_indices;  // indices in original cloud that survive
  z_filtered_indices.reserve(N);
  for (size_t i = 0; i < N; ++i) {
    const auto & pt = cloud->points[i];
    if (pt.z >= this->filter_z_min_ && pt.z <= this->filter_z_max_) {
      cloud_filtered->push_back(pt);
      z_filtered_indices.push_back(i);
    } else {
      is_ground[i] = true;
    }
  }

  // ---------------------------------
  // 2. Clip boxes
  // ---------------------------------
  std::vector<uint8_t> clipped_mask(cloud_filtered->size(), 0);

  for (const auto & clipped_area : clipped_areas_) {
    for (size_t i = 0; i < cloud_filtered->size(); ++i) {
      if (clipped_mask[i]) {
        continue;
      }
      if (point_in_clipped_area(cloud_filtered->points[i], clipped_area)) {
        clipped_mask[i] = 1;
        is_static[z_filtered_indices[i]] = true;
      }
    }
  }

  CloudPtr cloud_clipped(new pcl::PointCloud<pcl::PointXYZ>());
  cloud_clipped->reserve(cloud_filtered->size());
  std::vector<int> clipped_to_original;
  clipped_to_original.reserve(cloud_filtered->size());
  for (size_t i = 0; i < cloud_filtered->size(); ++i) {
    if (!clipped_mask[i]) {
      cloud_clipped->push_back(cloud_filtered->points[i]);
      clipped_to_original.push_back(z_filtered_indices[i]);
    }
  }

  // ---------------------------------
  // 3. Outlier removal
  // ---------------------------------
  CloudPtr cloud_no_outliers(new pcl::PointCloud<pcl::PointXYZ>(*cloud_clipped));
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outlier_filter;
  outlier_filter.setInputCloud(cloud_clipped);
  outlier_filter.setRadiusSearch(this->filter_outlier_radius_);
  outlier_filter.setMinNeighborsInRadius(this->filter_min_number_of_neighbors_);
  pcl::PointCloud<pcl::PointXYZ> temp_out;
  outlier_filter.filter(temp_out);
  *cloud_no_outliers = temp_out;

  // Mark points removed by outlier filter as static
  std::vector<uint8_t> surviving_mask(N, 0);
  for (size_t i = 0; i < cloud_no_outliers->size() && i < clipped_to_original.size(); ++i) {
    surviving_mask[clipped_to_original[i]] = 1;
  }
  for (auto idx : clipped_to_original) {
    if (!surviving_mask[idx]) {
      is_ground[idx] = true;
    }
  }

  // ---------------------------------
  // 4. Build static and dynamic clouds
  // ---------------------------------
  CloudPtr static_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  CloudPtr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (size_t i = 0; i < N; ++i) {
    if (is_ground[i]) {continue;}      // skip ground
    if (is_static[i]) {static_cloud->push_back(cloud->points[i]);} else {
      dynamic_cloud->push_back(cloud->points[i]);
    }
  }

  if (!static_cloud->empty()) {
    this->depth_buffer_.update(*static_cloud);
  }

  // ---------------------------------
  // 5. Voxel grid for dynamic points
  // ---------------------------------
  if (!dynamic_cloud->empty()) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(dynamic_cloud);
    voxel_filter.setLeafSize(this->filter_voxel_size_, this->filter_voxel_size_,
        this->filter_voxel_size_);
    CloudPtr dynamic_voxel(new pcl::PointCloud<pcl::PointXYZ>());
    voxel_filter.filter(*dynamic_voxel);
    dynamic_cloud.swap(dynamic_voxel);
  }

  // ---------------------------------
  // 6. Convert dynamic cloud to output
  // ---------------------------------
  pcl::toPCLPointCloud2(*dynamic_cloud, output_cloud);
}

void PclProcessing::setClippedAreas(const std::vector<ClippedArea> & clipped_areas)
{
  this->clipped_areas_ = clipped_areas;
}

void PclProcessing::setTransform(Eigen::Affine3d transform)
{
  this->transform_ = transform;
  this->depth_buffer_.setSensorOrigin(this->transform_.translation());
}

void PclProcessing::setFovTransform(Eigen::Affine3d transform)
{
  this->fov_transform_ = transform;
}

void PclProcessing::setOutlierFilterParameter(
  const float filter_outlier_radius,
  const float filter_min_number_of_neighbors)
{
  this->filter_outlier_radius_ = filter_outlier_radius;
  this->filter_min_number_of_neighbors_ = filter_min_number_of_neighbors;
}

void PclProcessing::setVoxelFilterSize(const float voxel_size)
{
  if (voxel_size <= 0) {
    throw std::invalid_argument("Voxel size must be greater then zero.");
  }
  this->filter_voxel_size_ = voxel_size;
}

void PclProcessing::setZFilterRange(const float z_min, const float z_max)
{
  if (z_min > z_max) {
    throw std::invalid_argument("Minimum value must be smaller or equal to maximum value.");
  }
  this->filter_z_min_ = z_min;
  this->filter_z_max_ = z_max;
}

void PclProcessing::setNumberOfBins(const int num_bins)
{
  if (num_bins <= 0) {
    throw std::invalid_argument("Bin size must be greater than zero.");
  }
  this->fov_estimator_.setNumberOfBins(num_bins);
}

void PclProcessing::setFOVHistoryLength(const int history_length)
{
  this->fov_estimator_.setHistoryLength(history_length);
}

void PclProcessing::setFOVMaxRange(const float max_range)
{
  this->fov_estimator_.setMaxRange(max_range);
}

void PclProcessing::setDepthBufferHistoryLength(const size_t length)
{
  this->depth_buffer_history_length_ = (length > 0) ? length : 1;
}

void PclProcessing::setDepthBufferMinMaxRange(const double min_m, const double max_m)
{
  if (min_m < 0 || min_m > max_m) {
    throw std::invalid_argument("Invalid depth buffer min/max range");
  }
  this->depth_buffer_min_range_ = min_m;
  this->depth_buffer_max_range_ = max_m;
  this->depth_buffer_.setMinRange(this->depth_buffer_min_range_);
  this->depth_buffer_.setMaxRange(this->depth_buffer_max_range_);
}

void PclProcessing::setDepthBufferGridParameters(
  const double resolution_m, const double size_x_m, const double size_y_m)
{
  if (resolution_m <= 0.0 || size_x_m <= 0.0 || size_y_m <= 0.0) {
    throw std::invalid_argument("Depth buffer grid parameters must be positive");
  }
  this->depth_buffer_grid_resolution_ = resolution_m;
  this->depth_buffer_grid_size_x_ = size_x_m;
  this->depth_buffer_grid_size_y_ = size_y_m;
}

void PclProcessing::setDepthBufferDecayRate(const double decay)
{
  this->depth_buffer_decay_rate_ = std::clamp(decay, 0.0, 1.0);
  this->depth_buffer_.setDecayRate(this->depth_buffer_decay_rate_);
}

void PclProcessing::setDepthBufferSensorOrigin(const Eigen::Vector3d & origin)
{
  this->depth_buffer_sensor_origin_ = origin;
  this->depth_buffer_.setSensorOrigin(this->depth_buffer_sensor_origin_);
}

void PclProcessing::setDepthBufferPreprocessThresholds(
  int low, int high, int strong_neighbor_threshold, int fill_radius, int fill_min_count)
{
  preprocess_T_low_ = low;
  preprocess_T_high_ = high;
  preprocess_K_ = strong_neighbor_threshold;
  preprocess_R_ = fill_radius;
  preprocess_min_count_ = fill_min_count;
}

void PclProcessing::setDepthBufferBins(int azimuth_bins, int elevation_bins)
{
  if (azimuth_bins <= 0 || elevation_bins <= 0) {
    throw std::invalid_argument("Depth buffer bin counts must be > 0");
  }
  depth_buffer_az_bins_ = azimuth_bins;
  depth_buffer_el_bins_ = elevation_bins;
}

ufil::type::OccupancyGrid PclProcessing::getOcclusionMap() const
{
  return this->depth_buffer_.getOccupancyGrid();
}

ufil::type::SensorFOV PclProcessing::getFOV() const
{
  return this->fov_estimator_.getFOV();
}

}  // namespace ufil_pcl_preprocessor
