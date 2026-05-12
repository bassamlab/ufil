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

#include <pcl/point_types.h>
#include <pcl/common/pca.h>

#include <memory>
#include <vector>

#include "ufil_pcl_preprocessor/definitions.hpp"
#include "ufil_pcl_preprocessor/visibility_control.h"
#include "ufil_pcl_preprocessor/depth_buffer.hpp"
#include "ufil_pcl_preprocessor/fov_estimator.hpp"
#include "ufil_object_tracking/types/occupancy_grid.hpp"

namespace ufil_pcl_preprocessor
{

class PclProcessing
{
private:
  float filter_outlier_radius_ = 0.0f;
  float filter_min_number_of_neighbors_ = 0.0f;
  float filter_voxel_size_ = 0.0f;
  float filter_z_max_ = 0.0f;
  float filter_z_min_ = 0.0f;

  std::vector<ClippedArea> clipped_areas_;
  Eigen::Affine3d transform_;
  Eigen::Affine3d fov_transform_;

  ufil_pcl_preprocessor::DepthBuffer depth_buffer_;
  ufil_pcl_preprocessor::FOVEstimator fov_estimator_;

  // Depth buffer configuration stored here so configureDepthBuffer() can use runtime params
  size_t depth_buffer_history_length_ = 10;
  double depth_buffer_min_range_ = 5.0;
  double depth_buffer_max_range_ = 44.0;
  double depth_buffer_grid_resolution_ = 0.4;
  double depth_buffer_grid_size_x_ = 80.0;
  double depth_buffer_grid_size_y_ = 80.0;
  double depth_buffer_decay_rate_ = 0.8;
  int depth_buffer_az_bins_ = 500;
  int depth_buffer_el_bins_ = 100;
  Eigen::Vector3d depth_buffer_sensor_origin_{0.2, -0.2, 5.9};
  int preprocess_T_low_ = 30;
  int preprocess_T_high_ = 40;
  int preprocess_K_ = 3;
  int preprocess_R_ = 1;
  int preprocess_min_count_ = 4;

public:
  using SharedPtr = std::shared_ptr<ufil_pcl_preprocessor::PclProcessing>;
  using UniquePtr = std::unique_ptr<ufil_pcl_preprocessor::PclProcessing>;

  void filter(const pcl::PCLPointCloud2::Ptr & input_cloud, pcl::PCLPointCloud2 & output_cloud);

  void setClippedAreas(const std::vector<ClippedArea> & clipped_areas);
  void setTransform(Eigen::Affine3d transform);
  void setFovTransform(Eigen::Affine3d transform);

  void setOutlierFilterParameter(
    const float filter_outlier_radius,
    const float filter_min_number_of_neighbors);
  void setVoxelFilterSize(const float voxel_size);
  void setZFilterRange(const float z_min, const float z_max);
  void setNumberOfBins(const int num_bins);
  // FOV estimator history length (how many last FOV estimates to average)
  void setFOVHistoryLength(const int history_length);
  void setFOVMaxRange(const float max_range);

  // Depth buffer parameter setters (exposed so node / dynamic params can update them)
  void setDepthBufferHistoryLength(const size_t length);
  void setDepthBufferMinMaxRange(const double min_m, const double max_m);
  void setDepthBufferGridParameters(
    const double resolution_m, const double size_x_m,
    const double size_y_m);
  void setDepthBufferDecayRate(const double decay);
  void setDepthBufferSensorOrigin(const Eigen::Vector3d & origin);
  void setDepthBufferPreprocessThresholds(
    int low, int high, int strong_neighbor_threshold,
    int fill_radius, int fill_min_count);
  void setDepthBufferBins(int azimuth_bins, int elevation_bins);

  ufil::type::OccupancyGrid getOcclusionMap() const;
  ufil::type::SensorFOV getFOV() const;
};

}  // namespace ufil_pcl_preprocessor

#endif  // UFIL_PCL_PREPROCESSOR__UFIL_PCL_PREPROCESSOR_HPP_
