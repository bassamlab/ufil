// Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#ifndef UFIL_PCL_PREPROCESSOR__CONFIG_HPP_
#define UFIL_PCL_PREPROCESSOR__CONFIG_HPP_

#include <optional>
#include <string>

#include "ufil_pcl_preprocessor/visibility_control.hpp"

namespace ufil_pcl_preprocessor
{

/// Point filtering applied to the cloud after it was transformed into the target frame.
struct FilterConfig
{
  /// Edge length of the voxel grid applied to the dynamic points [m].
  double voxel_size = 0.1;
  /// Radius used by the radius outlier removal [m].
  double outlier_radius = 0.5;
  /// Minimum number of neighbours within outlier_radius to keep a point.
  int min_neighbors = 40;
  /// Points below z_min (ground) or above z_max are discarded [m].
  double z_min = 0.2;
  double z_max = 5.0;
};

/// Field-of-view estimation performed in the sensor frame.
struct FovConfig
{
  /// Number of azimuth bins spanning [-pi/2, pi/2].
  int num_bins = 150;
  /// Number of per-cloud estimates averaged into the published field of view.
  int history_length = 60;
  /// Upper bound for the estimated range [m].
  double max_range = 32.0;
};

/// Occlusion grid built from the points inside the static (clipped) areas.
struct OcclusionGridConfig
{
  /// Only points within [min_range, max_range] contribute [m].
  double min_range = 5.0;
  double max_range = 44.0;
  /// Grid cell edge length [m].
  double resolution = 0.4;
  /// Grid extent, centred on the target frame origin [m].
  double size_x = 80.0;
  double size_y = 80.0;
  /// Decay of free cells per update in [0, 1]; 1 keeps cells, 0 clears them immediately.
  double decay_rate = 0.8;
};

struct PreprocessorConfig
{
  FilterConfig filter;
  FovConfig fov;
  OcclusionGridConfig occlusion_grid;
};

/**
 * @brief Check a configuration for consistency.
 * @return A human readable description of the first problem found, or std::nullopt if valid.
 */
UFIL_PCL_PREPROCESSOR_PUBLIC
std::optional<std::string> validate(const PreprocessorConfig & config);

}  // namespace ufil_pcl_preprocessor

#endif  // UFIL_PCL_PREPROCESSOR__CONFIG_HPP_
