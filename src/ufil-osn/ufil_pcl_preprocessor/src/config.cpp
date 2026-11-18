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

#include "ufil_pcl_preprocessor/config.hpp"

#include <cmath>
#include <optional>
#include <string>

namespace ufil_pcl_preprocessor
{

std::optional<std::string> validate(const PreprocessorConfig & config)
{
  const auto & filter = config.filter;
  const auto & fov = config.fov;
  const auto & grid = config.occlusion_grid;

  for (const double value : {filter.voxel_size, filter.outlier_radius, filter.z_min,
      filter.z_max, fov.max_range, grid.min_range, grid.max_range, grid.resolution,
      grid.size_x, grid.size_y, grid.decay_rate})
  {
    if (!std::isfinite(value)) {
      return "all floating point parameters must be finite";
    }
  }

  if (filter.voxel_size <= 0.0) {
    return "filter.voxel_size must be > 0";
  }
  if (filter.outlier_radius <= 0.0) {
    return "filter.outlier_radius must be > 0";
  }
  if (filter.min_neighbors < 0) {
    return "filter.min_neighbors must be >= 0";
  }
  if (filter.z_min >= filter.z_max) {
    return "filter.z_min must be smaller than filter.z_max";
  }
  if (fov.num_bins <= 0) {
    return "fov.num_bins must be > 0";
  }
  if (fov.history_length <= 0) {
    return "fov.history_length must be > 0";
  }
  if (fov.max_range <= 0.0) {
    return "fov.max_range must be > 0";
  }
  if (grid.min_range < 0.0) {
    return "occlusion_grid.min_range must be >= 0";
  }
  if (grid.min_range >= grid.max_range) {
    return "occlusion_grid.min_range must be smaller than occlusion_grid.max_range";
  }
  if (grid.resolution <= 0.0) {
    return "occlusion_grid.resolution must be > 0";
  }
  if (grid.size_x < grid.resolution || grid.size_y < grid.resolution) {
    return "occlusion_grid.size_x and occlusion_grid.size_y must be >= occlusion_grid.resolution";
  }
  if (grid.decay_rate < 0.0 || grid.decay_rate > 1.0) {
    return "occlusion_grid.decay_rate must be in [0, 1]";
  }
  return std::nullopt;
}

}  // namespace ufil_pcl_preprocessor
