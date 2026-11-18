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

#include "ufil_osn_tracker/config.hpp"

#include <cmath>
#include <optional>
#include <string>

namespace ufil_osn_tracker
{

std::optional<std::string> validate(const TrackerConfig & config)
{
  for (const double value : {config.association_threshold, config.history_duration,
      config.cluster_tolerance, config.dimension_covariance_multiplier,
      config.position_covariance_multiplier, config.orientation_covariance_multiplier,
      config.time_jump_threshold})
  {
    if (!std::isfinite(value)) {
      return "all floating point parameters must be finite";
    }
  }

  if (config.association_threshold <= 0.0) {
    return "association threshold must be > 0";
  }
  if (config.history_duration <= 0.0) {
    return "history duration must be > 0";
  }
  if (config.cluster_min_points < 1) {
    return "cluster minimum number of points must be >= 1";
  }
  if (config.cluster_max_points < config.cluster_min_points) {
    return "cluster maximum number of points must be >= the minimum number of points";
  }
  if (config.cluster_tolerance <= 0.0) {
    return "cluster tolerance must be > 0";
  }
  if (config.dimension_covariance_multiplier <= 0.0 ||
    config.position_covariance_multiplier <= 0.0 ||
    config.orientation_covariance_multiplier <= 0.0)
  {
    return "covariance multipliers must be > 0";
  }
  if (config.time_jump_threshold <= 0.0) {
    return "time jump threshold must be > 0";
  }
  return std::nullopt;
}

}  // namespace ufil_osn_tracker
