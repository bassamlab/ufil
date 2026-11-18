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

#ifndef UFIL_OSN_TRACKER__CONFIG_HPP_
#define UFIL_OSN_TRACKER__CONFIG_HPP_

#include <optional>
#include <string>

#include "ufil_osn_tracker/visibility_control.hpp"

namespace ufil_osn_tracker
{

struct TrackerConfig
{
  /// Maximum euclidean distance between a track and an associated measurement [m].
  double association_threshold = 4.0;
  /// Time span of the history kept per track [s].
  double history_duration = 10.0;

  /// Euclidean clustering of the dynamic point cloud into detections.
  int cluster_min_points = 40;
  int cluster_max_points = 50000;
  /// Maximum distance between neighbouring points of a cluster [m].
  double cluster_tolerance = 0.5;

  /// Scaling of the measurement covariances derived from the clusters.
  double dimension_covariance_multiplier = 10.0;
  double position_covariance_multiplier = 0.5;
  double orientation_covariance_multiplier = 0.01;

  /// A gap between consecutive updates larger than this resets the tracker, e.g. when a
  /// recording restarts [s].
  double time_jump_threshold = 10.0;
};

/**
 * @brief Check a configuration for consistency.
 * @return A human readable description of the first problem found, or std::nullopt if valid.
 */
UFIL_OSN_TRACKER_PUBLIC
std::optional<std::string> validate(const TrackerConfig & config);

}  // namespace ufil_osn_tracker

#endif  // UFIL_OSN_TRACKER__CONFIG_HPP_
