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

#ifndef UFIL_CENTRAL_FUSION__CONFIG_HPP_
#define UFIL_CENTRAL_FUSION__CONFIG_HPP_

#include <optional>
#include <string>
#include <vector>

#include "ufil_central_fusion/visibility_control.hpp"

namespace ufil_central_fusion
{

/// Trust in the existence and class estimates of a sensor, each in [0, 1].
struct SensorTrust
{
  double existence = 0.8;
  double car = 0.8;
  double truck = 0.8;
  double motorcycle = 0.8;
  double bicycle = 0.8;
  double pedestrian = 0.8;
  double stationary = 0.8;
  double other = 0.8;
};

/// A source of object lists fused by the RSU.
struct SensorConfig
{
  /// Unique name, e.g. "osn".
  std::string name;
  /// The objects stem from received CAMs. CAM objects do not initiate existence and messages
  /// with implausible stamps are dropped instead of resetting the fusion.
  bool is_cam = false;
  SensorTrust trust;
};

struct FusionConfig
{
  /// Maximum euclidean distance between a track and an associated measurement [m].
  double association_threshold = 3.0;

  /// Dempster-Shafer existence fusion: weight range and decay per update.
  double existence_min_weight = 0.2;
  double existence_max_weight = 0.8;
  double existence_decay_factor = 0.05;

  /// Dimension grid map: cell size [m], update factor and probability thresholds.
  double dimension_grid_resolution = 0.1;
  double dimension_alpha = 0.95;
  double dimension_p_min = 0.1;
  double dimension_p_ref = 0.5;
  double dimension_p_max = 0.9;

  /// Measurements are buffered this long to fuse late arriving data in order [s].
  double buffer_duration = 2.0;
  /// A larger gap between the stamps of consecutive measurements is treated as a time jump,
  /// e.g. a restarted recording [s].
  double time_jump_threshold = 10.0;
};

/**
 * @brief Check a configuration for consistency.
 * @return A human readable description of the first problem found, or std::nullopt if valid.
 */
UFIL_CENTRAL_FUSION_PUBLIC
std::optional<std::string> validate(const FusionConfig & config);

/// @copydoc validate(const FusionConfig &)
UFIL_CENTRAL_FUSION_PUBLIC
std::optional<std::string> validate(const std::vector<SensorConfig> & sensors);

}  // namespace ufil_central_fusion

#endif  // UFIL_CENTRAL_FUSION__CONFIG_HPP_
