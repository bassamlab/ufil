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

#ifndef SSL_WHEEL_TRACKER__CONFIG_HPP_
#define SSL_WHEEL_TRACKER__CONFIG_HPP_

#include <cstddef>
#include <optional>
#include <string>

#include "ssl_wheel_tracker/types.hpp"
#include "ssl_wheel_tracker/visibility_control.hpp"

namespace ssl_wheel_tracker
{

/// Configuration of the WheelDetector.
struct DetectorConfig
{
  /// Cells must exceed this pressure to be a local maximum [0, 254].
  int threshold = 10;
  /// Radius of the square window in which a local maximum is the largest value [cells, >= 1].
  std::size_t min_peak_distance = 2;
  /// Cells within this L1 distance of a maximum belong to its wheel contact [cells, >= 1].
  std::size_t contact_radius = 3;
  /// Local maxima within this distance of the grid border are ignored [cells].
  std::size_t exclude_border = 1;
};

/// Configuration of the WheelTracker.
struct TrackerConfig
{
  /// Spectral density of the white-noise acceleration of the constant velocity model per axis
  /// [m^2/s^3].
  double process_noise = 0.01;
  /// Variance of the detected position per axis [m^2].
  double measurement_noise = 0.01;
  /// A detection is only associated with a track if its distance to the predicted position is
  /// smaller [m]; it is also the cost of leaving a track without detection.
  double gate_distance = 2.0;
  /// Associated detections, including the first one, before a tentative track is confirmed.
  std::size_t confirmation_hits = 4;
  /// Tracks are deleted when no detection was associated for longer than this [s].
  double deletion_timeout = 0.25;
  /// Matched states kept in the history of a track.
  std::size_t history_length = 10;
  /// Prior velocity of a new track [m/s].
  Point2D initial_velocity{0.35, 0.35};
  /// Prior velocity variance of a new track per axis [m^2/s^2].
  double initial_velocity_variance = 10.0;
};

/// @return a description of the first invalid value, or std::nullopt if the configuration is
///         valid.
SSL_WHEEL_TRACKER_PUBLIC std::optional<std::string> validate(const DetectorConfig & config);
/// @copydoc validate(const DetectorConfig &)
SSL_WHEEL_TRACKER_PUBLIC std::optional<std::string> validate(const TrackerConfig & config);

}  // namespace ssl_wheel_tracker

#endif  // SSL_WHEEL_TRACKER__CONFIG_HPP_
