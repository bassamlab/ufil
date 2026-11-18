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

#ifndef SSL_VEHICLE_TRACKING__TYPES_HPP_
#define SSL_VEHICLE_TRACKING__TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ssl_vehicle_tracking
{

/// Two-dimensional point or vector [m] or [m/s].
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

/// Rigid transform in the plane: rotation by yaw, then translation by (x, y).
struct Pose2D
{
  double x = 0.0;    ///< [m]
  double y = 0.0;    ///< [m]
  double yaw = 0.0;  ///< [rad]
};

/// Latest state of a wheel track.
struct WheelTrack
{
  std::string id;
  Point2D position;  ///< [m]
  Point2D velocity;  ///< [m/s]
  /// Consecutive frames without detection of the wheel.
  int frames_unseen = 0;
};

/// Vehicle assembled from wheel tracks.
struct Vehicle
{
  std::uint32_t id = 0;
  /// Index of the vehicle template in VehicleTracker::templates().
  std::size_t template_index = 0;
  /// Assigned wheel track per template wheel, std::nullopt if unassigned.
  std::vector<std::optional<std::string>> wheel_tracks;
  /// Per template wheel: last frames_unseen of its track, decreased by one every frame.
  std::vector<std::int64_t> wheel_last_seen;
  /// Probability that each wheel track is correctly assigned.
  std::vector<double> wheel_probabilities;
  double existence_probability = 1.0;

  /// Mean position of the assigned wheels [m].
  Point2D position;
  /// Mean velocity of the assigned wheels [m/s].
  Point2D velocity;
  /// Direction of the velocity, kept while the vehicle stands still [rad].
  double yaw = 0.0;
};

}  // namespace ssl_vehicle_tracking

#endif  // SSL_VEHICLE_TRACKING__TYPES_HPP_
