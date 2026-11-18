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


#ifndef UFIL_OBU_OBJECT_AGGREGATION__OBJECT_AGGREGATOR_HPP_
#define UFIL_OBU_OBJECT_AGGREGATION__OBJECT_AGGREGATOR_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "ufil_obu_object_aggregation/config.hpp"
#include "ufil_obu_object_aggregation/visibility_control.hpp"

namespace ufil_obu_object_aggregation
{

/// Row-major 6x6 covariance over (x, y, z, roll, pitch, yaw) or its derivatives.
using Covariance6 = std::array<double, 36>;

struct GnssFix
{
  double latitude = 0.0;
  double longitude = 0.0;
  /// False if the receiver reports no fix.
  bool has_fix = false;
};

/// Filtered odometry of the own vehicle. Yaw is given in the global frame, the twist in the
/// vehicle frame (x forward, y left), as published by robot_localization.
struct Odometry
{
  double yaw = 0.0;
  double v_x = 0.0;
  double v_y = 0.0;
  double yaw_rate = 0.0;
  Covariance6 pose_covariance{};
  Covariance6 twist_covariance{};
};

/// Acceleration of the own vehicle in the vehicle frame (x forward, y left).
struct Acceleration
{
  double a_x = 0.0;
  double a_y = 0.0;
  Covariance6 covariance{};
};

struct UtmPosition
{
  double easting = 0.0;
  double northing = 0.0;
  int zone = 0;
  bool northern_hemisphere = true;

  /// Name of the corresponding frame, e.g. "utm_32n".
  UFIL_OBU_OBJECT_AGGREGATION_PUBLIC
  std::string frameId() const;
};

/// The own vehicle as an object. Velocity and acceleration are expressed in the global (UTM)
/// frame, like all UFIL object states.
struct OwnObject
{
  std::uint32_t id = 0;
  UtmPosition position;
  double v_x = 0.0;
  double v_y = 0.0;
  double a_x = 0.0;
  double a_y = 0.0;
  double yaw = 0.0;
  double yaw_rate = 0.0;
  /// Row-major 8x8 covariance over (x, y, v_x, v_y, a_x, a_y, yaw, yaw_rate).
  std::array<double, 64> state_covariance{};
  double length = 0.0;
  double width = 0.0;
  /// Row-major 3x3 covariance over (length, width, height).
  std::array<double, 9> dimension_covariance{};
  std::array<float, kObjectClassCount> classification{};
};

enum class AggregationEvent
{
  /// An object was produced.
  kAggregated,
  /// An object was produced in a different UTM zone than the previous one (or the first one).
  kUtmZoneChanged,
  /// The GNSS fix was missing or invalid; no object was produced.
  kRejectedInvalidFix,
};

/// Snapshot of the aggregator state, intended for monitoring.
struct AggregatorStatus
{
  std::uint32_t object_id = 0;
  std::uint64_t aggregated = 0;
  std::uint64_t rejected_invalid_fix = 0;
  /// Whether the most recent input was rejected.
  bool last_rejected = false;
  std::optional<UtmPosition> last_position;
};

/**
 * @brief Combines the GNSS fix, filtered odometry and acceleration of the own vehicle into
 * an object in UTM coordinates.
 *
 * A random object id is drawn on construction and kept for the lifetime of the aggregator.
 */
class UFIL_OBU_OBJECT_AGGREGATION_PUBLIC ObjectAggregator
{
public:
  /// @throws std::invalid_argument if the configuration is invalid.
  explicit ObjectAggregator(const AggregatorConfig & config = AggregatorConfig{});

  /// @throws std::invalid_argument if the configuration is invalid; the previous one is kept.
  void configure(const AggregatorConfig & config);
  const AggregatorConfig & config() const;

  AggregationEvent aggregate(
    const GnssFix & fix, const Odometry & odometry, const Acceleration & acceleration);

  /// The object produced by the most recent successful aggregate() call.
  const std::optional<OwnObject> & lastObject() const;

  AggregatorStatus status() const;

private:
  AggregatorConfig config_;
  std::uint32_t id_;
  std::optional<OwnObject> last_object_;
  std::uint64_t aggregated_ = 0;
  std::uint64_t rejected_invalid_fix_ = 0;
  bool last_rejected_ = false;
};

}  // namespace ufil_obu_object_aggregation

#endif  // UFIL_OBU_OBJECT_AGGREGATION__OBJECT_AGGREGATOR_HPP_
