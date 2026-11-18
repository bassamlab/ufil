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


#include "ufil_obu_object_aggregation/object_aggregator.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/UTMUPS.hpp>

namespace ufil_obu_object_aggregation
{

namespace
{

/// Variance of the configured, exactly known object dimensions.
constexpr double kDimensionVariance = 1e-9;

std::uint32_t randomId()
{
  std::random_device device;
  std::uniform_int_distribution<std::uint32_t> distribution;
  return distribution(device);
}

bool isValid(const GnssFix & fix)
{
  return fix.has_fix && std::isfinite(fix.latitude) && std::isfinite(fix.longitude) &&
         std::abs(fix.latitude) <= 90.0 && std::abs(fix.longitude) <= 180.0;
}

/// Copies the cross-covariances of the 2D state from the 6x6 ROS-style covariances. Velocity
/// and acceleration are still in the vehicle frame.
void fillStateCovariance(
  const Odometry & odometry, const Acceleration & acceleration, std::array<double, 64> & cov)
{
  const auto & pose = odometry.pose_covariance;
  const auto & twist = odometry.twist_covariance;
  const auto & accel = acceleration.covariance;

  cov[0] = pose[0];      // Cov(x, x)
  cov[1] = pose[1];      // Cov(x, y)
  cov[6] = pose[5];      // Cov(x, yaw)
  cov[8] = pose[6];      // Cov(y, x)
  cov[9] = pose[7];      // Cov(y, y)
  cov[14] = pose[11];    // Cov(y, yaw)
  cov[18] = twist[0];    // Cov(v_x, v_x)
  cov[19] = twist[1];    // Cov(v_x, v_y)
  cov[23] = twist[5];    // Cov(v_x, yaw_rate)
  cov[26] = twist[6];    // Cov(v_y, v_x)
  cov[27] = twist[7];    // Cov(v_y, v_y)
  cov[31] = twist[11];   // Cov(v_y, yaw_rate)
  cov[36] = accel[0];    // Cov(a_x, a_x)
  cov[37] = accel[1];    // Cov(a_x, a_y)
  cov[44] = accel[6];    // Cov(a_y, a_x)
  cov[45] = accel[7];    // Cov(a_y, a_y)
  cov[48] = pose[30];    // Cov(yaw, x)
  cov[49] = pose[31];    // Cov(yaw, y)
  cov[54] = pose[35];    // Cov(yaw, yaw)
  cov[58] = twist[30];   // Cov(yaw_rate, v_x)
  cov[59] = twist[31];   // Cov(yaw_rate, v_y)
  cov[63] = twist[35];   // Cov(yaw_rate, yaw_rate)
}

constexpr std::size_t kStateSize = 8;
static_assert(kStateSize * kStateSize == std::tuple_size_v<decltype(OwnObject::state_covariance)>);
constexpr std::size_t kVx = 2;
constexpr std::size_t kAx = 4;
constexpr std::size_t kYaw = 6;

/**
 * @brief Rotates velocity and acceleration from the vehicle frame into the global frame.
 *
 * v_global = R(yaw) * v_vehicle and a_global = R(yaw) * a_vehicle. The covariance is propagated
 * to first order with the Jacobian J of this transformation, P_global = J * P_vehicle * J^T,
 * including the dependency on the yaw.
 */
void rotateIntoGlobalFrame(OwnObject & object)
{
  using Matrix = std::array<std::array<double, kStateSize>, kStateSize>;

  const double c = std::cos(object.yaw);
  const double s = std::sin(object.yaw);

  Matrix jacobian{};
  for (std::size_t i = 0; i < kStateSize; ++i) {
    jacobian[i][i] = 1.0;
  }
  const auto set_rotation_block = [&](std::size_t first, double vx, double vy) {
      jacobian[first][first] = c;
      jacobian[first][first + 1] = -s;
      jacobian[first + 1][first] = s;
      jacobian[first + 1][first + 1] = c;
      // Derivative of R(yaw) * (vx, vy) with respect to yaw.
      jacobian[first][kYaw] = -s * vx - c * vy;
      jacobian[first + 1][kYaw] = c * vx - s * vy;
    };
  set_rotation_block(kVx, object.v_x, object.v_y);
  set_rotation_block(kAx, object.a_x, object.a_y);

  Matrix covariance{};
  for (std::size_t i = 0; i < kStateSize; ++i) {
    for (std::size_t j = 0; j < kStateSize; ++j) {
      covariance[i][j] = object.state_covariance[i * kStateSize + j];
    }
  }

  // P_global = J * P * J^T
  Matrix jp{};
  for (std::size_t i = 0; i < kStateSize; ++i) {
    for (std::size_t j = 0; j < kStateSize; ++j) {
      for (std::size_t k = 0; k < kStateSize; ++k) {
        jp[i][j] += jacobian[i][k] * covariance[k][j];
      }
    }
  }
  for (std::size_t i = 0; i < kStateSize; ++i) {
    for (std::size_t j = 0; j < kStateSize; ++j) {
      double value = 0.0;
      for (std::size_t k = 0; k < kStateSize; ++k) {
        value += jp[i][k] * jacobian[j][k];
      }
      object.state_covariance[i * kStateSize + j] = value;
    }
  }

  const double v_x = c * object.v_x - s * object.v_y;
  const double v_y = s * object.v_x + c * object.v_y;
  const double a_x = c * object.a_x - s * object.a_y;
  const double a_y = s * object.a_x + c * object.a_y;
  object.v_x = v_x;
  object.v_y = v_y;
  object.a_x = a_x;
  object.a_y = a_y;
}

}  // namespace

std::string UtmPosition::frameId() const
{
  return "utm_" + std::to_string(zone) + (northern_hemisphere ? "n" : "s");
}

ObjectAggregator::ObjectAggregator(const AggregatorConfig & config)
: id_(randomId())
{
  configure(config);
}

void ObjectAggregator::configure(const AggregatorConfig & config)
{
  if (const auto error = validate(config)) {
    throw std::invalid_argument("invalid object aggregation configuration: " + *error);
  }
  config_ = config;
}

const AggregatorConfig & ObjectAggregator::config() const
{
  return config_;
}

AggregationEvent ObjectAggregator::aggregate(
  const GnssFix & fix, const Odometry & odometry, const Acceleration & acceleration)
{
  last_rejected_ = true;
  if (!isValid(fix)) {
    ++rejected_invalid_fix_;
    return AggregationEvent::kRejectedInvalidFix;
  }

  UtmPosition position;
  try {
    const int setzone =
      config_.utm_zone == kUtmZoneAutomatic ? GeographicLib::UTMUPS::STANDARD : config_.utm_zone;
    GeographicLib::UTMUPS::Forward(
      fix.latitude, fix.longitude, position.zone, position.northern_hemisphere,
      position.easting, position.northing, setzone);
  } catch (const GeographicLib::GeographicErr &) {
    // E.g. a position too far outside the forced UTM zone.
    ++rejected_invalid_fix_;
    return AggregationEvent::kRejectedInvalidFix;
  }
  last_rejected_ = false;

  const bool zone_changed = !last_object_ ||
    last_object_->position.zone != position.zone ||
    last_object_->position.northern_hemisphere != position.northern_hemisphere;

  OwnObject object;
  object.id = id_;
  object.position = position;
  object.v_x = odometry.v_x;
  object.v_y = odometry.v_y;
  object.a_x = acceleration.a_x;
  object.a_y = acceleration.a_y;
  object.yaw = odometry.yaw;
  object.yaw_rate = odometry.yaw_rate;
  fillStateCovariance(odometry, acceleration, object.state_covariance);
  rotateIntoGlobalFrame(object);
  object.length = config_.object_length;
  object.width = config_.object_width;
  object.dimension_covariance[0] = kDimensionVariance;  // Var(length)
  object.dimension_covariance[4] = kDimensionVariance;  // Var(width)
  object.classification[static_cast<std::size_t>(config_.object_class)] = 1.0f;

  last_object_ = object;
  ++aggregated_;
  return zone_changed ? AggregationEvent::kUtmZoneChanged : AggregationEvent::kAggregated;
}

const std::optional<OwnObject> & ObjectAggregator::lastObject() const
{
  return last_object_;
}

AggregatorStatus ObjectAggregator::status() const
{
  AggregatorStatus status;
  status.object_id = id_;
  status.aggregated = aggregated_;
  status.rejected_invalid_fix = rejected_invalid_fix_;
  status.last_rejected = last_rejected_;
  if (last_object_) {
    status.last_position = last_object_->position;
  }
  return status;
}

}  // namespace ufil_obu_object_aggregation
