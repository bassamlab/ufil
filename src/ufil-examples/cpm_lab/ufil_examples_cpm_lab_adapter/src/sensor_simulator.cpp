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

#include "ufil_examples_cpm_lab_adapter/sensor_simulator.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <GeographicLib/UTMUPS.hpp>

namespace ufil_examples_cpm_lab_adapter
{

namespace
{

/// GNSS position variances with and without artificial errors [m^2].
constexpr double kArtificialGnssVariance = 1.0;
constexpr double kExactGnssVariance = 0.1;

double yawOf(const Eigen::Isometry3d & transform)
{
  return std::atan2(transform.linear()(1, 0), transform.linear()(0, 0));
}

}  // namespace

std::optional<std::string> validate(const SensorSimulatorConfig & config)
{
  if (!std::isfinite(config.origin_latitude) || std::abs(config.origin_latitude) > 90.0) {
    return "the origin latitude must be in [-90, 90] deg";
  }
  if (!std::isfinite(config.origin_longitude) || std::abs(config.origin_longitude) > 180.0) {
    return "the origin longitude must be in [-180, 180] deg";
  }
  if (!std::isfinite(config.max_state_age) || config.max_state_age <= 0.0) {
    return "the maximum state age must be positive";
  }
  return validate(config.gnss_errors);
}

SensorSimulator::SensorSimulator(
  const SensorSimulatorConfig & config, std::optional<unsigned int> seed)
: config_(config),
  artificial_gnss_(config.gnss_errors, seed)
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument("invalid sensor simulator configuration: " + *error);
  }
  GeographicLib::UTMUPS::Forward(
    config_.origin_latitude, config_.origin_longitude, zone_, northp_, origin_easting_,
    origin_northing_);
}

void SensorSimulator::setState(const VehicleState & state)
{
  state_ = state;
  status_.states_received++;
  status_.last_state_stamp_nanoseconds = state.stamp_nanoseconds;
  status_.has_state = true;
}

StateCheck SensorSimulator::checkState(std::int64_t now_nanoseconds)
{
  if (!state_) {
    return StateCheck::kNoState;
  }
  const double age = static_cast<double>(now_nanoseconds - state_->stamp_nanoseconds) * 1e-9;
  if (age >= config_.max_state_age) {
    state_.reset();
    status_.states_expired++;
    status_.has_state = false;
    return StateCheck::kExpired;
  }
  return StateCheck::kValid;
}

const VehicleState & SensorSimulator::requireState() const
{
  if (!state_) {
    throw std::logic_error("no vehicle state available");
  }
  return *state_;
}

GnssFix SensorSimulator::gnssFix(const Eigen::Isometry3d & state_to_target)
{
  const VehicleState & state = requireState();
  const Eigen::Vector3d lab_position = config_.artificial_gnss_errors ?
    artificial_gnss_.generateFix(state.position, state.yaw) :
    Eigen::Vector3d(state.position.x(), state.position.y(), 0.0);
  const Eigen::Vector3d position = (state_to_target * lab_position) * kLabScale;

  GnssFix fix;
  GeographicLib::UTMUPS::Reverse(
    zone_, northp_, origin_easting_ + position.x(), origin_northing_ + position.y(),
    fix.latitude, fix.longitude);
  fix.position_variance =
    config_.artificial_gnss_errors ? kArtificialGnssVariance : kExactGnssVariance;
  status_.gnss_fixes++;
  return fix;
}

ImuMeasurement SensorSimulator::imu(const Eigen::Isometry3d & state_to_target)
{
  const VehicleState & state = requireState();
  ImuMeasurement measurement;
  measurement.yaw = state.yaw + yawOf(state_to_target);
  measurement.angular_velocity = state.angular_velocity;
  measurement.linear_acceleration = state.linear_acceleration;
  status_.imu_measurements++;
  return measurement;
}

WheelOdometry SensorSimulator::wheelOdometry()
{
  const VehicleState & state = requireState();
  WheelOdometry odometry;
  odometry.linear_velocity =
    Eigen::Vector3d(state.linear_velocity.x(), state.linear_velocity.y(), 0.0) * kLabScale;
  status_.odometry_measurements++;
  return odometry;
}

}  // namespace ufil_examples_cpm_lab_adapter
