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

#include "ufil_etsi/vehicle_cam_generator.hpp"

#include <numbers>  // cpplint treats this C++20 header as a C system header

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ufil_etsi/cam_encoding.hpp"
#include "ufil_etsi/its_time.hpp"

#include "covariance.hpp"

namespace ufil_etsi
{

namespace
{

// Thresholds of condition 1 (ETSI EN 302 637-2, clause 6.1.3).
constexpr double kHeadingThreshold = 4.0 * std::numbers::pi / 180.0;  // [rad]
constexpr double kPositionThreshold = 4.0;  // [m]
constexpr double kSpeedThreshold = 0.5;  // [m/s]

}  // namespace

std::optional<std::string> validate(const VehicleCamGeneratorConfig & config)
{
  if (config.vehicle_role > etsi::kVehicleRoleMax) {
    return "vehicle_role must be in [0, 15]";
  }
  if (config.utm_zone.zone < 1 || config.utm_zone.zone > 60) {
    return "the UTM zone must be in [1, 60]";
  }
  return std::nullopt;
}

VehicleCamGenerator::VehicleCamGenerator(VehicleCamGeneratorConfig config)
: config_(std::move(config))
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument(*error);
  }
  status_.generation_interval = kGenCamMax;
}

void VehicleCamGenerator::setObject(const Object & object, Time measurement_time, Time now)
{
  object_ = object;
  measurement_time_ = measurement_time;
  ++status_.objects_received;
  status_.last_object_time = now;
}

void VehicleCamGenerator::clearObject()
{
  object_.reset();
}

VehicleCamGeneratorStatus VehicleCamGenerator::status() const
{
  auto result = status_;
  result.path_history_size = path_history_.size();
  return result;
}

bool VehicleCamGenerator::dynamicsChanged(const Object & object) const
{
  const Object & last = *last_cam_object_;
  return std::abs(normalizeAngle(object.yaw - last.yaw)) > kHeadingThreshold ||
         std::hypot(object.x - last.x, object.y - last.y) > kPositionThreshold ||
         std::abs(std::hypot(object.v_x, object.v_y) - std::hypot(last.v_x, last.v_y)) >
         kSpeedThreshold;
}

GenerationResult VehicleCamGenerator::check(Time now)
{
  GenerationResult result;
  if (!object_) {
    result.event = GenerationEvent::kNoObject;
    return result;
  }

  if (status_.last_generation_time && now < *status_.last_generation_time) {
    // The clock jumped backwards (e.g. a restarted recording): start over.
    status_.last_generation_time.reset();
    status_.generation_interval = kGenCamMax;
    consecutive_timeouts_ = 0;
  }

  // Generation conditions.
  if (!status_.last_generation_time) {
    result.trigger = GenerationTrigger::kFirst;
  } else {
    const Duration elapsed = now - *status_.last_generation_time;
    if (elapsed >= kGenCamMin && dynamicsChanged(*object_)) {
      result.trigger = GenerationTrigger::kDynamics;
    } else if (elapsed >= status_.generation_interval) {
      result.trigger = GenerationTrigger::kTimeout;
    } else {
      result.event = GenerationEvent::kNotDue;
      return result;
    }
  }

  // The object is used at most once, also if it cannot be encoded.
  const Object object = *std::exchange(object_, std::nullopt);
  const Time time = measurement_time_ + config_.time_shift;
  const auto generation_delta_time = generationDeltaTime(time);
  if (!generation_delta_time) {
    ++status_.invalid_time;
    result.event = GenerationEvent::kInvalidTime;
    return result;
  }
  const auto reference_position = encodeReferencePosition(object, config_.utm_zone);
  if (!reference_position) {
    ++status_.invalid_position;
    result.event = GenerationEvent::kInvalidPosition;
    return result;
  }

  Cam cam;
  cam.station_id = object.id;
  cam.generation_delta_time = *generation_delta_time;
  cam.station_type = encodeStationType(object);
  cam.reference_position = *reference_position;
  cam.vehicle_high_frequency = encodeVehicleHighFrequency(object);
  if (!last_low_frequency_time_ || now - *last_low_frequency_time_ >= kLowFrequencyInterval ||
    now < *last_low_frequency_time_)
  {
    CamVehicleLowFrequency low_frequency;
    low_frequency.vehicle_role = config_.vehicle_role;
    low_frequency.path_history = pathHistory(*reference_position, time);
    cam.vehicle_low_frequency = std::move(low_frequency);
    last_low_frequency_time_ = now;
    ++status_.low_frequency_containers;
  }
  addToPathHistory(*reference_position, time);

  // Update T_GenCam.
  switch (result.trigger) {
    case GenerationTrigger::kDynamics:
      status_.generation_interval =
        std::clamp(now - *status_.last_generation_time, kGenCamMin, kGenCamMax);
      consecutive_timeouts_ = 0;
      ++status_.triggered_by_dynamics;
      break;
    case GenerationTrigger::kTimeout:
      if (++consecutive_timeouts_ >= kNGenCam) {
        status_.generation_interval = kGenCamMax;
      }
      ++status_.triggered_by_timeout;
      break;
    case GenerationTrigger::kFirst:
    case GenerationTrigger::kNone:
      break;
  }

  last_cam_object_ = object;
  status_.last_generation_time = now;
  ++status_.cams_generated;
  result.event = GenerationEvent::kGenerated;
  result.cam = std::move(cam);
  return result;
}

std::vector<CamPathPoint> VehicleCamGenerator::pathHistory(
  const CamReferencePosition & reference, Time time) const
{
  // Each path point is the position of an older CAM relative to the preceding (younger) point,
  // starting at the reference position (ETSI TS 102 894-2, DF_PathHistory).
  std::vector<CamPathPoint> result;
  std::int32_t latitude = reference.latitude;
  std::int32_t longitude = reference.longitude;
  Time point_time = time;
  for (const auto & entry : path_history_) {
    CamPathPoint point;
    point.delta_latitude = entry.latitude - latitude;
    point.delta_longitude = entry.longitude - longitude;
    const auto delta_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(point_time - entry.time).count() / 10;
    if (point.delta_latitude < etsi::kDeltaLatitudeMin ||
      point.delta_latitude > etsi::kDeltaLatitudeMax ||
      point.delta_longitude < etsi::kDeltaLongitudeMin ||
      point.delta_longitude > etsi::kDeltaLongitudeMax || delta_time < 0 ||
      delta_time > etsi::kPathDeltaTimeMax)
    {
      break;  // The remaining points cannot be encoded.
    }
    point.delta_time = static_cast<std::uint16_t>(
      std::max<std::int64_t>(etsi::kPathDeltaTimeMin, delta_time));
    result.push_back(point);
    latitude = entry.latitude;
    longitude = entry.longitude;
    point_time = entry.time;
  }
  return result;
}

void VehicleCamGenerator::addToPathHistory(const CamReferencePosition & reference, Time time)
{
  if (!path_history_.empty() && time < path_history_.front().time) {
    // Time jumped backwards (e.g. a restarted recording), the history is no longer valid.
    path_history_.clear();
  }
  path_history_.push_front({reference.latitude, reference.longitude, time});
  if (path_history_.size() > etsi::kPathHistoryMaxSize) {
    path_history_.pop_back();
  }
}

}  // namespace ufil_etsi
