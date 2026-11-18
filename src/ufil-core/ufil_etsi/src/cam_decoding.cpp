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

#include "ufil_etsi/cam_decoding.hpp"

#include <numbers>  // cpplint treats this C++20 header as a C system header

#include <cmath>
#include <cstdint>

#include "covariance.hpp"

namespace ufil_etsi
{

namespace
{

constexpr double kDegreesToRadians = std::numbers::pi / 180.0;

/// Variance of a 95 % confidence given in the unit 1 / scale.
double varianceFromConfidence(double confidence, double scale)
{
  const double sigma = confidence / kConfidenceFactor / scale;
  return sigma * sigma;
}

}  // namespace

ValueWithVariance decodeYaw(std::uint16_t heading, std::uint8_t confidence)
{
  if (heading > 3600) {
    return {};
  }
  // Clockwise from north to counter-clockwise from east.
  const int yaw_decidegree = ((900 - heading) % 3600 + 3600) % 3600;
  ValueWithVariance result{normalizeAngle(yaw_decidegree * 0.1 * kDegreesToRadians), 0.0};
  if (confidence >= etsi::kHeadingConfidenceMin &&
    confidence < etsi::kHeadingConfidenceOutOfRange)
  {
    result.variance = varianceFromConfidence(confidence * kDegreesToRadians, 10.0);
  }
  return result;
}

ValueWithVariance decodeSpeed(
  std::uint16_t speed, std::uint8_t confidence, std::uint8_t drive_direction)
{
  if (speed >= etsi::kSpeedUnavailable ||
    (drive_direction != etsi::kDriveDirectionForward &&
    drive_direction != etsi::kDriveDirectionBackward))
  {
    return {};
  }
  const double sign = drive_direction == etsi::kDriveDirectionForward ? 1.0 : -1.0;
  ValueWithVariance result{sign * speed / 100.0, 0.0};
  if (confidence >= etsi::kSpeedConfidenceMin && confidence < etsi::kSpeedConfidenceOutOfRange) {
    result.variance = varianceFromConfidence(confidence, 100.0);
  }
  return result;
}

ValueWithVariance decodeAcceleration(std::int16_t value, std::uint8_t confidence)
{
  if (value < etsi::kAccelerationMin || value > etsi::kAccelerationMax) {
    return {};
  }
  ValueWithVariance result{value / 10.0, 0.0};
  if (confidence < etsi::kAccelerationConfidenceOutOfRange) {
    result.variance = varianceFromConfidence(confidence, 10.0);
  }
  return result;
}

ValueWithVariance decodeYawRate(std::int16_t yaw_rate, etsi::YawRateConfidence confidence)
{
  using etsi::YawRateConfidence;
  if (yaw_rate < etsi::kYawRateMin || yaw_rate > etsi::kYawRateMax) {
    return {};
  }
  ValueWithVariance result{yaw_rate / 100.0 * kDegreesToRadians, 0.0};
  // Middle of the confidence class [deg/s].
  double confidence_deg_s = 0.0;
  switch (confidence) {
    case YawRateConfidence::kDegSec000_01:
      confidence_deg_s = 0.005;
      break;
    case YawRateConfidence::kDegSec000_05:
      confidence_deg_s = 0.03;
      break;
    case YawRateConfidence::kDegSec000_10:
      confidence_deg_s = 0.075;
      break;
    case YawRateConfidence::kDegSec001_00:
      confidence_deg_s = 0.55;
      break;
    case YawRateConfidence::kDegSec005_00:
      confidence_deg_s = 3.0;
      break;
    case YawRateConfidence::kDegSec010_00:
      confidence_deg_s = 7.5;
      break;
    case YawRateConfidence::kDegSec100_00:
      confidence_deg_s = 55.0;
      break;
    case YawRateConfidence::kOutOfRange:
    case YawRateConfidence::kUnavailable:
      return result;  // Unknown confidence, variance 0.
  }
  result.variance = varianceFromConfidence(confidence_deg_s * kDegreesToRadians, 1.0);
  return result;
}

ObjectClass classFromStationType(std::uint8_t station_type)
{
  namespace type = etsi::station_type;
  switch (station_type) {
    case type::kPassengerCar:
      return ObjectClass::kCar;
    case type::kLightTruck:
    case type::kHeavyTruck:
      return ObjectClass::kTruck;
    case type::kMoped:
    case type::kMotorcycle:
      return ObjectClass::kMotorcycle;
    case type::kCyclist:
      return ObjectClass::kBicycle;
    case type::kPedestrian:
      return ObjectClass::kPedestrian;
    case type::kRoadSideUnit:
      return ObjectClass::kStationary;
    default:
      return ObjectClass::kOther;
  }
}

bool isVehicleStationType(std::uint8_t station_type)
{
  return station_type >= etsi::station_type::kMoped &&
         station_type <= etsi::station_type::kSpecialVehicles;
}

DecodeResult decodeVehicleCam(const Cam & cam)
{
  DecodeResult result;
  if (!isVehicleStationType(cam.station_type) || !cam.vehicle_high_frequency) {
    result.status = DecodeStatus::kNotAVehicle;
    return result;
  }
  const auto & hf = *cam.vehicle_high_frequency;
  const auto & reference = cam.reference_position;
  Object & object = result.object;

  // Position
  if (reference.latitude == etsi::kLatitudeUnavailable ||
    reference.longitude == etsi::kLongitudeUnavailable)
  {
    result.status = DecodeStatus::kPositionUnavailable;
    return result;
  }
  const auto utm = geoToUtm({reference.latitude * 1e-7, reference.longitude * 1e-7});
  if (!utm) {
    result.status = DecodeStatus::kPositionUnavailable;
    return result;
  }
  result.zone = utm->zone;
  object.x = utm->easting;
  object.y = utm->northing;
  const auto valid_axis = [](std::uint16_t axis) {
      return axis < etsi::kSemiAxisLengthOutOfRange;
    };
  if (reference.semi_major_orientation <= 3600 && valid_axis(reference.semi_major_confidence) &&
    valid_axis(reference.semi_minor_confidence))
  {
    // The major axis is given clockwise from north, covarianceFromAxes expects the angle
    // counter-clockwise from east.
    const double orientation = reference.semi_major_orientation * 0.1 * kDegreesToRadians;
    const auto covariance = covarianceFromAxes(
      varianceFromConfidence(reference.semi_major_confidence, 100.0),
      varianceFromConfidence(reference.semi_minor_confidence, 100.0),
      std::numbers::pi / 2.0 - orientation);
    object.setCovariance(kX, kX, covariance[0]);
    object.setCovariance(kX, kY, covariance[1]);
    object.setCovariance(kY, kX, covariance[1]);
    object.setCovariance(kY, kY, covariance[2]);
  }

  // Yaw
  const auto yaw = decodeYaw(hf.heading, hf.heading_confidence);
  object.yaw = yaw.value;
  object.setCovariance(kYaw, kYaw, yaw.variance);
  const double cos_yaw = std::cos(object.yaw);
  const double sin_yaw = std::sin(object.yaw);

  // Rotates a vector and its diagonal covariance from the vehicle frame into the global frame.
  const auto set_global = [&](
    StateIndex x, StateIndex y, double & value_x, double & value_y, double longitudinal,
    double lateral, double var_longitudinal, double var_lateral) {
      value_x = cos_yaw * longitudinal - sin_yaw * lateral;
      value_y = sin_yaw * longitudinal + cos_yaw * lateral;
      const auto covariance = covarianceFromAxes(var_longitudinal, var_lateral, object.yaw);
      object.setCovariance(x, x, covariance[0]);
      object.setCovariance(x, y, covariance[1]);
      object.setCovariance(y, x, covariance[1]);
      object.setCovariance(y, y, covariance[2]);
    };
  const auto set_unknown = [&object](StateIndex x, StateIndex y) {
      object.setCovariance(x, x, -1.0);
      object.setCovariance(y, y, -1.0);
    };

  // Velocity: the CAM speed is along the heading; the lateral velocity is 0 with a tiny
  // variance to avoid instabilities.
  const auto speed = decodeSpeed(hf.speed, hf.speed_confidence, hf.drive_direction);
  if (speed.variance < 0.0) {
    set_unknown(kVx, kVy);
  } else if (speed.variance == 0.0) {
    set_global(kVx, kVy, object.v_x, object.v_y, speed.value, 0.0, 0.0, 0.0);
  } else {
    set_global(kVx, kVy, object.v_x, object.v_y, speed.value, 0.0, speed.variance, 1e-9);
  }

  // Acceleration, requires the longitudinal and the lateral acceleration.
  const auto longitudinal = decodeAcceleration(
    hf.longitudinal_acceleration, hf.longitudinal_acceleration_confidence);
  const auto lateral = hf.lateral_acceleration ?
    decodeAcceleration(hf.lateral_acceleration->value, hf.lateral_acceleration->confidence) :
    ValueWithVariance{};
  if (longitudinal.variance < 0.0 || lateral.variance < 0.0) {
    set_unknown(kAx, kAy);
  } else if (longitudinal.variance == 0.0 || lateral.variance == 0.0) {
    set_global(kAx, kAy, object.a_x, object.a_y, longitudinal.value, lateral.value, 0.0, 0.0);
  } else {
    set_global(
      kAx, kAy, object.a_x, object.a_y, longitudinal.value, lateral.value,
      longitudinal.variance, lateral.variance);
  }

  // Yaw rate
  const auto yaw_rate = decodeYawRate(hf.yaw_rate, hf.yaw_rate_confidence);
  object.yaw_rate = yaw_rate.value;
  object.setCovariance(kYawRate, kYawRate, yaw_rate.variance);

  // Dimension: the CAM resolution of 0.1 m is used as variance. The length is unknown if a
  // trailer of unknown length may be attached. The height is not transmitted.
  const bool length_known =
    hf.vehicle_length >= etsi::kVehicleLengthMin &&
    hf.vehicle_length < etsi::kVehicleLengthOutOfRange &&
    (hf.vehicle_length_confidence_indication == etsi::kNoTrailerPresent ||
    hf.vehicle_length_confidence_indication == etsi::kTrailerPresentWithKnownLength);
  if (length_known) {
    object.length = hf.vehicle_length / 10.0;
    object.dimension_covariance[0] = 0.1;
  } else {
    object.dimension_covariance[0] = -1.0;
  }
  if (hf.vehicle_width >= etsi::kVehicleWidthMin &&
    hf.vehicle_width < etsi::kVehicleWidthOutOfRange)
  {
    object.width = hf.vehicle_width / 10.0;
    object.dimension_covariance[4] = 0.1;
  } else {
    object.dimension_covariance[4] = -1.0;
  }
  object.dimension_covariance[8] = -1.0;

  object.id = cam.station_id;
  // A CAM was just received from the station, so it exists.
  object.existence_probability = 1.0;
  object.classification[static_cast<std::size_t>(classFromStationType(cam.station_type))] = 1.0;
  return result;
}

}  // namespace ufil_etsi
