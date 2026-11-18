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

#include "ufil_etsi/cam_encoding.hpp"

#include <numbers>  // cpplint treats this C++20 header as a C system header

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

#include "covariance.hpp"

namespace ufil_etsi
{

namespace
{

/**
 * Encodes the 95 % confidence of a variance in the unit 1 / scale: truncated, at least min,
 * out_of_range if it does not fit, unavailable for an unknown (negative) variance.
 */
template<typename T>
T encodeConfidence(double variance, double scale, T min, T out_of_range, T unavailable)
{
  if (!(variance >= 0.0)) {
    return unavailable;
  }
  const double confidence = kConfidenceFactor * std::sqrt(variance) * scale;
  if (confidence >= out_of_range) {
    return out_of_range;
  }
  return std::max(min, static_cast<T>(confidence));
}

/// Truncates value * scale and caps it to [min, max]; unavailable if the value is not finite.
template<typename T>
T encodeTruncated(double value, double scale, T min, T max, T unavailable)
{
  if (!std::isfinite(value)) {
    return unavailable;
  }
  return static_cast<T>(std::clamp(std::trunc(value * scale), static_cast<double>(min),
      static_cast<double>(max)));
}

}  // namespace

std::uint8_t encodeStationType(const Object & object)
{
  switch (mostLikelyClass(object)) {
    case ObjectClass::kCar:
      return etsi::station_type::kPassengerCar;
    case ObjectClass::kTruck:
      return etsi::station_type::kLightTruck;
    case ObjectClass::kMotorcycle:
      return etsi::station_type::kMotorcycle;
    case ObjectClass::kBicycle:
      return etsi::station_type::kCyclist;
    case ObjectClass::kPedestrian:
      return etsi::station_type::kPedestrian;
    case ObjectClass::kStationary:
      return etsi::station_type::kRoadSideUnit;
    case ObjectClass::kOther:
      break;
  }
  return etsi::station_type::kUnknown;
}

std::optional<CamReferencePosition> encodeReferencePosition(const Object & object, UtmZone zone)
{
  // The CAM reference point is the centre of the front of the vehicle. UFIL uses the object
  // centre on both the generating and the receiving side.
  const auto position = utmToGeo(object.x, object.y, zone);
  if (!position) {
    return std::nullopt;
  }

  CamReferencePosition result;
  result.latitude = static_cast<std::int32_t>(std::llround(position->latitude * 1e7));
  result.longitude = static_cast<std::int32_t>(std::llround(position->longitude * 1e7));
  // The object has no altitude.
  result.altitude = etsi::kAltitudeUnavailable;
  result.altitude_confidence = etsi::kAltitudeConfidenceUnavailable;

  const double var_east = object.variance(kX);
  const double var_north = object.variance(kY);
  if (!(var_east >= 0.0 && var_north >= 0.0)) {
    return result;  // Confidence ellipse unavailable.
  }
  const auto ellipse = principalAxes(var_east, object.covarianceAt(kX, kY), var_north);
  const auto encode_axis = [](double variance) {
      return encodeConfidence<std::uint16_t>(
        variance, 100.0, etsi::kSemiAxisLengthMin, etsi::kSemiAxisLengthOutOfRange,
        etsi::kSemiAxisLengthUnavailable);
    };
  result.semi_major_confidence = encode_axis(ellipse.major_variance);
  result.semi_minor_confidence = encode_axis(ellipse.minor_variance);
  // The heading of the major axis, clockwise from north, in [0, 180) deg.
  double orientation_deg = 90.0 - ellipse.major_angle * 180.0 / std::numbers::pi;
  orientation_deg = std::fmod(std::fmod(orientation_deg, 180.0) + 180.0, 180.0);
  result.semi_major_orientation =
    static_cast<std::uint16_t>(std::min(1799.0, std::round(orientation_deg * 10.0)));
  return result;
}

std::uint16_t encodeHeading(double yaw)
{
  if (!std::isfinite(yaw)) {
    return etsi::kHeadingUnavailable;
  }
  // [0, 360) deg counter-clockwise from east, then clockwise from north.
  double yaw_deg = std::fmod(yaw * 180.0 / std::numbers::pi, 360.0);
  if (yaw_deg < 0.0) {
    yaw_deg += 360.0;
  }
  // Rounded to the nearest 0.1 deg.
  const auto yaw_decidegree = static_cast<int>(std::lround(yaw_deg * 10.0));
  return static_cast<std::uint16_t>(((900 - yaw_decidegree) % 3600 + 3600) % 3600);
}

std::uint8_t encodeHeadingConfidence(double yaw_variance)
{
  return encodeConfidence<std::uint8_t>(
    yaw_variance, 1800.0 / std::numbers::pi, etsi::kHeadingConfidenceMin,
    etsi::kHeadingConfidenceOutOfRange, etsi::kHeadingConfidenceUnavailable);
}

std::uint16_t encodeSpeed(double speed)
{
  return encodeTruncated<std::uint16_t>(
    std::abs(speed), 100.0, 0, etsi::kSpeedMax, etsi::kSpeedUnavailable);
}

std::uint8_t encodeSpeedConfidence(double speed_variance)
{
  return encodeConfidence<std::uint8_t>(
    speed_variance, 100.0, etsi::kSpeedConfidenceMin, etsi::kSpeedConfidenceOutOfRange,
    etsi::kSpeedConfidenceUnavailable);
}

std::uint8_t encodeDriveDirection(double yaw, double v_x, double v_y)
{
  if (!std::isfinite(yaw) || !std::isfinite(v_x) || !std::isfinite(v_y)) {
    return etsi::kDriveDirectionUnavailable;
  }
  // A standing vehicle is driving forward. Otherwise compare the direction of the velocity
  // with the heading.
  return v_x * std::cos(yaw) + v_y * std::sin(yaw) >= 0.0 ?
         etsi::kDriveDirectionForward : etsi::kDriveDirectionBackward;
}

std::uint16_t encodeVehicleLength(double length)
{
  return encodeTruncated<std::uint16_t>(
    length, 10.0, etsi::kVehicleLengthMin, etsi::kVehicleLengthOutOfRange,
    etsi::kVehicleLengthUnavailable);
}

std::uint8_t encodeVehicleWidth(double width)
{
  return encodeTruncated<std::uint8_t>(
    width, 10.0, etsi::kVehicleWidthMin, etsi::kVehicleWidthOutOfRange,
    etsi::kVehicleWidthUnavailable);
}

std::int16_t encodeAcceleration(double acceleration)
{
  if (!std::isfinite(acceleration)) {
    return etsi::kAccelerationUnavailable;
  }
  // Rounded to the nearest 0.1 m/s^2 step.
  return static_cast<std::int16_t>(
    std::clamp(
      std::round(acceleration * 10.0), double{etsi::kAccelerationMin},
      double{etsi::kAccelerationMax}));
}

std::uint8_t encodeAccelerationConfidence(double acceleration_variance)
{
  return encodeConfidence<std::uint8_t>(
    acceleration_variance, 10.0, etsi::kAccelerationConfidenceMin,
    etsi::kAccelerationConfidenceOutOfRange, etsi::kAccelerationConfidenceUnavailable);
}

std::int16_t encodeYawRate(double yaw_rate)
{
  return encodeTruncated<std::int16_t>(
    yaw_rate, 18000.0 / std::numbers::pi, etsi::kYawRateMin, etsi::kYawRateMax,
    etsi::kYawRateUnavailable);
}

etsi::YawRateConfidence encodeYawRateConfidence(double yaw_rate_variance)
{
  using etsi::YawRateConfidence;
  if (!(yaw_rate_variance >= 0.0)) {
    return YawRateConfidence::kUnavailable;
  }
  const double confidence_deg_s =
    kConfidenceFactor * std::sqrt(yaw_rate_variance) * 180.0 / std::numbers::pi;
  if (confidence_deg_s <= 0.01) {
    return YawRateConfidence::kDegSec000_01;
  } else if (confidence_deg_s <= 0.05) {
    return YawRateConfidence::kDegSec000_05;
  } else if (confidence_deg_s <= 0.1) {
    return YawRateConfidence::kDegSec000_10;
  } else if (confidence_deg_s <= 1.0) {
    return YawRateConfidence::kDegSec001_00;
  } else if (confidence_deg_s <= 5.0) {
    return YawRateConfidence::kDegSec005_00;
  } else if (confidence_deg_s <= 10.0) {
    return YawRateConfidence::kDegSec010_00;
  } else if (confidence_deg_s <= 100.0) {
    return YawRateConfidence::kDegSec100_00;
  }
  return YawRateConfidence::kOutOfRange;
}

CamVehicleHighFrequency encodeVehicleHighFrequency(const Object & object)
{
  CamVehicleHighFrequency result;

  result.heading = encodeHeading(object.yaw);
  result.heading_confidence = encodeHeadingConfidence(object.variance(kYaw));

  // The CAM speed is the speed along the heading; project the global velocity covariance onto
  // the heading.
  const double cos_yaw = std::cos(object.yaw);
  const double sin_yaw = std::sin(object.yaw);
  const auto projected = [&object](StateIndex x, StateIndex y, double u_x, double u_y) {
      if (!(object.variance(x) >= 0.0 && object.variance(y) >= 0.0)) {
        return -1.0;
      }
      return projectedVariance(
        object.variance(x), object.covarianceAt(x, y), object.variance(y), u_x, u_y);
    };
  result.speed = encodeSpeed(std::hypot(object.v_x, object.v_y));
  result.speed_confidence = encodeSpeedConfidence(projected(kVx, kVy, cos_yaw, sin_yaw));
  result.drive_direction = encodeDriveDirection(object.yaw, object.v_x, object.v_y);

  // Trailers are not supported. Unknown dimensions (negative variance) are unavailable.
  if (object.dimension_covariance[0] >= 0.0) {
    result.vehicle_length = encodeVehicleLength(object.length);
    result.vehicle_length_confidence_indication = etsi::kNoTrailerPresent;
  }
  if (object.dimension_covariance[4] >= 0.0) {
    result.vehicle_width = encodeVehicleWidth(object.width);
  }

  // Acceleration in the vehicle frame: along the heading h = (cos yaw, sin yaw) and its left
  // normal l = (-sin yaw, cos yaw).
  result.longitudinal_acceleration =
    encodeAcceleration(cos_yaw * object.a_x + sin_yaw * object.a_y);
  result.longitudinal_acceleration_confidence =
    encodeAccelerationConfidence(projected(kAx, kAy, cos_yaw, sin_yaw));
  CamLateralAcceleration lateral;
  lateral.value = encodeAcceleration(-sin_yaw * object.a_x + cos_yaw * object.a_y);
  lateral.confidence = encodeAccelerationConfidence(projected(kAx, kAy, -sin_yaw, cos_yaw));
  result.lateral_acceleration = lateral;

  // Positive yaw rates turn to the left in both ROS and ETSI.
  result.yaw_rate = encodeYawRate(object.yaw_rate);
  result.yaw_rate_confidence = encodeYawRateConfidence(object.variance(kYawRate));
  return result;
}

}  // namespace ufil_etsi
