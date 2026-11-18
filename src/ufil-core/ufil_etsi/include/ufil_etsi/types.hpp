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

#ifndef UFIL_ETSI__TYPES_HPP_
#define UFIL_ETSI__TYPES_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "ufil_etsi/visibility_control.hpp"

namespace ufil_etsi
{

/// Duration and point in time [ns]. Points in time are relative to the Unix epoch (ROS time).
using Duration = std::chrono::nanoseconds;
using Time = std::chrono::nanoseconds;

/// Object classes, in the order of the UFIL classification.
enum class ObjectClass : std::uint8_t
{
  kCar = 0,
  kTruck,
  kMotorcycle,
  kBicycle,
  kPedestrian,
  kStationary,
  kOther,
};
inline constexpr std::size_t kObjectClassCount = 7;

/// Index of a quantity in the state vector and the row-major 8x8 state covariance.
enum StateIndex : std::size_t
{
  kX = 0,
  kY,
  kVx,
  kVy,
  kAx,
  kAy,
  kYaw,
  kYawRate,
};
inline constexpr std::size_t kStateSize = 8;

/**
 * @brief Object in UTM coordinates (x east, y north, yaw counter-clockwise from east).
 *
 * Velocity and acceleration are global. A negative variance on the diagonal of a covariance
 * marks the quantity as unknown (UFIL convention).
 */
struct Object
{
  std::uint32_t id = 0;

  double x = 0.0;  ///< Easting [m]
  double y = 0.0;  ///< Northing [m]
  double v_x = 0.0;  ///< [m/s]
  double v_y = 0.0;  ///< [m/s]
  double a_x = 0.0;  ///< [m/s^2]
  double a_y = 0.0;  ///< [m/s^2]
  double yaw = 0.0;  ///< [rad]
  double yaw_rate = 0.0;  ///< [rad/s]
  /// Row-major covariance of (x, y, v_x, v_y, a_x, a_y, yaw, yaw_rate).
  std::array<double, kStateSize * kStateSize> covariance{};

  double length = 0.0;  ///< [m]
  double width = 0.0;  ///< [m]
  double height = 0.0;  ///< [m]
  /// Row-major covariance of (length, width, height).
  std::array<double, 9> dimension_covariance{};

  double existence_probability = 0.0;
  /// Probability per ObjectClass.
  std::array<double, kObjectClassCount> classification{};

  double variance(StateIndex index) const {return covariance[index * kStateSize + index];}
  double covarianceAt(StateIndex row, StateIndex column) const
  {
    return covariance[row * kStateSize + column];
  }
  void setCovariance(StateIndex row, StateIndex column, double value)
  {
    covariance[row * kStateSize + column] = value;
  }
};

/// Most likely class of the object, kOther if the classification is empty.
UFIL_ETSI_PUBLIC ObjectClass mostLikelyClass(const Object & object);

/// UTM zone, encoded in UFIL frame ids as "utm_<zone><n|s>", e.g. "utm_32n".
struct UtmZone
{
  int zone = 32;
  bool northern = true;

  bool operator==(const UtmZone &) const = default;
};

/// Parses a frame id like "utm_32n". Returns std::nullopt if the frame is not a UTM frame.
UFIL_ETSI_PUBLIC std::optional<UtmZone> utmZoneFromFrame(const std::string & frame_id);

/// Frame id of a UTM zone, e.g. "utm_32n".
UFIL_ETSI_PUBLIC std::string frameFromUtmZone(const UtmZone & zone);

/// Geographic position (WGS84).
struct GeoPosition
{
  double latitude = 0.0;  ///< [deg]
  double longitude = 0.0;  ///< [deg]
};

/// Converts a UTM position into WGS84. Returns std::nullopt if the position is not valid.
UFIL_ETSI_PUBLIC std::optional<GeoPosition> utmToGeo(double easting, double northing, UtmZone zone);

/// UTM position in the standard zone of a WGS84 position.
struct UtmPosition
{
  double easting = 0.0;  ///< [m]
  double northing = 0.0;  ///< [m]
  UtmZone zone;
};

/// Converts a WGS84 position into UTM. Returns std::nullopt if the position is not valid.
UFIL_ETSI_PUBLIC std::optional<UtmPosition> geoToUtm(const GeoPosition & position);

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__TYPES_HPP_
