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

#ifndef UFIL_ETSI__CAM_DECODING_HPP_
#define UFIL_ETSI__CAM_DECODING_HPP_

// Decoding of the ETSI data elements of a vehicle CAM into a UFIL object.
//
// Unavailable values are reported as unknown (value 0, variance -1). Values without a usable
// confidence get a variance of 0. Confidences are 95 % values of a normal distribution.

#include <cstdint>
#include <optional>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/types.hpp"
#include "ufil_etsi/visibility_control.hpp"

namespace ufil_etsi
{

/// Value with its variance; a negative variance marks an unknown value.
struct ValueWithVariance
{
  double value = 0.0;
  double variance = -1.0;
};

/// Yaw [rad, counter-clockwise from east, in (-pi, pi]] of a heading.
UFIL_ETSI_PUBLIC ValueWithVariance decodeYaw(std::uint16_t heading, std::uint8_t confidence);

/// Signed speed along the heading [m/s], negative when driving backwards.
UFIL_ETSI_PUBLIC ValueWithVariance decodeSpeed(
  std::uint16_t speed, std::uint8_t confidence, std::uint8_t drive_direction);

/// Acceleration [m/s^2].
UFIL_ETSI_PUBLIC ValueWithVariance decodeAcceleration(std::int16_t value, std::uint8_t confidence);

/// Yaw rate [rad/s]. The variance uses the middle of the confidence class.
UFIL_ETSI_PUBLIC ValueWithVariance decodeYawRate(
  std::int16_t yaw_rate, etsi::YawRateConfidence confidence);

/// UFIL classification of a station type. Similar station types are merged into one class.
UFIL_ETSI_PUBLIC ObjectClass classFromStationType(std::uint8_t station_type);

/// True for the station types of vehicles (moped up to special vehicles).
UFIL_ETSI_PUBLIC bool isVehicleStationType(std::uint8_t station_type);

/// Outcome of decodeVehicleCam.
enum class DecodeStatus
{
  kOk,
  kNotAVehicle,  ///< Station type is not a vehicle or the vehicle container is missing.
  kPositionUnavailable,  ///< Reference position is unavailable or invalid.
};

/// Result of decodeVehicleCam: the object in UTM and the UTM zone of the reference position.
struct DecodeResult
{
  DecodeStatus status = DecodeStatus::kOk;
  /// Only valid if status is kOk.
  Object object;
  UtmZone zone;
};

/**
 * @brief Converts a vehicle CAM into an object in UTM coordinates.
 *
 * The reference position is used as object position (the generator does the same), velocity
 * and acceleration are rotated from the vehicle frame into the global frame. The object id is
 * the station id, the existence probability is 1.
 */
UFIL_ETSI_PUBLIC DecodeResult decodeVehicleCam(const Cam & cam);

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__CAM_DECODING_HPP_
