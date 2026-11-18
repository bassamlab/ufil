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

#ifndef UFIL_ETSI__CAM_HPP_
#define UFIL_ETSI__CAM_HPP_

// ROS-independent representation of the parts of an ETSI CAM (EN 302 637-2 V1.4.1, data
// elements of TS 102 894-2) that UFIL generates and evaluates. Values are in the ETSI encoding
// (integer units, special values for "unavailable"). The other fields of a CAM are constant or
// not supported and are set by the ROS 2 adapter when it builds the message.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace ufil_etsi
{

/// ETSI data element constants used by the encoding and decoding.
namespace etsi
{

namespace station_type
{
inline constexpr std::uint8_t kUnknown = 0;
inline constexpr std::uint8_t kPedestrian = 1;
inline constexpr std::uint8_t kCyclist = 2;
inline constexpr std::uint8_t kMoped = 3;
inline constexpr std::uint8_t kMotorcycle = 4;
inline constexpr std::uint8_t kPassengerCar = 5;
inline constexpr std::uint8_t kBus = 6;
inline constexpr std::uint8_t kLightTruck = 7;
inline constexpr std::uint8_t kHeavyTruck = 8;
inline constexpr std::uint8_t kTrailer = 9;
inline constexpr std::uint8_t kSpecialVehicles = 10;
inline constexpr std::uint8_t kTram = 11;
inline constexpr std::uint8_t kRoadSideUnit = 15;
}  // namespace station_type

inline constexpr std::int32_t kLatitudeUnavailable = 900000001;
inline constexpr std::int32_t kLongitudeUnavailable = 1800000001;
inline constexpr std::int32_t kAltitudeMin = -100000;
inline constexpr std::int32_t kAltitudeMax = 800000;
inline constexpr std::int32_t kAltitudeUnavailable = 800001;
inline constexpr std::uint8_t kAltitudeConfidence1Cm = 0;
inline constexpr std::uint8_t kAltitudeConfidenceUnavailable = 15;
inline constexpr std::uint16_t kSemiAxisLengthMin = 1;
inline constexpr std::uint16_t kSemiAxisLengthOutOfRange = 4094;
inline constexpr std::uint16_t kSemiAxisLengthUnavailable = 4095;

inline constexpr std::uint16_t kHeadingNorth = 0;
inline constexpr std::uint16_t kHeadingUnavailable = 3601;
inline constexpr std::uint8_t kHeadingConfidenceMin = 1;
inline constexpr std::uint8_t kHeadingConfidenceOutOfRange = 126;
inline constexpr std::uint8_t kHeadingConfidenceUnavailable = 127;

inline constexpr std::uint16_t kSpeedMax = 16382;
inline constexpr std::uint16_t kSpeedUnavailable = 16383;
inline constexpr std::uint8_t kSpeedConfidenceMin = 1;
inline constexpr std::uint8_t kSpeedConfidenceOutOfRange = 126;
inline constexpr std::uint8_t kSpeedConfidenceUnavailable = 127;

inline constexpr std::uint8_t kDriveDirectionForward = 0;
inline constexpr std::uint8_t kDriveDirectionBackward = 1;
inline constexpr std::uint8_t kDriveDirectionUnavailable = 2;

inline constexpr std::uint16_t kVehicleLengthMin = 1;
inline constexpr std::uint16_t kVehicleLengthOutOfRange = 1022;
inline constexpr std::uint16_t kVehicleLengthUnavailable = 1023;
inline constexpr std::uint8_t kNoTrailerPresent = 0;
inline constexpr std::uint8_t kTrailerPresentWithKnownLength = 1;
inline constexpr std::uint8_t kTrailerPresentWithUnknownLength = 2;
inline constexpr std::uint8_t kTrailerPresenceIsUnknown = 3;
inline constexpr std::uint8_t kVehicleLengthConfidenceUnavailable = 4;
inline constexpr std::uint8_t kVehicleWidthMin = 1;
inline constexpr std::uint8_t kVehicleWidthOutOfRange = 61;
inline constexpr std::uint8_t kVehicleWidthUnavailable = 62;

inline constexpr std::int16_t kAccelerationMin = -160;
inline constexpr std::int16_t kAccelerationMax = 160;
inline constexpr std::int16_t kAccelerationUnavailable = 161;
inline constexpr std::uint8_t kAccelerationConfidenceMin = 1;
inline constexpr std::uint8_t kAccelerationConfidenceOutOfRange = 101;
inline constexpr std::uint8_t kAccelerationConfidenceUnavailable = 102;

inline constexpr std::int16_t kYawRateMin = -32766;
inline constexpr std::int16_t kYawRateMax = 32766;
inline constexpr std::int16_t kYawRateUnavailable = 32767;

/// YawRateConfidence classes [deg/s].
enum class YawRateConfidence : std::uint8_t
{
  kDegSec000_01 = 0,
  kDegSec000_05,
  kDegSec000_10,
  kDegSec001_00,
  kDegSec005_00,
  kDegSec010_00,
  kDegSec100_00,
  kOutOfRange,
  kUnavailable,
};

inline constexpr std::uint8_t kVehicleRoleMax = 15;

inline constexpr std::int32_t kDeltaLatitudeMin = -131071;
inline constexpr std::int32_t kDeltaLatitudeMax = 131071;
inline constexpr std::int32_t kDeltaLongitudeMin = -131071;
inline constexpr std::int32_t kDeltaLongitudeMax = 131071;
inline constexpr std::int16_t kDeltaAltitudeUnavailable = 12800;
inline constexpr std::uint16_t kPathDeltaTimeMin = 1;
inline constexpr std::uint16_t kPathDeltaTimeMax = 65535;
/// Maximum number of path points in a CAM path history (EN 302 637-2 V1.4.1).
inline constexpr std::size_t kPathHistoryMaxSize = 23;

}  // namespace etsi

/// Factor between the standard deviation and the 95 % confidence of a normal distribution,
/// used for all confidences of a CAM.
inline constexpr double kConfidenceFactor = 1.96;

/// Reference position of the basic container.
struct CamReferencePosition
{
  std::int32_t latitude = etsi::kLatitudeUnavailable;  ///< [0.1 microdegree]
  std::int32_t longitude = etsi::kLongitudeUnavailable;  ///< [0.1 microdegree]
  std::uint16_t semi_major_confidence = etsi::kSemiAxisLengthUnavailable;  ///< [cm]
  std::uint16_t semi_minor_confidence = etsi::kSemiAxisLengthUnavailable;  ///< [cm]
  std::uint16_t semi_major_orientation = etsi::kHeadingUnavailable;  ///< [0.1 deg] from north
  std::int32_t altitude = etsi::kAltitudeUnavailable;  ///< [cm]
  std::uint8_t altitude_confidence = etsi::kAltitudeConfidenceUnavailable;
};

/// Lateral acceleration, optional in the high-frequency container.
struct CamLateralAcceleration
{
  std::int16_t value = etsi::kAccelerationUnavailable;  ///< [0.1 m/s^2], positive to the left
  std::uint8_t confidence = etsi::kAccelerationConfidenceUnavailable;  ///< [0.1 m/s^2]
};

/// Basic vehicle container high frequency.
struct CamVehicleHighFrequency
{
  std::uint16_t heading = etsi::kHeadingUnavailable;  ///< [0.1 deg], clockwise from north
  std::uint8_t heading_confidence = etsi::kHeadingConfidenceUnavailable;  ///< [0.1 deg]
  std::uint16_t speed = etsi::kSpeedUnavailable;  ///< [cm/s]
  std::uint8_t speed_confidence = etsi::kSpeedConfidenceUnavailable;  ///< [cm/s]
  std::uint8_t drive_direction = etsi::kDriveDirectionUnavailable;
  std::uint16_t vehicle_length = etsi::kVehicleLengthUnavailable;  ///< [0.1 m]
  std::uint8_t vehicle_length_confidence_indication = etsi::kVehicleLengthConfidenceUnavailable;
  std::uint8_t vehicle_width = etsi::kVehicleWidthUnavailable;  ///< [0.1 m]
  std::int16_t longitudinal_acceleration = etsi::kAccelerationUnavailable;  ///< [0.1 m/s^2]
  std::uint8_t longitudinal_acceleration_confidence = etsi::kAccelerationConfidenceUnavailable;
  std::optional<CamLateralAcceleration> lateral_acceleration;
  std::int16_t yaw_rate = etsi::kYawRateUnavailable;  ///< [0.01 deg/s], positive to the left
  etsi::YawRateConfidence yaw_rate_confidence = etsi::YawRateConfidence::kUnavailable;
};

/// Path point of the path history, relative to the preceding point.
struct CamPathPoint
{
  std::int32_t delta_latitude = 0;  ///< [0.1 microdegree]
  std::int32_t delta_longitude = 0;  ///< [0.1 microdegree]
  std::int16_t delta_altitude = etsi::kDeltaAltitudeUnavailable;  ///< [cm]
  std::optional<std::uint16_t> delta_time;  ///< [10 ms]
};

/// Basic vehicle container low frequency. Exterior lights are not supported (all off).
struct CamVehicleLowFrequency
{
  std::uint8_t vehicle_role = 0;
  std::vector<CamPathPoint> path_history;
};

/**
 * @brief The UFIL relevant content of a CAM.
 *
 * A vehicle CAM has a vehicle_high_frequency container, a CAM without it is a road side unit CAM
 * (RSU high-frequency container without protected communication zones).
 */
struct Cam
{
  std::uint32_t station_id = 0;
  std::uint16_t generation_delta_time = 0;  ///< [ms], TimestampIts modulo 65536
  std::uint8_t station_type = etsi::station_type::kUnknown;
  CamReferencePosition reference_position;
  std::optional<CamVehicleHighFrequency> vehicle_high_frequency;
  std::optional<CamVehicleLowFrequency> vehicle_low_frequency;
};

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__CAM_HPP_
