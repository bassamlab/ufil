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

#ifndef UFIL_ETSI__CAM_ENCODING_HPP_
#define UFIL_ETSI__CAM_ENCODING_HPP_

// Encoding of a UFIL object into the ETSI data elements of a CAM.
//
// Confidences are given with a confidence level of 95 %, assuming a normal distribution
// (1.96 sigma). Values outside the range of a data element are capped; confidences that are
// too large are encoded as "out of range", unknown quantities (negative variance) as
// "unavailable".

#include <cstdint>
#include <optional>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/types.hpp"
#include "ufil_etsi/visibility_control.hpp"

namespace ufil_etsi
{

/// Station type of the most likely class of the object.
UFIL_ETSI_PUBLIC std::uint8_t encodeStationType(const Object & object);

/**
 * @brief Reference position of the object.
 *
 * The UFIL object position (the centre of the object) is used as reference position. The
 * confidence ellipse is the 95 % ellipse of the position covariance. Returns std::nullopt if
 * the position cannot be converted into WGS84.
 */
UFIL_ETSI_PUBLIC std::optional<CamReferencePosition> encodeReferencePosition(
  const Object & object, UtmZone zone);

/// Heading [0.1 deg, clockwise from north] of a yaw [rad, counter-clockwise from east].
UFIL_ETSI_PUBLIC std::uint16_t encodeHeading(double yaw);

/// Heading confidence of a yaw variance [rad^2].
UFIL_ETSI_PUBLIC std::uint8_t encodeHeadingConfidence(double yaw_variance);

/// Speed [cm/s] of a speed [m/s] (absolute value).
UFIL_ETSI_PUBLIC std::uint16_t encodeSpeed(double speed);

/// Speed confidence of a speed variance [m^2/s^2].
UFIL_ETSI_PUBLIC std::uint8_t encodeSpeedConfidence(double speed_variance);

/// Drive direction: forward if the velocity points less than 90 deg away from the heading.
UFIL_ETSI_PUBLIC std::uint8_t encodeDriveDirection(double yaw, double v_x, double v_y);

/// Vehicle length [0.1 m] of a length [m].
UFIL_ETSI_PUBLIC std::uint16_t encodeVehicleLength(double length);

/// Vehicle width [0.1 m] of a width [m].
UFIL_ETSI_PUBLIC std::uint8_t encodeVehicleWidth(double width);

/// Longitudinal or lateral acceleration [0.1 m/s^2] of an acceleration [m/s^2].
UFIL_ETSI_PUBLIC std::int16_t encodeAcceleration(double acceleration);

/// Acceleration confidence of an acceleration variance [m^2/s^4].
UFIL_ETSI_PUBLIC std::uint8_t encodeAccelerationConfidence(double acceleration_variance);

/// Yaw rate [0.01 deg/s] of a yaw rate [rad/s].
UFIL_ETSI_PUBLIC std::int16_t encodeYawRate(double yaw_rate);

/// Yaw rate confidence class of a yaw rate variance [rad^2/s^2].
UFIL_ETSI_PUBLIC etsi::YawRateConfidence encodeYawRateConfidence(double yaw_rate_variance);

/**
 * @brief Basic vehicle container high frequency of the object.
 *
 * Speed and acceleration are projected into the vehicle frame (x along the heading, y to the
 * left), as the UFIL object state is global. Trailers are not supported.
 */
UFIL_ETSI_PUBLIC CamVehicleHighFrequency encodeVehicleHighFrequency(const Object & object);

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__CAM_ENCODING_HPP_
