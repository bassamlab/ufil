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

#include <gtest/gtest.h>

#include <numbers>  // cpplint treats this C++20 header as a C system header

#include <cmath>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/cam_encoding.hpp"
#include "ufil_etsi/types.hpp"

// Core library tests, no ROS 2 required.

using ufil_etsi::Object;
using ufil_etsi::ObjectClass;
using ufil_etsi::UtmZone;
namespace etsi = ufil_etsi::etsi;

namespace
{

constexpr double kPi = std::numbers::pi;

Object objectOfClass(ObjectClass object_class)
{
  Object object;
  object.classification[static_cast<std::size_t>(object_class)] = 0.9;
  return object;
}

}  // namespace

TEST(CamEncoding, HeadingIsClockwiseFromNorth)
{
  EXPECT_EQ(ufil_etsi::encodeHeading(0.0), 900);  // east
  EXPECT_EQ(ufil_etsi::encodeHeading(kPi / 2.0), 0);  // north
  EXPECT_EQ(ufil_etsi::encodeHeading(kPi), 2700);  // west
  EXPECT_EQ(ufil_etsi::encodeHeading(-kPi / 2.0), 1800);  // south
  EXPECT_EQ(ufil_etsi::encodeHeading(2.5 * kPi), 0);  // wraps
  EXPECT_EQ(ufil_etsi::encodeHeading(NAN), etsi::kHeadingUnavailable);
}

TEST(CamEncoding, ConfidencesAreCappedAndUnknownIsUnavailable)
{
  // sigma 1 deg -> 1.96 deg -> 19 (0.1 deg)
  const double one_degree = kPi / 180.0;
  EXPECT_EQ(ufil_etsi::encodeHeadingConfidence(one_degree * one_degree), 19);
  EXPECT_EQ(ufil_etsi::encodeHeadingConfidence(0.0), etsi::kHeadingConfidenceMin);
  // sigma 15 deg used to overflow the 8 bit value, now out of range.
  EXPECT_EQ(
    ufil_etsi::encodeHeadingConfidence(std::pow(15.0 * one_degree, 2)),
    etsi::kHeadingConfidenceOutOfRange);
  EXPECT_EQ(ufil_etsi::encodeHeadingConfidence(-1.0), etsi::kHeadingConfidenceUnavailable);

  EXPECT_EQ(ufil_etsi::encodeSpeedConfidence(100.0), etsi::kSpeedConfidenceOutOfRange);
  EXPECT_EQ(ufil_etsi::encodeSpeedConfidence(-1.0), etsi::kSpeedConfidenceUnavailable);
  EXPECT_EQ(
    ufil_etsi::encodeAccelerationConfidence(100.0), etsi::kAccelerationConfidenceOutOfRange);
  EXPECT_EQ(
    ufil_etsi::encodeAccelerationConfidence(-1.0), etsi::kAccelerationConfidenceUnavailable);
}

TEST(CamEncoding, ValuesAreCappedToTheirRange)
{
  EXPECT_EQ(ufil_etsi::encodeSpeed(12.345), 1234);
  EXPECT_EQ(ufil_etsi::encodeSpeed(-2.0), 200);
  EXPECT_EQ(ufil_etsi::encodeSpeed(1000.0), etsi::kSpeedMax);
  EXPECT_EQ(ufil_etsi::encodeVehicleLength(4.56), 45);
  EXPECT_EQ(ufil_etsi::encodeVehicleLength(0.0), etsi::kVehicleLengthMin);
  EXPECT_EQ(ufil_etsi::encodeVehicleLength(500.0), etsi::kVehicleLengthOutOfRange);
  EXPECT_EQ(ufil_etsi::encodeVehicleWidth(1.84), 18);
  EXPECT_EQ(ufil_etsi::encodeVehicleWidth(10.0), etsi::kVehicleWidthOutOfRange);
  EXPECT_EQ(ufil_etsi::encodeAcceleration(-1.26), -13);
  EXPECT_EQ(ufil_etsi::encodeAcceleration(50.0), etsi::kAccelerationMax);
  EXPECT_EQ(ufil_etsi::encodeAcceleration(-50.0), etsi::kAccelerationMin);
  // 0.1 rad/s = 5.73 deg/s
  EXPECT_EQ(ufil_etsi::encodeYawRate(0.1), 572);
  EXPECT_EQ(ufil_etsi::encodeYawRate(-0.1), -572);
  EXPECT_EQ(ufil_etsi::encodeYawRate(100.0), etsi::kYawRateMax);
  EXPECT_EQ(ufil_etsi::encodeYawRate(-100.0), etsi::kYawRateMin);
}

TEST(CamEncoding, YawRateConfidenceClasses)
{
  using etsi::YawRateConfidence;
  const auto variance = [](double confidence_deg_s) {
      const double sigma = confidence_deg_s / ufil_etsi::kConfidenceFactor * kPi / 180.0;
      return sigma * sigma;
    };
  EXPECT_EQ(ufil_etsi::encodeYawRateConfidence(0.0), YawRateConfidence::kDegSec000_01);
  EXPECT_EQ(ufil_etsi::encodeYawRateConfidence(variance(0.04)), YawRateConfidence::kDegSec000_05);
  // Confidences below 1 deg/s used to be truncated to 0 and reported as 0.01 deg/s.
  EXPECT_EQ(ufil_etsi::encodeYawRateConfidence(variance(0.5)), YawRateConfidence::kDegSec001_00);
  EXPECT_EQ(ufil_etsi::encodeYawRateConfidence(variance(3.0)), YawRateConfidence::kDegSec005_00);
  EXPECT_EQ(ufil_etsi::encodeYawRateConfidence(variance(50.0)), YawRateConfidence::kDegSec100_00);
  EXPECT_EQ(ufil_etsi::encodeYawRateConfidence(variance(500.0)), YawRateConfidence::kOutOfRange);
  EXPECT_EQ(ufil_etsi::encodeYawRateConfidence(-1.0), YawRateConfidence::kUnavailable);
}

TEST(CamEncoding, DriveDirection)
{
  EXPECT_EQ(ufil_etsi::encodeDriveDirection(0.0, 5.0, 0.0), etsi::kDriveDirectionForward);
  EXPECT_EQ(ufil_etsi::encodeDriveDirection(0.0, -5.0, 0.0), etsi::kDriveDirectionBackward);
  // Heading west, driving east: backwards (was forward because of a missing absolute value).
  EXPECT_EQ(ufil_etsi::encodeDriveDirection(kPi, 5.0, 0.1), etsi::kDriveDirectionBackward);
  EXPECT_EQ(ufil_etsi::encodeDriveDirection(kPi, 0.0, 0.0), etsi::kDriveDirectionForward);
}

TEST(CamEncoding, StationTypeOfMostLikelyClass)
{
  namespace type = etsi::station_type;
  EXPECT_EQ(ufil_etsi::encodeStationType(objectOfClass(ObjectClass::kCar)), type::kPassengerCar);
  EXPECT_EQ(ufil_etsi::encodeStationType(objectOfClass(ObjectClass::kTruck)), type::kLightTruck);
  EXPECT_EQ(
    ufil_etsi::encodeStationType(objectOfClass(ObjectClass::kMotorcycle)), type::kMotorcycle);
  EXPECT_EQ(ufil_etsi::encodeStationType(objectOfClass(ObjectClass::kBicycle)), type::kCyclist);
  EXPECT_EQ(
    ufil_etsi::encodeStationType(objectOfClass(ObjectClass::kPedestrian)), type::kPedestrian);
  EXPECT_EQ(
    ufil_etsi::encodeStationType(objectOfClass(ObjectClass::kStationary)), type::kRoadSideUnit);
  EXPECT_EQ(ufil_etsi::encodeStationType(objectOfClass(ObjectClass::kOther)), type::kUnknown);
  EXPECT_EQ(ufil_etsi::encodeStationType(Object{}), type::kUnknown);
}

TEST(CamEncoding, ReferencePositionAndConfidenceEllipse)
{
  Object object;
  // On the central meridian (9 deg east) of zone 32.
  object.x = 500000.0;
  object.y = 5000000.0;
  object.setCovariance(ufil_etsi::kX, ufil_etsi::kX, 0.01);  // east: sigma 0.1 m
  object.setCovariance(ufil_etsi::kY, ufil_etsi::kY, 0.04);  // north: sigma 0.2 m

  const auto position = ufil_etsi::encodeReferencePosition(object, UtmZone{32, true});
  ASSERT_TRUE(position.has_value());
  EXPECT_EQ(position->longitude, 90000000);
  EXPECT_GT(position->latitude, 451000000);
  EXPECT_LT(position->latitude, 452000000);
  EXPECT_EQ(position->altitude, etsi::kAltitudeUnavailable);
  EXPECT_EQ(position->semi_major_orientation, 0);  // north
  EXPECT_EQ(position->semi_major_confidence, 39);  // 1.96 * 20 cm
  EXPECT_EQ(position->semi_minor_confidence, 19);

  // Larger uncertainty east: the major axis points east (was always north before).
  object.setCovariance(ufil_etsi::kX, ufil_etsi::kX, 0.04);
  object.setCovariance(ufil_etsi::kY, ufil_etsi::kY, 0.01);
  auto rotated = ufil_etsi::encodeReferencePosition(object, UtmZone{32, true});
  EXPECT_EQ(rotated->semi_major_orientation, 900);
  EXPECT_EQ(rotated->semi_major_confidence, 39);

  // Correlated: major axis north-east.
  object.setCovariance(ufil_etsi::kX, ufil_etsi::kX, 0.025);
  object.setCovariance(ufil_etsi::kY, ufil_etsi::kY, 0.025);
  object.setCovariance(ufil_etsi::kX, ufil_etsi::kY, 0.015);
  object.setCovariance(ufil_etsi::kY, ufil_etsi::kX, 0.015);
  rotated = ufil_etsi::encodeReferencePosition(object, UtmZone{32, true});
  EXPECT_EQ(rotated->semi_major_orientation, 450);
  EXPECT_EQ(rotated->semi_major_confidence, 39);
  EXPECT_EQ(rotated->semi_minor_confidence, 19);

  // Unknown position covariance.
  object.setCovariance(ufil_etsi::kX, ufil_etsi::kX, -1.0);
  const auto unknown = ufil_etsi::encodeReferencePosition(object, UtmZone{32, true});
  EXPECT_EQ(unknown->semi_major_confidence, etsi::kSemiAxisLengthUnavailable);
  EXPECT_EQ(unknown->semi_major_orientation, etsi::kHeadingUnavailable);

  // Not a valid UTM position.
  object.x = 1e9;
  EXPECT_FALSE(ufil_etsi::encodeReferencePosition(object, UtmZone{32, true}).has_value());
}

TEST(CamEncoding, VehicleHighFrequencyFields)
{
  Object object;
  object.yaw = 0.0;
  object.v_x = 5.0;
  object.yaw_rate = 0.1;
  object.length = 4.5;
  object.width = 1.8;
  object.dimension_covariance[0] = 0.01;
  object.dimension_covariance[4] = -1.0;  // width unknown
  const auto hf = ufil_etsi::encodeVehicleHighFrequency(object);
  EXPECT_EQ(hf.heading, 900);
  EXPECT_EQ(hf.speed, 500);
  EXPECT_EQ(hf.vehicle_length, 45);
  EXPECT_EQ(hf.vehicle_length_confidence_indication, etsi::kNoTrailerPresent);
  EXPECT_EQ(hf.vehicle_width, etsi::kVehicleWidthUnavailable);
  EXPECT_EQ(hf.yaw_rate, 572);
  EXPECT_TRUE(hf.lateral_acceleration.has_value());
}
