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
#include "ufil_etsi/cam_decoding.hpp"
#include "ufil_etsi/cam_encoding.hpp"
#include "ufil_etsi/types.hpp"

// Core library tests, no ROS 2 required.

using ufil_etsi::Cam;
using ufil_etsi::DecodeStatus;
using ufil_etsi::Object;
using ufil_etsi::ObjectClass;
using ufil_etsi::UtmZone;
namespace etsi = ufil_etsi::etsi;

namespace
{

constexpr double kPi = std::numbers::pi;

/// A car heading north-west, encoded like the vehicle CAM generator does.
Cam encodedCar(Object & object)
{
  object.id = 42;
  object.x = 691000.0;
  object.y = 5335000.0;
  object.yaw = 0.75 * kPi;
  object.v_x = -5.0;
  object.v_y = 5.0;
  object.a_x = -1.0;
  object.a_y = 0.0;
  object.yaw_rate = 0.2;
  object.length = 4.5;
  object.width = 1.8;
  object.classification[static_cast<std::size_t>(ObjectClass::kCar)] = 1.0;
  for (std::size_t i = 0; i < ufil_etsi::kStateSize; ++i) {
    object.covariance[i * ufil_etsi::kStateSize + i] = 0.04;
  }
  object.setCovariance(ufil_etsi::kAx, ufil_etsi::kAx, 1.0);
  object.setCovariance(ufil_etsi::kAy, ufil_etsi::kAy, 1.0);
  object.setCovariance(ufil_etsi::kYaw, ufil_etsi::kYaw, 1e-4);

  Cam cam;
  cam.station_id = object.id;
  cam.station_type = ufil_etsi::encodeStationType(object);
  cam.reference_position = *ufil_etsi::encodeReferencePosition(object, UtmZone{32, true});
  cam.vehicle_high_frequency = ufil_etsi::encodeVehicleHighFrequency(object);
  return cam;
}

}  // namespace

TEST(CamDecoding, RoundTripOfAVehicle)
{
  Object original;
  const Cam cam = encodedCar(original);
  const auto result = ufil_etsi::decodeVehicleCam(cam);
  ASSERT_EQ(result.status, DecodeStatus::kOk);
  EXPECT_EQ(result.zone, (UtmZone{32, true}));

  const Object & object = result.object;
  EXPECT_EQ(object.id, 42u);
  EXPECT_NEAR(object.x, original.x, 0.02);
  EXPECT_NEAR(object.y, original.y, 0.02);
  EXPECT_NEAR(object.yaw, original.yaw, 0.002);
  EXPECT_NEAR(object.v_x, original.v_x, 0.02);
  EXPECT_NEAR(object.v_y, original.v_y, 0.02);
  EXPECT_NEAR(object.a_x, original.a_x, 0.1);
  EXPECT_NEAR(object.a_y, original.a_y, 0.1);
  EXPECT_NEAR(object.yaw_rate, original.yaw_rate, 0.001);
  EXPECT_NEAR(object.length, 4.5, 1e-9);
  EXPECT_NEAR(object.width, 1.8, 1e-9);
  EXPECT_DOUBLE_EQ(object.dimension_covariance[8], -1.0);
  EXPECT_DOUBLE_EQ(object.existence_probability, 1.0);
  EXPECT_EQ(ufil_etsi::mostLikelyClass(object), ObjectClass::kCar);

  // Variances survive the 95 % confidence encoding within its resolution (the confidences are
  // truncated, so the decoded variances are slightly smaller).
  EXPECT_NEAR(object.variance(ufil_etsi::kX), 0.04, 0.002);
  EXPECT_NEAR(object.variance(ufil_etsi::kY), 0.04, 0.002);
  EXPECT_NEAR(object.variance(ufil_etsi::kYaw), 1e-4, 1e-5);
  EXPECT_NEAR(object.variance(ufil_etsi::kAx) + object.variance(ufil_etsi::kAy), 2.0, 0.15);
}

TEST(CamDecoding, PositionCovarianceIsRotatedBack)
{
  Object original;
  Cam cam = encodedCar(original);
  // Major axis north-east, 1.96 * 20 cm; minor 1.96 * 10 cm.
  cam.reference_position.semi_major_orientation = 450;
  cam.reference_position.semi_major_confidence = 39;
  cam.reference_position.semi_minor_confidence = 19;
  const auto object = ufil_etsi::decodeVehicleCam(cam).object;
  EXPECT_NEAR(object.covarianceAt(ufil_etsi::kX, ufil_etsi::kX), 0.025, 0.002);
  EXPECT_NEAR(object.covarianceAt(ufil_etsi::kY, ufil_etsi::kY), 0.025, 0.002);
  EXPECT_NEAR(object.covarianceAt(ufil_etsi::kX, ufil_etsi::kY), 0.015, 0.002);
  EXPECT_DOUBLE_EQ(
    object.covarianceAt(ufil_etsi::kX, ufil_etsi::kY),
    object.covarianceAt(ufil_etsi::kY, ufil_etsi::kX));
}

TEST(CamDecoding, UnavailableValuesAreUnknown)
{
  Object original;
  Cam cam = encodedCar(original);
  auto & hf = *cam.vehicle_high_frequency;
  hf.speed = etsi::kSpeedUnavailable;
  hf.lateral_acceleration.reset();
  hf.yaw_rate = etsi::kYawRateUnavailable;
  hf.heading_confidence = etsi::kHeadingConfidenceUnavailable;
  hf.vehicle_length_confidence_indication = etsi::kTrailerPresentWithUnknownLength;
  const auto object = ufil_etsi::decodeVehicleCam(cam).object;
  EXPECT_DOUBLE_EQ(object.variance(ufil_etsi::kVx), -1.0);
  EXPECT_DOUBLE_EQ(object.variance(ufil_etsi::kAx), -1.0);
  EXPECT_DOUBLE_EQ(object.variance(ufil_etsi::kYawRate), -1.0);
  EXPECT_DOUBLE_EQ(object.variance(ufil_etsi::kYaw), 0.0);  // value known, confidence not
  EXPECT_DOUBLE_EQ(object.dimension_covariance[0], -1.0);
}

TEST(CamDecoding, OnlyVehiclesWithAPositionAreConverted)
{
  Object original;
  Cam cam = encodedCar(original);
  cam.station_type = etsi::station_type::kRoadSideUnit;
  EXPECT_EQ(ufil_etsi::decodeVehicleCam(cam).status, DecodeStatus::kNotAVehicle);
  cam.station_type = etsi::station_type::kPedestrian;
  EXPECT_EQ(ufil_etsi::decodeVehicleCam(cam).status, DecodeStatus::kNotAVehicle);

  cam.station_type = etsi::station_type::kPassengerCar;
  cam.vehicle_high_frequency.reset();
  EXPECT_EQ(ufil_etsi::decodeVehicleCam(cam).status, DecodeStatus::kNotAVehicle);

  cam = encodedCar(original);
  cam.reference_position.latitude = etsi::kLatitudeUnavailable;
  EXPECT_EQ(ufil_etsi::decodeVehicleCam(cam).status, DecodeStatus::kPositionUnavailable);
}

TEST(CamDecoding, StationTypeClassification)
{
  namespace type = etsi::station_type;
  EXPECT_EQ(ufil_etsi::classFromStationType(type::kPassengerCar), ObjectClass::kCar);
  // Trucks and mopeds were mixed up by comparisons with or-ed constants before.
  EXPECT_EQ(ufil_etsi::classFromStationType(type::kLightTruck), ObjectClass::kTruck);
  EXPECT_EQ(ufil_etsi::classFromStationType(type::kHeavyTruck), ObjectClass::kTruck);
  EXPECT_EQ(ufil_etsi::classFromStationType(type::kMoped), ObjectClass::kMotorcycle);
  EXPECT_EQ(ufil_etsi::classFromStationType(type::kMotorcycle), ObjectClass::kMotorcycle);
  EXPECT_EQ(ufil_etsi::classFromStationType(type::kRoadSideUnit), ObjectClass::kStationary);
  EXPECT_EQ(ufil_etsi::classFromStationType(type::kBus), ObjectClass::kOther);
  EXPECT_TRUE(ufil_etsi::isVehicleStationType(type::kMoped));
  EXPECT_TRUE(ufil_etsi::isVehicleStationType(type::kSpecialVehicles));
  EXPECT_FALSE(ufil_etsi::isVehicleStationType(type::kCyclist));
  EXPECT_FALSE(ufil_etsi::isVehicleStationType(type::kTram));
}

TEST(CamDecoding, SingleValues)
{
  // Backwards at 2 m/s with 1.96 * 10 cm/s confidence.
  const auto speed = ufil_etsi::decodeSpeed(200, 19, etsi::kDriveDirectionBackward);
  EXPECT_DOUBLE_EQ(speed.value, -2.0);
  EXPECT_NEAR(speed.variance, 0.01 * 0.97, 0.001);
  EXPECT_DOUBLE_EQ(ufil_etsi::decodeYaw(0, 1).value, kPi / 2.0);
  EXPECT_NEAR(ufil_etsi::decodeYaw(2700, 1).value, kPi, 1e-12);
  EXPECT_LT(ufil_etsi::decodeYaw(etsi::kHeadingUnavailable, 1).variance, 0.0);
  EXPECT_DOUBLE_EQ(ufil_etsi::decodeAcceleration(-13, 1).value, -1.3);
  EXPECT_LT(ufil_etsi::decodeAcceleration(etsi::kAccelerationUnavailable, 1).variance, 0.0);
  const auto yaw_rate =
    ufil_etsi::decodeYawRate(-572, etsi::YawRateConfidence::kDegSec001_00);
  EXPECT_NEAR(yaw_rate.value, -0.1, 0.001);
  EXPECT_GT(yaw_rate.variance, 0.0);
}
