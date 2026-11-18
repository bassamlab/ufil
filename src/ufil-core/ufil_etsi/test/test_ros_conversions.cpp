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

#include <chrono>

#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <ufil_msgs/msg/classification.hpp>
#include <ufil_msgs/msg/object.hpp>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/types.hpp"
#include "ufil_etsi_ros.hpp"

namespace conversions = ufil_etsi::conversions;
namespace etsi = ufil_etsi::etsi;
using etsi_its_cam_msgs::msg::CAM;
using ufil_msgs::msg::Classification;

namespace
{

ufil_etsi::Cam vehicleCam()
{
  ufil_etsi::Cam cam;
  cam.station_id = 99;
  cam.generation_delta_time = 1234;
  cam.station_type = etsi::station_type::kPassengerCar;
  cam.reference_position.latitude = 480797720;
  cam.reference_position.longitude = 116357350;
  cam.reference_position.semi_major_confidence = 39;
  cam.reference_position.semi_minor_confidence = 19;
  cam.reference_position.semi_major_orientation = 450;
  ufil_etsi::CamVehicleHighFrequency hf;
  hf.heading = 900;
  hf.heading_confidence = 10;
  hf.speed = 1000;
  hf.speed_confidence = 39;
  hf.drive_direction = etsi::kDriveDirectionForward;
  hf.vehicle_length = 45;
  hf.vehicle_length_confidence_indication = etsi::kNoTrailerPresent;
  hf.vehicle_width = 18;
  hf.longitudinal_acceleration = -20;
  hf.longitudinal_acceleration_confidence = 1;
  hf.lateral_acceleration = ufil_etsi::CamLateralAcceleration{15, 9};
  hf.yaw_rate = 572;
  hf.yaw_rate_confidence = etsi::YawRateConfidence::kDegSec001_00;
  cam.vehicle_high_frequency = hf;
  ufil_etsi::CamVehicleLowFrequency lf;
  lf.vehicle_role = 6;
  lf.path_history.push_back({-672, 15, etsi::kDeltaAltitudeUnavailable, 50});
  cam.vehicle_low_frequency = lf;
  return cam;
}

}  // namespace

TEST(RosConversions, VehicleCamToMsg)
{
  const CAM msg = conversions::camToMsg(vehicleCam());
  EXPECT_EQ(msg.header.protocol_version, 2);
  EXPECT_EQ(msg.header.message_id, etsi_its_cam_msgs::msg::ItsPduHeader::MESSAGE_ID_CAM);
  EXPECT_EQ(msg.header.station_id.value, 99u);
  EXPECT_EQ(msg.cam.generation_delta_time.value, 1234);
  const auto & parameters = msg.cam.cam_parameters;
  EXPECT_EQ(
    parameters.basic_container.reference_position.position_confidence_ellipse
    .semi_major_orientation.value, 450);
  EXPECT_EQ(
    parameters.high_frequency_container.choice,
    etsi_its_cam_msgs::msg::HighFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY);
  const auto & hf = parameters.high_frequency_container.basic_vehicle_container_high_frequency;
  EXPECT_EQ(hf.speed.speed_value.value, 1000);
  // The lateral acceleration is transmitted (its presence flag was never set before).
  EXPECT_TRUE(hf.lateral_acceleration_is_present);
  EXPECT_EQ(hf.lateral_acceleration.lateral_acceleration_value.value, 15);
  EXPECT_FALSE(hf.vertical_acceleration_is_present);
  EXPECT_EQ(
    hf.curvature.curvature_value.value, etsi_its_cam_msgs::msg::CurvatureValue::UNAVAILABLE);
  EXPECT_EQ(hf.yaw_rate.yaw_rate_confidence.value, 3);
  ASSERT_TRUE(parameters.low_frequency_container_is_present);
  const auto & lf = parameters.low_frequency_container.basic_vehicle_container_low_frequency;
  EXPECT_EQ(lf.vehicle_role.value, 6);
  ASSERT_EQ(lf.exterior_lights.value.size(), 1u);
  EXPECT_EQ(lf.exterior_lights.value[0], 0);
  ASSERT_EQ(lf.path_history.array.size(), 1u);
  EXPECT_TRUE(lf.path_history.array[0].path_delta_time_is_present);
  EXPECT_EQ(lf.path_history.array[0].path_delta_time.value, 50);
  EXPECT_FALSE(parameters.special_vehicle_container_is_present);
}

TEST(RosConversions, CamRoundTrip)
{
  const auto original = vehicleCam();
  const auto cam = conversions::camFromMsg(conversions::camToMsg(original));
  EXPECT_EQ(cam.station_id, original.station_id);
  EXPECT_EQ(cam.generation_delta_time, original.generation_delta_time);
  EXPECT_EQ(cam.station_type, original.station_type);
  EXPECT_EQ(cam.reference_position.latitude, original.reference_position.latitude);
  EXPECT_EQ(cam.reference_position.semi_minor_confidence, 19);
  ASSERT_TRUE(cam.vehicle_high_frequency.has_value());
  EXPECT_EQ(cam.vehicle_high_frequency->heading, 900);
  EXPECT_EQ(cam.vehicle_high_frequency->lateral_acceleration->confidence, 9);
  EXPECT_EQ(
    cam.vehicle_high_frequency->yaw_rate_confidence, etsi::YawRateConfidence::kDegSec001_00);
  ASSERT_TRUE(cam.vehicle_low_frequency.has_value());
  EXPECT_EQ(cam.vehicle_low_frequency->path_history.at(0).delta_longitude, 15);
}

TEST(RosConversions, RsuCam)
{
  ufil_etsi::Cam rsu;
  rsu.station_type = etsi::station_type::kRoadSideUnit;
  const CAM msg = conversions::camToMsg(rsu);
  EXPECT_EQ(
    msg.cam.cam_parameters.high_frequency_container.choice,
    etsi_its_cam_msgs::msg::HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY);
  EXPECT_FALSE(msg.cam.cam_parameters.low_frequency_container_is_present);
  EXPECT_FALSE(conversions::camFromMsg(msg).vehicle_high_frequency.has_value());
}

TEST(RosConversions, Objects)
{
  ufil_msgs::msg::Object msg;
  msg.id = 5;
  msg.state.state.x = 1.0;
  msg.state.state.yaw_rate = 0.3;
  msg.state.covariance[63] = 0.01;
  msg.dimension.dimension.length = 4.0;
  msg.dimension.covariance[4] = -1.0;
  msg.classification.classification[Classification::PEDESTRIAN] = 0.7f;

  const auto object = conversions::objectFromMsg(msg);
  EXPECT_EQ(object.id, 5u);
  EXPECT_DOUBLE_EQ(object.x, 1.0);
  EXPECT_DOUBLE_EQ(object.variance(ufil_etsi::kYawRate), 0.01);
  EXPECT_DOUBLE_EQ(object.length, 4.0);
  EXPECT_DOUBLE_EQ(object.dimension_covariance[4], -1.0);
  EXPECT_EQ(ufil_etsi::mostLikelyClass(object), ufil_etsi::ObjectClass::kPedestrian);

  const auto back = conversions::objectToMsg(object);
  EXPECT_EQ(back.id, 5u);
  EXPECT_DOUBLE_EQ(back.state.state.yaw_rate, 0.3);
  EXPECT_FLOAT_EQ(back.classification.classification[Classification::PEDESTRIAN], 0.7f);
  EXPECT_TRUE(back.features.c);
  EXPECT_TRUE(back.features.fl);
}

TEST(RosConversions, Time)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1704067200;
  stamp.nanosec = 5;
  const auto time = conversions::timeFromMsg(stamp);
  EXPECT_EQ(time, std::chrono::seconds(1704067200) + std::chrono::nanoseconds(5));
  const auto back = conversions::timeToMsg(time);
  EXPECT_EQ(back.sec, stamp.sec);
  EXPECT_EQ(back.nanosec, stamp.nanosec);
}
