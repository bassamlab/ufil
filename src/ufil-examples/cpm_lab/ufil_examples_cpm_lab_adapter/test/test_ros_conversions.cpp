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

#include <Eigen/Geometry>
#include <numbers>  // listed with the C headers, cpplint does not know this C++20 header

#include <cpm_lab_lab_msgs/msg/vehicle_state.hpp>
#include <cpm_lab_map_msgs/msg/lane_control_light.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include "ufil_examples_cpm_lab_adapter_ros.hpp"

namespace conversions = ufil_examples_cpm_lab_adapter::conversions;

TEST(RosConversionsTest, VehicleStateFromMsg)
{
  cpm_lab_lab_msgs::msg::VehicleState msg;
  msg.header.stamp.sec = 12;
  msg.header.stamp.nanosec = 345;
  msg.header.frame_id = "lab";
  msg.pose.position.x = 1.0;
  msg.pose.position.y = 2.0;
  msg.pose.orientation = conversions::yawToMsg(0.5);
  msg.twist.linear.x = 0.3;
  msg.twist.angular.z = 0.1;
  msg.accel.linear.x = 0.2;

  const auto state = conversions::vehicleStateFromMsg(msg);
  EXPECT_EQ(state.stamp_nanoseconds, 12000000345);
  EXPECT_EQ(state.frame_id, "lab");
  EXPECT_EQ(state.position, Eigen::Vector3d(1.0, 2.0, 0.0));
  EXPECT_NEAR(state.yaw, 0.5, 1e-12);
  EXPECT_EQ(state.linear_velocity, Eigen::Vector3d(0.3, 0.0, 0.0));
  EXPECT_EQ(state.angular_velocity, Eigen::Vector3d(0.0, 0.0, 0.1));
  EXPECT_EQ(state.linear_acceleration, Eigen::Vector3d(0.2, 0.0, 0.0));

  EXPECT_EQ(conversions::stampToMsg(state.stamp_nanoseconds), msg.header.stamp);
}

TEST(RosConversionsTest, ObjectToMsg)
{
  ufil_examples_cpm_lab_adapter::VehicleObject object;
  object.id = 19;
  object.existence_probability = 0.99;
  object.x = 18.0;
  object.y = 36.0;
  object.vx = 1.0;
  object.vy = 2.0;
  object.yaw = 0.5;
  object.state_variances = {0.9, 0.9, 1.4, 1.4, 0.05, -1.0};
  object.length = 3.96;
  object.width = 1.89;
  object.height = 1.08;
  object.dimension_variances = {1.458, 0.018, 0.018};

  const auto msg = conversions::objectToMsg(object);
  EXPECT_EQ(msg.id, 19u);
  EXPECT_DOUBLE_EQ(msg.existence_probability, 0.99);
  EXPECT_DOUBLE_EQ(msg.classification.classification[ufil_msgs::msg::Classification::CAR], 1.0);
  EXPECT_TRUE(msg.features.c);

  const auto state =
    ufil_ros::stateFromMsg<ufil::type::state::PoseVelocity2D>(msg.state);
  EXPECT_DOUBLE_EQ(state.x(), 18.0);
  EXPECT_DOUBLE_EQ(state.y(), 36.0);
  EXPECT_DOUBLE_EQ(state.vx(), 1.0);
  EXPECT_DOUBLE_EQ(state.vy(), 2.0);
  EXPECT_DOUBLE_EQ(state.yaw(), 0.5);
  EXPECT_DOUBLE_EQ(state.covariance()(0, 0), 0.9);
  EXPECT_DOUBLE_EQ(state.covariance()(4, 4), 0.05);
  EXPECT_DOUBLE_EQ(state.covariance()(0, 1), 0.0);

  const auto dimension =
          ufil_ros::dimensionFromMsg<ufil::type::dimension::Dimension3D>(msg.dimension);
  EXPECT_DOUBLE_EQ(dimension.length(), 3.96);
  EXPECT_DOUBLE_EQ(dimension.width(), 1.89);
  EXPECT_DOUBLE_EQ(dimension.height(), 1.08);
  EXPECT_DOUBLE_EQ(dimension.covariance()(0, 0), 1.458);
}

TEST(RosConversionsTest, LaneControlLightFromMsg)
{
  cpm_lab_map_msgs::msg::LaneControlLight msg;
  msg.header.stamp.sec = 3;
  msg.header.stamp.nanosec = 500000000;
  msg.header.frame_id = "map";
  msg.segments.resize(2);
  msg.segments[0].segment_index = 0;
  msg.segments[0].image_name = "282-black";
  msg.segments[1].segment_index = 1;

  const auto lights = conversions::laneControlLightFromMsg(msg);
  EXPECT_DOUBLE_EQ(lights.stamp, 3.5);
  EXPECT_EQ(lights.frame_id, "map");
  ASSERT_EQ(lights.segments.size(), 2u);
  EXPECT_EQ(lights.segments[0].segment_index, 0);
  EXPECT_EQ(lights.segments[0].image_name, "282-black");
  EXPECT_EQ(lights.segments[1].segment_index, 1);
  EXPECT_EQ(lights.segments[1].image_name, "");
}

TEST(RosConversionsTest, IsometryFromMsg)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.transform.translation.x = 1.0;
  msg.transform.rotation = conversions::yawToMsg(std::numbers::pi / 2.0);
  const auto transform = conversions::isometryFromMsg(msg);
  EXPECT_TRUE(transform.isApprox(
      Eigen::Translation3d(1.0, 0.0, 0.0) *
      Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ())));
}

TEST(RosConversionsTest, SensorMessages)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 5;

  const auto fix = conversions::navSatFixToMsg({48.0, 11.0, 1.0}, stamp, "gnss_1");
  EXPECT_EQ(fix.header.stamp, stamp);
  EXPECT_EQ(fix.header.frame_id, "gnss_1");
  EXPECT_EQ(fix.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
  EXPECT_DOUBLE_EQ(fix.latitude, 48.0);
  EXPECT_DOUBLE_EQ(fix.longitude, 11.0);
  EXPECT_EQ(fix.position_covariance_type,
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
  EXPECT_DOUBLE_EQ(fix.position_covariance[0], 1.0);
  EXPECT_DOUBLE_EQ(fix.position_covariance[4], 1.0);
  EXPECT_DOUBLE_EQ(fix.position_covariance[8], 1.0);

  ufil_examples_cpm_lab_adapter::ImuMeasurement measurement;
  measurement.yaw = 0.7;
  measurement.angular_velocity = {0.0, 0.0, 0.2};
  measurement.linear_acceleration = {1.0, 0.0, 0.0};
  const auto imu = conversions::imuToMsg(measurement, stamp, "imu_1");
  EXPECT_EQ(imu.header.frame_id, "imu_1");
  EXPECT_NEAR(conversions::yawFromMsg(imu.orientation), 0.7, 1e-12);
  EXPECT_DOUBLE_EQ(imu.angular_velocity.z, 0.2);
  EXPECT_DOUBLE_EQ(imu.linear_acceleration.x, 1.0);

  const auto odometry = conversions::odometryToMsg({{18.0, 9.0, 0.0}}, stamp, "odom_1",
    "base_link_1");
  EXPECT_EQ(odometry.header.frame_id, "odom_1");
  EXPECT_EQ(odometry.child_frame_id, "base_link_1");
  EXPECT_DOUBLE_EQ(odometry.twist.twist.linear.x, 18.0);
  EXPECT_DOUBLE_EQ(odometry.twist.twist.linear.y, 9.0);
}
