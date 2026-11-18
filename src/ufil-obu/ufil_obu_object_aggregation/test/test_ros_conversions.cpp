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

#include <tf2/LinearMath/Quaternion.h>
#include <numbers>  // listed with the C headers, cpplint does not know this C++20 header

#include "ufil_obu_object_aggregation_ros.hpp"

namespace conversions = ufil_obu_object_aggregation::conversions;

TEST(RosConversions, FixStatusMapsToHasFix)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.latitude = 50.0;
  msg.longitude = 6.0;
  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  EXPECT_TRUE(conversions::fixFromMsg(msg).has_fix);
  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  EXPECT_TRUE(conversions::fixFromMsg(msg).has_fix);
  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  EXPECT_FALSE(conversions::fixFromMsg(msg).has_fix);
}

TEST(RosConversions, OdometryAndAcceleration)
{
  nav_msgs::msg::Odometry odom;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, std::numbers::pi / 4.0);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = 2.0;
  odom.twist.twist.angular.z = 0.3;
  odom.pose.covariance[35] = 0.01;
  const auto odometry = conversions::odometryFromMsg(odom);
  EXPECT_NEAR(odometry.yaw, std::numbers::pi / 4.0, 1e-9);
  EXPECT_DOUBLE_EQ(odometry.v_x, 2.0);
  EXPECT_DOUBLE_EQ(odometry.yaw_rate, 0.3);
  EXPECT_DOUBLE_EQ(odometry.pose_covariance[35], 0.01);

  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.accel.accel.linear.y = -1.5;
  accel.accel.covariance[0] = 0.2;
  const auto acceleration = conversions::accelerationFromMsg(accel);
  EXPECT_DOUBLE_EQ(acceleration.a_y, -1.5);
  EXPECT_DOUBLE_EQ(acceleration.covariance[0], 0.2);
}

TEST(RosConversions, LatestStamp)
{
  builtin_interfaces::msg::Time a, b, c;
  a.sec = 10;
  a.nanosec = 900;
  b.sec = 11;
  b.nanosec = 1;
  c.sec = 11;
  c.nanosec = 0;
  const auto latest = conversions::latestStamp({a, b, c});
  EXPECT_EQ(latest.sec, 11);
  EXPECT_EQ(latest.nanosec, 1u);
}

TEST(RosConversions, ObjectToMsg)
{
  ufil_obu_object_aggregation::OwnObject object;
  object.id = 42;
  object.position.easting = 294230.0;
  object.position.northing = 5629400.0;
  object.position.zone = 32;
  object.v_x = 3.0;
  object.length = 4.5;
  object.width = 1.8;
  object.dimension_covariance[4] = 1e-9;
  object.state_covariance[63] = 0.5;
  object.classification[0] = 1.0f;

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  const auto msg = conversions::objectToMsg(object, stamp);
  EXPECT_EQ(msg.header.frame_id, "utm_32n");
  EXPECT_EQ(msg.header.stamp.sec, 7);
  EXPECT_EQ(msg.object.id, 42u);
  EXPECT_DOUBLE_EQ(msg.object.state.state.x, 294230.0);
  EXPECT_DOUBLE_EQ(msg.object.state.state.v_x, 3.0);
  EXPECT_DOUBLE_EQ(msg.object.state.covariance[63], 0.5);
  EXPECT_DOUBLE_EQ(msg.object.dimension.dimension.width, 1.8);
  EXPECT_DOUBLE_EQ(msg.object.dimension.covariance[4], 1e-9);
  EXPECT_FLOAT_EQ(msg.object.classification.classification[0], 1.0f);
  EXPECT_DOUBLE_EQ(msg.object.existence_probability, 1.0);
  EXPECT_TRUE(msg.object.features.c);
}
