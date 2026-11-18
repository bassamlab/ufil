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

#include <cmath>
#include <cstdint>

#include "ssl_simulator_ros.hpp"

namespace conversions = ssl_simulator::conversions;
using ssl_simulator::Timestamp;

namespace
{

constexpr double kPi = 3.14159265358979323846;

Timestamp at(double seconds)
{
  return ufil::from_seconds<Timestamp>(seconds);
}

ufil_msgs::msg::Object vehicleMsg()
{
  ufil_msgs::msg::Object object;
  object.id = 12;
  object.state.state.x = 1.0;
  object.state.state.y = 2.0;
  object.state.state.v_x = 3.0;
  object.state.state.v_y = -4.0;
  object.state.state.yaw = 0.5;
  object.state.state.yaw_rate = 0.1;
  ufil_msgs::msg::Axle front;
  front.track_width = 1.6f;
  front.center_to_axle = 1.4f;
  ufil_msgs::msg::Axle rear;
  rear.single_track = true;
  rear.center_to_axle = -1.2f;
  object.axles = {front, rear};
  return object;
}

}  // namespace

TEST(RosConversions, NamesFollowTheNodeId)
{
  EXPECT_EQ(conversions::mapTopic(1), "ssl_1/map");
  EXPECT_EQ(conversions::matFrameId(4), "ssl_frame_4");
}

TEST(RosConversions, Timestamps)
{
  const rclcpp::Time time(12, 500000000, RCL_ROS_TIME);
  EXPECT_EQ(conversions::timestampFromRos(time), at(12.5));
}

TEST(RosConversions, VehicleUsesAbsoluteVelocity)
{
  const auto vehicle = conversions::vehicleFromMsg(vehicleMsg(), at(3.0));
  EXPECT_EQ(vehicle.id, 12u);
  EXPECT_DOUBLE_EQ(vehicle.state.x(), 1.0);
  EXPECT_DOUBLE_EQ(vehicle.state.y(), 2.0);
  EXPECT_DOUBLE_EQ(vehicle.state.vx(), 3.0);
  EXPECT_DOUBLE_EQ(vehicle.state.vy(), -4.0);
  EXPECT_DOUBLE_EQ(vehicle.state.yaw(), 0.5);
  EXPECT_DOUBLE_EQ(vehicle.state.yawRate(), 0.1);
  EXPECT_EQ(vehicle.state.timestamp(), at(3.0));
  ASSERT_EQ(vehicle.axles.size(), 2u);
  EXPECT_FALSE(vehicle.axles[0].single_track);
  EXPECT_NEAR(vehicle.axles[0].track_width, 1.6, 1e-6);
  EXPECT_NEAR(vehicle.axles[0].center_to_axle, 1.4, 1e-6);
  EXPECT_TRUE(vehicle.axles[1].single_track);
  EXPECT_NEAR(vehicle.axles[1].center_to_axle, -1.2, 1e-6);
}

TEST(RosConversions, ObjectListStampIsTheStateTime)
{
  ufil_msgs::msg::ObjectList msg;
  msg.header.stamp.sec = 7;
  msg.header.stamp.nanosec = 250000000;
  msg.objects = {vehicleMsg(), vehicleMsg()};
  const auto vehicles = conversions::vehiclesFromMsg(msg, at(9.0));
  ASSERT_EQ(vehicles.size(), 2u);
  EXPECT_EQ(vehicles[0].state.timestamp(), at(7.25));

  // Without a stamp the reception time is used.
  msg.header.stamp = builtin_interfaces::msg::Time();
  EXPECT_EQ(conversions::vehiclesFromMsg(msg, at(9.0))[1].state.timestamp(), at(9.0));
}

TEST(RosConversions, PressureMapToOccupancyGrid)
{
  ssl_simulator::SimulatorConfig config;
  config.pose = {205.0, 27.92, 0.011, kPi / 2.0};
  config.width = 0.6;
  config.height = 0.4;
  ssl_simulator::PressureMap pressure(config.columns(), config.rows());
  pressure.at(2, 1) = 27;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 5;

  const auto msg = conversions::pressureMapToMsg(pressure, config, stamp);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.header.stamp.sec, 5);
  EXPECT_EQ(msg.info.map_load_time.sec, 5);
  EXPECT_EQ(msg.info.width, 3u);
  EXPECT_EQ(msg.info.height, 2u);
  EXPECT_FLOAT_EQ(msg.info.resolution, 0.2f);
  EXPECT_DOUBLE_EQ(msg.info.origin.position.x, 205.0);
  EXPECT_DOUBLE_EQ(msg.info.origin.position.y, 27.92);
  EXPECT_DOUBLE_EQ(msg.info.origin.position.z, 0.011);
  EXPECT_NEAR(msg.info.origin.orientation.z, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(msg.info.origin.orientation.w, std::sqrt(0.5), 1e-12);
  ASSERT_EQ(msg.data.size(), 6u);
  EXPECT_EQ(msg.data[1 * 3 + 2], 27);
  EXPECT_EQ(msg.data[0], 0);
}

TEST(RosConversions, MatTransform)
{
  const ssl_simulator::MatPose pose{1.0, 2.0, 0.011, kPi};
  builtin_interfaces::msg::Time stamp;
  const auto msg = conversions::matTransformToMsg(pose, 2, stamp);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.child_frame_id, "ssl_frame_2");
  EXPECT_DOUBLE_EQ(msg.transform.translation.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.transform.translation.y, 2.0);
  EXPECT_DOUBLE_EQ(msg.transform.translation.z, 0.011);
  EXPECT_NEAR(msg.transform.rotation.z, 1.0, 1e-12);
  EXPECT_NEAR(msg.transform.rotation.w, 0.0, 1e-12);
}
