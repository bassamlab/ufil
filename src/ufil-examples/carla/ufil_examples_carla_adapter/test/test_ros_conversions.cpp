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
#include <vector>

#include "ufil_examples_carla_adapter_ros.hpp"

namespace conversions = ufil_examples_carla_adapter::conversions;
using ufil_examples_carla_adapter::Axle;
using ufil_examples_carla_adapter::Object;
using ufil_examples_carla_adapter::ObjectClass;

namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace

TEST(RosConversionsTest, YawFromQuaternion)
{
  EXPECT_NEAR(conversions::yawFromMsg(yawQuaternion(0.0)), 0.0, 1e-12);
  EXPECT_NEAR(conversions::yawFromMsg(yawQuaternion(1.2)), 1.2, 1e-12);
  EXPECT_NEAR(conversions::yawFromMsg(yawQuaternion(-2.5)), -2.5, 1e-12);
}

TEST(RosConversionsTest, ActorsFromMsg)
{
  carla_msgs::msg::CarlaActorList msg;
  carla_msgs::msg::CarlaActorInfo actor;
  actor.id = 42;
  actor.parent_id = 1;
  actor.type = "vehicle.tesla.model3";
  actor.rolename = "vehicle_3";
  msg.actors.push_back(actor);

  const auto actors = conversions::actorsFromMsg(msg);
  ASSERT_EQ(actors.size(), 1u);
  EXPECT_EQ(actors[0].id, 42u);
  EXPECT_EQ(actors[0].type, "vehicle.tesla.model3");
  EXPECT_EQ(actors[0].role_name, "vehicle_3");
}

TEST(RosConversionsTest, ObjectsFromMsg)
{
  derived_object_msgs::msg::ObjectArray msg;
  derived_object_msgs::msg::Object in;
  in.id = 7;
  in.pose.position.x = 1.0;
  in.pose.position.y = 2.0;
  in.pose.position.z = 3.0;
  in.pose.orientation = yawQuaternion(0.3);
  in.twist.linear.x = 4.0;
  in.twist.linear.y = 5.0;
  in.twist.angular.z = 0.2;
  in.accel.linear.x = 6.0;
  in.accel.linear.y = 7.0;
  in.shape.dimensions = {4.5, 2.0, 1.4};
  msg.objects.push_back(in);
  // Missing dimensions are zero.
  msg.objects.emplace_back();

  const auto objects = conversions::objectsFromMsg(msg);
  ASSERT_EQ(objects.size(), 2u);
  const auto & object = objects[0];
  EXPECT_EQ(object.id, 7u);
  EXPECT_DOUBLE_EQ(object.state.x, 1.0);
  EXPECT_DOUBLE_EQ(object.state.y, 2.0);
  EXPECT_DOUBLE_EQ(object.state.v_x, 4.0);
  EXPECT_DOUBLE_EQ(object.state.v_y, 5.0);
  EXPECT_DOUBLE_EQ(object.state.a_x, 6.0);
  EXPECT_DOUBLE_EQ(object.state.a_y, 7.0);
  EXPECT_NEAR(object.state.yaw, 0.3, 1e-12);
  EXPECT_DOUBLE_EQ(object.state.yaw_rate, 0.2);
  EXPECT_DOUBLE_EQ(object.dimensions.length, 4.5);
  EXPECT_DOUBLE_EQ(object.dimensions.width, 2.0);
  EXPECT_DOUBLE_EQ(object.dimensions.height, 1.4);
  EXPECT_DOUBLE_EQ(objects[1].dimensions.length, 0.0);
}

TEST(RosConversionsTest, PoseFromTransform)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.transform.translation.x = 392000.0;
  msg.transform.translation.y = 5117000.0;
  msg.transform.rotation = yawQuaternion(kPi / 4.0);
  const auto pose = conversions::poseFromMsg(msg);
  EXPECT_DOUBLE_EQ(pose.x, 392000.0);
  EXPECT_DOUBLE_EQ(pose.y, 5117000.0);
  EXPECT_NEAR(pose.yaw, kPi / 4.0, 1e-12);
}

TEST(RosConversionsTest, ObjectToMsg)
{
  Object object;
  object.id = 3;
  object.state = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.7, 0.8};
  object.state_variances = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
  object.dimensions = {4.0, 2.0, 1.5};
  object.dimension_variances = {0.01, 0.02, 0.03};
  object.existence_probability = 1.0;
  object.object_class = ObjectClass::kTruck;
  Axle axle;
  axle.track_width = 2.5;
  axle.center_to_axle = -1.5;
  axle.wheel_detected = {true, false};
  object.axles = {axle};

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 12;
  const auto list = conversions::toMsg({object}, header);
  EXPECT_EQ(list.header, header);
  ASSERT_EQ(list.objects.size(), 1u);
  const auto & msg = list.objects[0];

  EXPECT_EQ(msg.id, 3u);
  EXPECT_DOUBLE_EQ(msg.state.state.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.state.state.y, 2.0);
  EXPECT_DOUBLE_EQ(msg.state.state.v_x, 3.0);
  EXPECT_DOUBLE_EQ(msg.state.state.v_y, 4.0);
  EXPECT_DOUBLE_EQ(msg.state.state.a_x, 5.0);
  EXPECT_DOUBLE_EQ(msg.state.state.a_y, 6.0);
  EXPECT_DOUBLE_EQ(msg.state.state.yaw, 0.7);
  EXPECT_DOUBLE_EQ(msg.state.state.yaw_rate, 0.8);
  // Diagonal covariances.
  EXPECT_DOUBLE_EQ(msg.state.covariance[0], 0.1);
  EXPECT_DOUBLE_EQ(msg.state.covariance[9], 0.2);
  EXPECT_DOUBLE_EQ(msg.state.covariance[63], 0.8);
  EXPECT_DOUBLE_EQ(msg.state.covariance[1], 0.0);
  EXPECT_DOUBLE_EQ(msg.dimension.dimension.length, 4.0);
  EXPECT_DOUBLE_EQ(msg.dimension.covariance[0], 0.01);
  EXPECT_DOUBLE_EQ(msg.dimension.covariance[4], 0.02);
  EXPECT_DOUBLE_EQ(msg.dimension.covariance[8], 0.03);
  EXPECT_DOUBLE_EQ(msg.existence_probability, 1.0);

  for (std::size_t i = 0; i < msg.classification.classification.size(); ++i) {
    EXPECT_FLOAT_EQ(
      msg.classification.classification[i],
      i == ufil_msgs::msg::Classification::TRUCK ? 1.0F : 0.0F);
  }
  EXPECT_TRUE(msg.features.fl && msg.features.br && msg.features.c);

  ASSERT_EQ(msg.axles.size(), 1u);
  EXPECT_FLOAT_EQ(msg.axles[0].track_width, 2.5F);
  EXPECT_FLOAT_EQ(msg.axles[0].center_to_axle, -1.5F);
  EXPECT_EQ(msg.axles[0].wheel_detected, (std::vector<bool>{true, false}));
}

TEST(RosConversionsTest, ClassificationIndex)
{
  EXPECT_EQ(
    conversions::classificationIndex(ObjectClass::kCar), ufil_msgs::msg::Classification::CAR);
  EXPECT_EQ(
    conversions::classificationIndex(ObjectClass::kMotorcycle),
    ufil_msgs::msg::Classification::MOTORCYCLE);
  EXPECT_EQ(
    conversions::classificationIndex(ObjectClass::kOther), ufil_msgs::msg::Classification::OTHER);
}
