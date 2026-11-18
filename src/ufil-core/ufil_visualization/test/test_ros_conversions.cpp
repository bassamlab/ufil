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

#include <cstddef>
#include <vector>

#include <std_msgs/msg/header.hpp>
#include <ufil_msgs/msg/classification.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "ufil_visualization_ros.hpp"

namespace conversions = ufil_visualization::conversions;
using ufil_visualization::Marker;
using ufil_visualization::MarkerType;
using ufil_visualization::ObjectClass;
using MarkerMsg = visualization_msgs::msg::Marker;

TEST(RosConversionsTest, MarkerToMsg)
{
  Marker marker;
  marker.ns = "body";
  marker.id = 3;
  marker.type = MarkerType::kLineList;
  marker.position = {1.0, 2.0, 3.0};
  marker.orientation = {0.0, 0.0, 1.0, 0.0};
  marker.scale = {0.1, 0.2, 0.3};
  marker.color = {0.25F, 0.5F, 0.75F, 1.0F};
  marker.points = {{0.0, 0.0, 0.0}, {1.0, 1.0, 0.5}};
  marker.text = "text";
  marker.lifetime = 1.5;
  marker.frame_locked = true;

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 12;

  const auto msg = conversions::markerToMsg(marker, header);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.header.stamp.sec, 12);
  EXPECT_EQ(msg.ns, "body");
  EXPECT_EQ(msg.id, 3);
  EXPECT_EQ(msg.type, MarkerMsg::LINE_LIST);
  EXPECT_EQ(msg.action, MarkerMsg::ADD);
  EXPECT_DOUBLE_EQ(msg.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(msg.pose.orientation.z, 1.0);
  EXPECT_DOUBLE_EQ(msg.pose.orientation.w, 0.0);
  EXPECT_DOUBLE_EQ(msg.scale.z, 0.3);
  EXPECT_FLOAT_EQ(msg.color.g, 0.5F);
  ASSERT_EQ(msg.points.size(), 2u);
  EXPECT_DOUBLE_EQ(msg.points[1].z, 0.5);
  EXPECT_EQ(msg.text, "text");
  EXPECT_EQ(msg.lifetime.sec, 1);
  EXPECT_EQ(msg.lifetime.nanosec, 500000000u);
  EXPECT_TRUE(msg.frame_locked);
}

TEST(RosConversionsTest, MarkerTypes)
{
  EXPECT_EQ(conversions::markerTypeToMsg(MarkerType::kArrow), MarkerMsg::ARROW);
  EXPECT_EQ(conversions::markerTypeToMsg(MarkerType::kCube), MarkerMsg::CUBE);
  EXPECT_EQ(conversions::markerTypeToMsg(MarkerType::kLineStrip), MarkerMsg::LINE_STRIP);
  EXPECT_EQ(conversions::markerTypeToMsg(MarkerType::kLineList), MarkerMsg::LINE_LIST);
  EXPECT_EQ(conversions::markerTypeToMsg(MarkerType::kTriangleList), MarkerMsg::TRIANGLE_LIST);
  EXPECT_EQ(
    conversions::markerTypeToMsg(MarkerType::kTextViewFacing), MarkerMsg::TEXT_VIEW_FACING);
}

TEST(RosConversionsTest, MarkersToMsgReplacesPreviousMarkers)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  const std::vector<Marker> markers(2);

  const auto replacing = conversions::markersToMsg(markers, header, true);
  ASSERT_EQ(replacing.markers.size(), 3u);
  EXPECT_EQ(replacing.markers[0].action, MarkerMsg::DELETEALL);
  EXPECT_EQ(replacing.markers[0].header.frame_id, "map");
  EXPECT_EQ(replacing.markers[1].action, MarkerMsg::ADD);

  const auto adding = conversions::markersToMsg(markers, header, false);
  ASSERT_EQ(adding.markers.size(), 2u);
  EXPECT_EQ(adding.markers[0].action, MarkerMsg::ADD);
}

TEST(RosConversionsTest, ObjectFromMsg)
{
  ufil_msgs::msg::Object msg;
  msg.id = 17;
  msg.state.state.x = 1.0;
  msg.state.state.y = 2.0;
  msg.state.state.v_x = 3.0;
  msg.state.state.v_y = 4.0;
  msg.state.state.yaw = 0.5;
  msg.state.covariance[0] = 0.1;
  msg.state.covariance[1] = 0.2;
  msg.state.covariance[8] = 0.3;
  msg.state.covariance[9] = 0.4;
  msg.state.covariance[6 * 8 + 6] = 0.05;
  msg.dimension.dimension.length = 4.5;
  msg.dimension.dimension.width = 1.8;
  msg.dimension.dimension.height = 1.4;
  msg.existence_probability = 0.9;
  msg.classification.classification[ufil_msgs::msg::Classification::BICYCLE] = 0.7F;
  msg.classification.classification[ufil_msgs::msg::Classification::OTHER] = 0.3F;

  const auto object = conversions::objectFromMsg(msg);
  EXPECT_EQ(object.id, 17u);
  EXPECT_DOUBLE_EQ(object.x, 1.0);
  EXPECT_DOUBLE_EQ(object.y, 2.0);
  EXPECT_DOUBLE_EQ(object.vx, 3.0);
  EXPECT_DOUBLE_EQ(object.vy, 4.0);
  EXPECT_DOUBLE_EQ(object.yaw, 0.5);
  EXPECT_DOUBLE_EQ(object.length, 4.5);
  EXPECT_DOUBLE_EQ(object.width, 1.8);
  EXPECT_DOUBLE_EQ(object.height, 1.4);
  EXPECT_DOUBLE_EQ(object.position_covariance[0], 0.1);
  EXPECT_DOUBLE_EQ(object.position_covariance[1], 0.2);
  EXPECT_DOUBLE_EQ(object.position_covariance[2], 0.3);
  EXPECT_DOUBLE_EQ(object.position_covariance[3], 0.4);
  EXPECT_DOUBLE_EQ(object.yaw_variance, 0.05);
  EXPECT_DOUBLE_EQ(object.existence_probability, 0.9);
  EXPECT_FLOAT_EQ(
    static_cast<float>(object.class_probabilities[static_cast<std::size_t>(ObjectClass::kBicycle)]),
    0.7F);
  EXPECT_EQ(
    ufil_visualization::mostLikelyClass(object.class_probabilities), ObjectClass::kBicycle);
}

TEST(RosConversionsTest, ObjectsFromMsg)
{
  ufil_msgs::msg::ObjectList msg;
  msg.objects.resize(3);
  msg.objects[2].id = 5;
  const auto objects = conversions::objectsFromMsg(msg);
  ASSERT_EQ(objects.size(), 3u);
  EXPECT_EQ(objects[2].id, 5u);
}
