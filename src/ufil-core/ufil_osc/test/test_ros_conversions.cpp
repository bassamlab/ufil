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

#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_ros/json_ros.hpp>

#include "ufil_osc_ros.hpp"

TEST(RosConversions, ObjectListPayloadIsJson)
{
  ufil_msgs::msg::ObjectList msg;
  msg.header.frame_id = "map";
  msg.objects.resize(2);
  msg.objects[0].id = 7;
  const std::string payload = ufil_osc::conversions::payloadFromMsg(msg);
  EXPECT_EQ(payload, ufil_ros::to_json(msg));
  ASSERT_FALSE(payload.empty());
  EXPECT_EQ(payload.front(), '{');
  EXPECT_EQ(payload.back(), '}');
}

TEST(RosConversions, OccupancyGridPayloadIsJson)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = 2;
  msg.info.height = 2;
  msg.data = {0, 100, -1, 50};
  const std::string payload = ufil_osc::conversions::payloadFromMsg(msg);
  EXPECT_EQ(payload, ufil_ros::to_json(msg));
  ASSERT_FALSE(payload.empty());
  EXPECT_EQ(payload.front(), '{');
}
