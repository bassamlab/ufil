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

#include <limits>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_ros/json_ros.hpp"

TEST(JsonRos, EscapesFrameId)
{
  ufil_msgs::msg::ObjectList msg;
  msg.header.frame_id = "a\"b\\c\nd";
  const std::string json = ufil_ros::to_json(msg);
  EXPECT_NE(json.find(R"("frame_id":"a\"b\\c\nd")"), std::string::npos) << json;
}

TEST(JsonRos, KeepsFullPrecision)
{
  ufil_msgs::msg::ObjectList msg;
  ufil_msgs::msg::Object object;
  object.state.state.x = 1234567.125;
  object.state.covariance[0] = 1e-7;
  msg.objects.push_back(object);
  const std::string json = ufil_ros::to_json(msg);
  EXPECT_NE(json.find(R"("x":1234567.125)"), std::string::npos) << json;
  EXPECT_NE(json.find("1e-07"), std::string::npos) << json;
}

TEST(JsonRos, NonFiniteNumbersAreNull)
{
  ufil_msgs::msg::ObjectList msg;
  ufil_msgs::msg::Object object;
  object.state.state.x = std::numeric_limits<double>::quiet_NaN();
  object.state.state.y = std::numeric_limits<double>::infinity();
  msg.objects.push_back(object);
  const std::string json = ufil_ros::to_json(msg);
  EXPECT_NE(json.find(R"("x":null)"), std::string::npos) << json;
  EXPECT_NE(json.find(R"("y":null)"), std::string::npos) << json;
  EXPECT_EQ(json.find("nan"), std::string::npos) << json;
  EXPECT_EQ(json.find("inf"), std::string::npos) << json;
}

TEST(JsonRos, SerializesOccupancyGrid)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = 2;
  grid.info.height = 1;
  grid.info.resolution = 0.5f;
  grid.data = {-1, 100};
  const std::string json = ufil_ros::to_json(grid);
  EXPECT_NE(json.find(R"("width":2)"), std::string::npos) << json;
  EXPECT_NE(json.find(R"("resolution":0.5)"), std::string::npos) << json;
}
