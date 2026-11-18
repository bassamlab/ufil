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

#include <set>

#include <ufil_msgs/msg/object_list.hpp>

#include "tracker_ros.hpp"

namespace conversions = ufil_examples_pedestrian_tracker::conversions;
using ufil_examples_pedestrian_tracker::PedestrianTracker;

namespace
{

ufil_msgs::msg::Object measuredObject(double x, double y)
{
  ufil_msgs::msg::Object object;
  object.state.state.x = x;
  object.state.state.y = y;
  object.state.covariance[0] = 1.0;
  object.state.covariance[9] = 1.0;
  object.state.covariance[54] = 0.1;
  for (const auto index : {18, 27, 36, 45, 63}) {
    object.state.covariance[index] = -1.0;
  }
  object.dimension.dimension.length = 1.0;
  object.dimension.dimension.width = 0.5;
  object.dimension.dimension.height = 1.8;
  object.dimension.covariance[0] = 1.0;
  object.dimension.covariance[4] = 1.0;
  object.dimension.covariance[8] = 1.0;
  return object;
}

}  // namespace

TEST(RosConversionsTest, Timestamps)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 500000000;
  EXPECT_EQ(
    ufil::to_nanoseconds(conversions::timestampFromMsg(stamp)), 12500000000);
  EXPECT_EQ(
    ufil::to_nanoseconds(conversions::timestampFromRos(rclcpp::Time(3, 0, RCL_ROS_TIME))),
    3000000000);
}

TEST(RosConversionsTest, DetectionsFromObjectList)
{
  ufil_msgs::msg::ObjectList msg;
  msg.objects = {measuredObject(1.0, 2.0), measuredObject(-3.0, 4.0)};
  const auto detections = conversions::detectionsFromMsg(msg);
  ASSERT_EQ(detections.size(), 2u);
  bool found = false;
  for (const auto & detection : detections) {
    if (std::abs(detection.x() - 1.0F) < 1e-6F) {
      found = true;
      EXPECT_FLOAT_EQ(detection.y(), 2.0F);
      EXPECT_FLOAT_EQ(detection.dimension().width(), 0.5F);
      EXPECT_FLOAT_EQ(detection.covariance()(0, 0), 1.0F);
    }
  }
  EXPECT_TRUE(found);
}

TEST(RosConversionsTest, TracksToObjectList)
{
  ufil_msgs::msg::ObjectList measured;
  measured.objects = {measuredObject(1.0, 2.0)};
  PedestrianTracker tracker;
  tracker.update(
    conversions::detectionsFromMsg(measured), ufil::from_seconds<ufil::type::Timestamp>(1.0));

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 1;
  const auto msg = conversions::tracksToMsg(tracker.tracks(), header);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.header.stamp.sec, 1);
  ASSERT_EQ(msg.objects.size(), 1u);
  EXPECT_NEAR(msg.objects[0].state.state.x, 1.0, 1e-3);
  EXPECT_NEAR(msg.objects[0].state.state.y, 2.0, 1e-3);
}
