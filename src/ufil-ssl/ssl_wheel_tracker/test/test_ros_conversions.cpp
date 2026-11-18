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
#include <vector>

#include "ssl_wheel_tracker_ros.hpp"

namespace conversions = ssl_wheel_tracker::conversions;
using ssl_wheel_tracker::Point2D;
using ssl_wheel_tracker::Timestamp;
using ssl_wheel_tracker::Track;
using ssl_wheel_tracker::TrackState;

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

TEST(RosConversions, Timestamps)
{
  const rclcpp::Time time(12, 345678901, RCL_ROS_TIME);
  const Timestamp stamp = conversions::timestampFromRos(time);
  EXPECT_EQ(ufil::to_nanoseconds(stamp), time.nanoseconds());
  const auto msg = conversions::timestampToMsg(stamp);
  EXPECT_EQ(msg.sec, 12);
  EXPECT_EQ(msg.nanosec, 345678901u);
}

TEST(RosConversions, GridFromMsg)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = 3;
  msg.info.height = 2;
  msg.info.resolution = 0.2f;
  msg.info.origin.position.x = 1.0;
  msg.info.origin.position.y = 2.0;
  msg.info.origin.orientation = yawQuaternion(0.5);
  msg.data = {0, 10, 127, -1, -128, 5};

  const auto grid = conversions::gridFromMsg(msg);
  EXPECT_TRUE(grid.valid());
  EXPECT_EQ(grid.columns, 3u);
  EXPECT_EQ(grid.rows, 2u);
  EXPECT_NEAR(grid.resolution, 0.2, 1e-6);
  EXPECT_DOUBLE_EQ(grid.origin.x, 1.0);
  EXPECT_DOUBLE_EQ(grid.origin.y, 2.0);
  EXPECT_NEAR(grid.origin.yaw, 0.5, 1e-12);
  // Cells are unsigned pressures, row-major.
  EXPECT_EQ(grid.at(2, 0), 127u);
  EXPECT_EQ(grid.at(0, 1), 255u);
  EXPECT_EQ(grid.at(1, 1), 128u);

  msg.data.pop_back();
  EXPECT_FALSE(conversions::gridFromMsg(msg).valid());
}

TEST(RosConversions, TransformPoint)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = 10.0;
  transform.translation.y = -2.0;
  transform.translation.z = 0.011;
  transform.rotation = yawQuaternion(kPi / 2.0);

  const Point2D p = conversions::transformPoint(transform, {1.0, 2.0});
  EXPECT_NEAR(p.x, 10.0 - 2.0, 1e-12);
  EXPECT_NEAR(p.y, -2.0 + 1.0, 1e-12);

  const Point2D identity = conversions::transformPoint(geometry_msgs::msg::Transform{}, {1.0, 2.0});
  EXPECT_DOUBLE_EQ(identity.x, 1.0);
  EXPECT_DOUBLE_EQ(identity.y, 2.0);
}

TEST(RosConversions, WheelsToMsg)
{
  ssl_wheel_tracker::WheelDetection detection;
  detection.pressure = 42.0;
  const auto msg = conversions::wheelsToMsg(
    {{1.0, 2.0}, {3.0, 4.0}}, {detection, detection},
    rclcpp::Time(5, 0, RCL_ROS_TIME), "map", 7);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.header.stamp.sec, 5);
  ASSERT_EQ(msg.wheels.size(), 2u);
  EXPECT_EQ(msg.wheels[0].wheel_id, 7);
  EXPECT_EQ(msg.wheels[1].wheel_id, 8);
  EXPECT_FLOAT_EQ(msg.wheels[1].x, 3.0f);
  EXPECT_FLOAT_EQ(msg.wheels[1].y, 4.0f);
  EXPECT_FLOAT_EQ(msg.wheels[0].intensity, 42.0f);
}

TEST(RosConversions, TracksToMsg)
{
  TrackState matched;
  matched.mean << 1.0, 2.0, 3.0, 4.0;
  matched.covariance = Eigen::Matrix4d::Zero();
  matched.covariance(0, 1) = 0.5;
  matched.covariance(2, 2) = 0.25;
  matched.matched = true;
  TrackState predicted = matched;
  predicted.matched = false;

  Track track;
  track.id = 17;
  track.misses = 300;
  track.history = {matched, predicted};

  const auto msg = conversions::tracksToMsg(
    {track}, rclcpp::Time(2, 500, RCL_ROS_TIME), "map");
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.header.stamp.nanosec, 500u);
  ASSERT_EQ(msg.tracks.size(), 1u);
  const auto & track_msg = msg.tracks[0];
  EXPECT_EQ(track_msg.track_id, "17");
  EXPECT_EQ(track_msg.frames_unseen, 127);
  ASSERT_EQ(track_msg.states.size(), 2u);
  EXPECT_FLOAT_EQ(track_msg.states[0].x, 1.0f);
  EXPECT_FLOAT_EQ(track_msg.states[0].vx, 2.0f);
  EXPECT_FLOAT_EQ(track_msg.states[0].y, 3.0f);
  EXPECT_FLOAT_EQ(track_msg.states[0].vy, 4.0f);
  EXPECT_FLOAT_EQ(track_msg.states[0].covariance[1], 0.5f);
  EXPECT_FLOAT_EQ(track_msg.states[0].covariance[10], 0.25f);
  EXPECT_EQ(track_msg.states[0].state, "matched");
  EXPECT_EQ(track_msg.states[1].state, "predicted");
}
