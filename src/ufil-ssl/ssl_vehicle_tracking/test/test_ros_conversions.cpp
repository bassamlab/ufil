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
#include <vector>

#include "ssl_vehicle_tracking_ros.hpp"

namespace conversions = ssl_vehicle_tracking::conversions;
using ssl_vehicle_tracking::Vehicle;
using ssl_vehicle_tracking::VehicleTemplate;

namespace
{

VehicleTemplate truck()
{
  return ssl_vehicle_tracking::loadTemplate(std::string(TEMPLATE_DIR) + "/truck_carla.json", "t");
}

ssl_vehicle_tracking_msgs::msg::Track trackMsg(const std::string & id, float x, float y)
{
  ssl_vehicle_tracking_msgs::msg::Track track;
  track.track_id = id;
  track.frames_unseen = 2;
  ssl_vehicle_tracking_msgs::msg::TrackState old_state;
  old_state.x = -100.0f;
  ssl_vehicle_tracking_msgs::msg::TrackState state;
  state.x = x;
  state.y = y;
  state.vx = 1.5f;
  state.vy = -0.5f;
  track.states = {old_state, state};
  return track;
}

}  // namespace

TEST(RosConversions, WheelTracksUseTheLatestState)
{
  ssl_vehicle_tracking_msgs::msg::TrackArray msg;
  msg.tracks = {trackMsg("7", 1.0f, 2.0f), trackMsg("8", 3.0f, 4.0f)};
  msg.tracks.emplace_back();  // without state
  msg.tracks.push_back(trackMsg("9", std::numeric_limits<float>::quiet_NaN(), 0.0f));

  const auto [tracks, rejected] = conversions::wheelTracksFromMsg(msg);
  EXPECT_EQ(rejected, 2u);
  ASSERT_EQ(tracks.size(), 2u);
  EXPECT_EQ(tracks[0].id, "7");
  EXPECT_DOUBLE_EQ(tracks[0].position.x, 1.0);
  EXPECT_DOUBLE_EQ(tracks[0].position.y, 2.0);
  EXPECT_DOUBLE_EQ(tracks[0].velocity.x, 1.5);
  EXPECT_DOUBLE_EQ(tracks[0].velocity.y, -0.5);
  EXPECT_EQ(tracks[0].frames_unseen, 2);
  EXPECT_EQ(tracks[1].id, "8");
}

TEST(RosConversions, VehicleToMsg)
{
  const auto vehicle_template = truck();
  Vehicle vehicle;
  vehicle.id = 42;
  vehicle.position = {1.0, 2.0};
  vehicle.velocity = {3.0, 4.0};
  vehicle.yaw = 0.9;
  vehicle.existence_probability = 0.8;

  const auto msg = conversions::vehicleToMsg(vehicle, vehicle_template);
  EXPECT_EQ(msg.id, 42u);
  EXPECT_DOUBLE_EQ(msg.state.state.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.state.state.y, 2.0);
  EXPECT_DOUBLE_EQ(msg.state.state.v_x, 3.0);
  EXPECT_DOUBLE_EQ(msg.state.state.v_y, 4.0);
  EXPECT_DOUBLE_EQ(msg.state.state.yaw, 0.9);
  EXPECT_DOUBLE_EQ(msg.state.state.yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(msg.state.covariance[0], 0.01);
  EXPECT_DOUBLE_EQ(msg.state.covariance[63], conversions::kNotEstimated);
  EXPECT_DOUBLE_EQ(msg.dimension.dimension.length, vehicle_template.description().length);
  EXPECT_DOUBLE_EQ(msg.dimension.covariance[8], conversions::kNotEstimated);
  EXPECT_DOUBLE_EQ(msg.existence_probability, 0.8);
  EXPECT_FLOAT_EQ(msg.classification.classification[1], 1.0f);
  EXPECT_FLOAT_EQ(msg.classification.classification[0], 0.0f);
  ASSERT_EQ(msg.axles.size(), 3u);
  EXPECT_FALSE(msg.axles[0].single_track);
  EXPECT_NEAR(msg.axles[0].center_to_axle, 3.96785521507, 1e-6);
  EXPECT_EQ(msg.axles[1].wheel_detected.size(), 4u);
}

TEST(RosConversions, VehiclesToMsgFiltersByExistence)
{
  const std::vector<VehicleTemplate> templates{truck()};
  Vehicle probable;
  probable.id = 1;
  probable.existence_probability = 0.6;
  Vehicle improbable = probable;
  improbable.id = 2;
  improbable.existence_probability = 0.4;
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 3;

  const auto msg = conversions::vehiclesToMsg({probable, improbable}, templates, header, 0.5);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.header.stamp.sec, 3);
  ASSERT_EQ(msg.objects.size(), 1u);
  EXPECT_EQ(msg.objects[0].id, 1u);
  EXPECT_EQ(conversions::vehiclesToMsg({probable, improbable}, templates, header, 0.0)
    .objects.size(), 2u);
}
