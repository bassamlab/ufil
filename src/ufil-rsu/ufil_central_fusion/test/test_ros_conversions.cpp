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

#include <map>

#include "ufil_central_fusion_ros.hpp"

namespace conversions = ufil_central_fusion::conversions;

TEST(RosConversions, KnownStateEntriesAreAssigned)
{
  ufil_msgs::msg::Object object;
  object.state.state.x = 1.0;
  object.state.state.y = 2.0;
  object.state.state.v_x = 3.0;
  object.state.state.yaw = 0.5;
  object.state.state.yaw_rate = 0.3;
  object.state.covariance[0] = 0.1;     // x
  object.state.covariance[9] = 0.1;     // y
  object.state.covariance[18] = 0.2;    // v_x
  object.state.covariance[27] = -1.0;   // v_y unknown
  object.state.covariance[54] = 0.01;   // yaw
  object.state.covariance[63] = 0.01;   // yaw rate
  object.dimension.dimension.length = 4.5;
  object.dimension.covariance[0] = 0.1;
  object.dimension.covariance[4] = -1.0;  // width unknown
  object.existence_probability = 0.7;

  const auto measurement = conversions::measurementFromMsg(object);
  EXPECT_TRUE(measurement.hasX());
  EXPECT_TRUE(measurement.hasVx());
  EXPECT_FALSE(measurement.hasVy());
  EXPECT_TRUE(measurement.hasYaw());
  EXPECT_FALSE(measurement.hasYawRate());  // never used
  EXPECT_FLOAT_EQ(measurement.x(), 1.0f);
  EXPECT_FLOAT_EQ(measurement.yaw(), 0.5f);
  EXPECT_TRUE(measurement.dimension().hasLength());
  EXPECT_FALSE(measurement.dimension().hasWidth());
  EXPECT_FLOAT_EQ(measurement.existenceProbability(), 0.7f);
}

TEST(RosConversions, ObjectListAndTimestamp)
{
  ufil_msgs::msg::ObjectList msg;
  msg.objects.resize(2);
  msg.objects[1].state.state.x = 10.0;
  EXPECT_EQ(conversions::measurementsFromMsg(msg).size(), 2u);

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 5;
  EXPECT_EQ(
    ufil::to_nanoseconds<ufil::type::Timestamp>(conversions::timestampFromMsg(stamp)),
    12000000005LL);
}

TEST(RosConversions, EmptyTrackListKeepsHeader)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  const auto msg = conversions::tracksToMsg(
    std::map<ufil::type::Id, ufil_central_fusion::Track>{}, 0.2, header);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_TRUE(msg.objects.empty());
}
