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
#include <set>

#include "ufil_osn_tracker_ros.hpp"

namespace conversions = ufil_osn_tracker::conversions;

TEST(RosConversions, TimestampKeepsNanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1700000000;
  stamp.nanosec = 123456789;
  const auto timestamp = conversions::timestampFromMsg(stamp);
  EXPECT_EQ(
    ufil::to_nanoseconds<ufil::type::Timestamp>(timestamp),
    1700000000LL * 1000000000LL + 123456789LL);
}

TEST(RosConversions, PointCloudBecomesSingleDetection)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = "sensor";
  EXPECT_EQ(conversions::detectionsFromMsg(msg).size(), 1u);
}

TEST(RosConversions, EmptyListsKeepHeader)
{
  std_msgs::msg::Header header;
  header.frame_id = "sensor";
  header.stamp.sec = 7;

  const auto tracks =
    conversions::tracksToMsg(std::map<ufil::type::Id, ufil_osn_tracker::Track>{}, 0.2, header);
  EXPECT_EQ(tracks.header.frame_id, "sensor");
  EXPECT_TRUE(tracks.objects.empty());

  const auto measurements =
    conversions::measurementsToMsg(std::set<ufil_osn_tracker::Measurement>{}, header);
  EXPECT_EQ(measurements.header.stamp.sec, 7);

  const auto associations = conversions::associationsToMsg(
    std::map<ufil::type::Id, ufil_osn_tracker::Measurement>{}, header);
  EXPECT_EQ(associations.header.frame_id, "sensor");
}

TEST(RosConversions, AssociationsAreConverted)
{
  ufil_osn_tracker::Measurement measurement;
  measurement.x() = 1.5;
  measurement.y() = -2.0;
  std::map<ufil::type::Id, ufil_osn_tracker::Measurement> associations;
  associations.emplace(ufil::type::Id(), measurement);

  const auto msg = conversions::associationsToMsg(associations, std_msgs::msg::Header());
  ASSERT_EQ(msg.objects.size(), 1u);
  EXPECT_DOUBLE_EQ(msg.objects[0].state.state.x, 1.5);
  EXPECT_DOUBLE_EQ(msg.objects[0].state.state.y, -2.0);
}
