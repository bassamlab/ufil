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

#include <cstdint>
#include <vector>

#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_message_filter_ros.hpp"

namespace conversions = ufil_message_filter::conversions;

namespace
{

ufil_msgs::msg::Object makeObject(std::uint32_t id, double x, double y)
{
  ufil_msgs::msg::Object object;
  object.id = id;
  object.state.state.x = x;
  object.state.state.y = y;
  object.state.state.v_x = 1.5;
  object.existence_probability = 0.9;
  return object;
}

ufil_msgs::msg::ObjectList makeList()
{
  ufil_msgs::msg::ObjectList msg;
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 42;
  msg.objects = {makeObject(1, 1.0, 2.0), makeObject(2, 3.0, 4.0), makeObject(3, 5.0, 6.0)};
  return msg;
}

}  // namespace

TEST(RosConversionsTest, PositionsFromMsg)
{
  const auto positions = conversions::positionsFromMsg(makeList());
  ASSERT_EQ(positions.size(), 3u);
  EXPECT_DOUBLE_EQ(positions[1].x, 3.0);
  EXPECT_DOUBLE_EQ(positions[1].y, 4.0);
  EXPECT_TRUE(conversions::positionsFromMsg(ufil_msgs::msg::ObjectList()).empty());
}

TEST(RosConversionsTest, SelectObjectsKeepsHeaderAndObjects)
{
  const auto msg = makeList();
  const auto selected = conversions::selectObjects(msg, {0, 2});
  EXPECT_EQ(selected.header, msg.header);
  ASSERT_EQ(selected.objects.size(), 2u);
  EXPECT_EQ(selected.objects[0], msg.objects[0]);
  EXPECT_EQ(selected.objects[1], msg.objects[2]);

  EXPECT_TRUE(conversions::selectObjects(msg, {}).objects.empty());
}
