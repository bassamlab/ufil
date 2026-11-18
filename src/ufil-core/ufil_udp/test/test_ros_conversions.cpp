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

#include <rclcpp/time.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

#include "ufil_udp_ros.hpp"

using ufil_udp::Packet;
using ufil_udp::conversions::packetToMsg;
using ufil_udp::conversions::payloadFromMsg;

TEST(RosConversionsTest, PacketToMsg)
{
  Packet packet;
  packet.payload = {1, 2, 3};
  packet.source_address = "10.0.0.7";
  packet.source_port = 4711;

  const auto msg = packetToMsg(packet, rclcpp::Time(12, 345, RCL_ROS_TIME));
  EXPECT_EQ(msg.header.stamp.sec, 12);
  EXPECT_EQ(msg.header.stamp.nanosec, 345u);
  EXPECT_TRUE(msg.header.frame_id.empty());
  EXPECT_EQ(msg.address, "10.0.0.7");
  EXPECT_EQ(msg.src_port, 4711u);
  EXPECT_EQ(msg.data, (std::vector<std::uint8_t>{1, 2, 3}));
}

TEST(RosConversionsTest, PayloadFromMsgRefersToTheData)
{
  udp_msgs::msg::UdpPacket msg;
  msg.address = "10.0.0.7";
  msg.data = {4, 5};
  const auto payload = payloadFromMsg(msg);
  EXPECT_EQ(payload.data(), msg.data.data());
  ASSERT_EQ(payload.size(), 2u);
  EXPECT_EQ(payload[1], 5u);

  EXPECT_TRUE(payloadFromMsg(udp_msgs::msg::UdpPacket{}).empty());
}
