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

#ifndef UFIL_UDP__COMPONENT__UFIL_UDP_ROS_HPP_
#define UFIL_UDP__COMPONENT__UFIL_UDP_ROS_HPP_

// Conversions between udp_msgs/UdpPacket and the ROS-independent ufil_udp types.

#include <cstdint>
#include <span>  // NOLINT(build/include_order): cpplint does not know C++20 headers
#include <utility>

#include <rclcpp/time.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

#include "ufil_udp/packet.hpp"

namespace ufil_udp::conversions
{

/// Converts a received packet into a message stamped with the reception time.
inline udp_msgs::msg::UdpPacket packetToMsg(Packet packet, const rclcpp::Time & stamp)
{
  udp_msgs::msg::UdpPacket msg;
  msg.header.stamp = stamp;
  msg.address = std::move(packet.source_address);
  msg.src_port = packet.source_port;
  msg.data = std::move(packet.payload);
  return msg;
}

/// Payload to send; address and src_port of the message are ignored, the sender has a fixed
/// target. The span refers to the data of the message.
inline std::span<const std::uint8_t> payloadFromMsg(const udp_msgs::msg::UdpPacket & msg)
{
  return msg.data;
}

}  // namespace ufil_udp::conversions

#endif  // UFIL_UDP__COMPONENT__UFIL_UDP_ROS_HPP_
