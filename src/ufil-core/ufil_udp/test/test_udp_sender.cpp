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

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <vector>

#include "ufil_udp/udp_receiver.hpp"
#include "ufil_udp/udp_sender.hpp"

using ufil_udp::Packet;
using ufil_udp::ReceiverConfig;
using ufil_udp::SenderConfig;
using ufil_udp::UdpReceiver;
using ufil_udp::UdpSender;

TEST(UdpSenderTest, InvalidConfigurationThrows)
{
  SenderConfig config;
  config.target_address = "localhost";
  EXPECT_THROW(UdpSender{config}, std::invalid_argument);
  config = SenderConfig{};
  config.target_port = 0;
  EXPECT_THROW(UdpSender{config}, std::invalid_argument);
}

TEST(UdpSenderTest, AcceptsBroadcastTarget)
{
  SenderConfig config;
  config.target_address = "255.255.255.255";
  UdpSender sender(config);
  EXPECT_EQ(sender.config().target_address, "255.255.255.255");
}

TEST(UdpSenderTest, CountsSentPackets)
{
  ReceiverConfig receiver_config;
  receiver_config.listen_port = 0;
  UdpReceiver receiver(receiver_config, [](Packet &&) {});

  SenderConfig config;
  config.target_port = receiver.localPort();
  UdpSender sender(config);
  EXPECT_FALSE(sender.status().last_packet_time.has_value());

  const std::vector<std::uint8_t> payload{1, 2, 3, 4};
  EXPECT_FALSE(sender.send(payload));
  EXPECT_FALSE(sender.send(payload));

  const auto status = sender.status();
  EXPECT_EQ(status.packets, 2u);
  EXPECT_EQ(status.bytes, 8u);
  EXPECT_EQ(status.errors, 0u);
  EXPECT_TRUE(status.last_packet_time.has_value());

  for (int i = 0; i < 100 && receiver.status().packets < 2; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(receiver.status().packets, 2u);
}

TEST(UdpSenderTest, FailedSendIsReportedAndCounted)
{
  UdpSender sender(SenderConfig{});
  // Larger than the maximum UDP payload.
  const std::vector<std::uint8_t> payload(70000);
  const auto error = sender.send(payload);
  ASSERT_TRUE(error.has_value());
  EXPECT_FALSE(error->empty());

  const auto status = sender.status();
  EXPECT_EQ(status.packets, 0u);
  EXPECT_EQ(status.errors, 1u);
  EXPECT_EQ(status.last_error, *error);
}
