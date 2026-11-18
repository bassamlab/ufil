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
#include <stdexcept>
#include <string>

#include "udp_receiver.hpp"
#include "ufil_osc/osc_message.hpp"
#include "ufil_osc/osc_sender.hpp"

using ufil_osc::OscConfig;
using ufil_osc::OscSender;
using ufil_osc::SendResult;

namespace
{

OscConfig localConfig(int port)
{
  OscConfig config;
  config.host = "127.0.0.1";
  config.port = port;
  config.address = "/test/objects";
  return config;
}

}  // namespace

TEST(OscSender, RejectsInvalidConfiguration)
{
  OscConfig config;
  config.address = "objects";
  EXPECT_THROW(OscSender{config}, std::invalid_argument);
}

TEST(OscSender, RejectsUnresolvableHost)
{
  OscConfig config;
  config.host = "no-such-host.invalid";
  EXPECT_THROW(OscSender{config}, std::runtime_error);
}

TEST(OscSender, SendsStringMessage)
{
  ufil_osc::test::UdpReceiver receiver;
  OscSender sender(localConfig(receiver.port()));
  EXPECT_EQ(sender.endpoint(), "127.0.0.1:" + std::to_string(receiver.port()));
  EXPECT_FALSE(sender.status().last_result);

  const std::string payload = R"({"object_list":{"objects":[]}})";
  EXPECT_EQ(sender.send(payload), SendResult::kSent);
  const auto message = receiver.receive(std::chrono::seconds(2));
  ASSERT_TRUE(message);
  EXPECT_EQ(message->address, "/test/objects");
  EXPECT_EQ(message->argument, payload);

  const auto status = sender.status();
  EXPECT_EQ(status.sent, 1u);
  EXPECT_EQ(status.failed, 0u);
  EXPECT_EQ(status.last_result, SendResult::kSent);
  EXPECT_EQ(status.last_packet_size, ufil_osc::stringMessageSize("/test/objects", payload.size()));
}

TEST(OscSender, PrefersIpv4)
{
  ufil_osc::test::UdpReceiver receiver;
  OscConfig config = localConfig(receiver.port());
  config.host = "localhost";
  OscSender sender(config);
  EXPECT_EQ(sender.endpoint(), "127.0.0.1:" + std::to_string(receiver.port()));
  EXPECT_EQ(sender.send("{}"), SendResult::kSent);
  EXPECT_TRUE(receiver.receive(std::chrono::seconds(2)));
}

TEST(OscSender, SendsLargestPacket)
{
  ufil_osc::test::UdpReceiver receiver;
  OscSender sender(localConfig(receiver.port()));
  // "/test/objects" takes 16 bytes and ",s" 4 bytes, the string its length + 1 rounded up.
  const std::string payload(OscSender::kMaxPacketSize - 16 - 4 - 4, 'x');
  ASSERT_LE(ufil_osc::stringMessageSize("/test/objects", payload.size()),
    OscSender::kMaxPacketSize);
  EXPECT_EQ(sender.send(payload), SendResult::kSent);
  const auto message = receiver.receive(std::chrono::seconds(2));
  ASSERT_TRUE(message);
  EXPECT_EQ(message->argument, payload);
}

TEST(OscSender, RejectsPayloadWithNul)
{
  ufil_osc::test::UdpReceiver receiver;
  OscSender sender(localConfig(receiver.port()));
  EXPECT_EQ(sender.send(std::string("a\0b", 3)), SendResult::kFailed);
  EXPECT_EQ(sender.status().failed, 1u);
  EXPECT_EQ(sender.status().last_result, SendResult::kFailed);
}

TEST(OscSender, RejectsTooLargePayload)
{
  ufil_osc::test::UdpReceiver receiver;
  OscSender sender(localConfig(receiver.port()));
  EXPECT_EQ(sender.send(std::string(70000, 'x')), SendResult::kTooLarge);
  EXPECT_FALSE(receiver.receive(std::chrono::milliseconds(200)));

  const auto status = sender.status();
  EXPECT_EQ(status.sent, 0u);
  EXPECT_EQ(status.too_large, 1u);
  EXPECT_EQ(status.last_result, SendResult::kTooLarge);
  EXPECT_FALSE(status.last_error.empty());
  EXPECT_GT(status.last_packet_size, OscSender::kMaxPacketSize);
}
