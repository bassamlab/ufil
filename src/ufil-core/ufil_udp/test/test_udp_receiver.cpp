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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ufil_udp/udp_receiver.hpp"
#include "ufil_udp/udp_sender.hpp"

using ufil_udp::Packet;
using ufil_udp::ReceiverConfig;
using ufil_udp::SenderConfig;
using ufil_udp::UdpReceiver;
using ufil_udp::UdpSender;

namespace
{

constexpr auto kTimeout = std::chrono::seconds(5);

/// Collects the packets of a receiver.
class PacketQueue
{
public:
  void push(Packet && packet)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    packets_.push_back(std::move(packet));
    condition_.notify_all();
  }

  /// @return true if at least count packets arrived within the timeout.
  bool waitFor(std::size_t count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, kTimeout, [&] {return packets_.size() >= count;});
  }

  std::vector<Packet> packets()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return packets_;
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<Packet> packets_;
};

ReceiverConfig anyPort()
{
  ReceiverConfig config;
  config.listen_port = 0;
  return config;
}

UdpSender senderTo(const UdpReceiver & receiver)
{
  SenderConfig config;
  config.target_address = "127.0.0.1";
  config.target_port = receiver.localPort();
  return UdpSender(config);
}

std::vector<std::uint8_t> bytes(const std::string & text)
{
  return {text.begin(), text.end()};
}

}  // namespace

TEST(UdpReceiverTest, ReceivesPacketsOnLocalhost)
{
  PacketQueue queue;
  UdpReceiver receiver(anyPort(), [&](Packet && packet) {queue.push(std::move(packet));});
  ASSERT_NE(receiver.localPort(), 0u);
  EXPECT_FALSE(receiver.status().last_packet_time.has_value());

  auto sender = senderTo(receiver);
  ASSERT_FALSE(sender.send(bytes("first")));
  ASSERT_FALSE(sender.send(bytes("second")));
  ASSERT_FALSE(sender.send(bytes("")));
  ASSERT_TRUE(queue.waitFor(3));

  const auto packets = queue.packets();
  EXPECT_EQ(packets[0].payload, bytes("first"));
  EXPECT_EQ(packets[1].payload, bytes("second"));
  EXPECT_TRUE(packets[2].payload.empty());
  EXPECT_EQ(packets[0].source_address, "127.0.0.1");
  EXPECT_NE(packets[0].source_port, 0u);

  const auto status = receiver.status();
  EXPECT_TRUE(status.receiving);
  EXPECT_EQ(status.local_port, receiver.localPort());
  EXPECT_EQ(status.packets, 3u);
  EXPECT_EQ(status.bytes, 11u);
  EXPECT_EQ(status.errors, 0u);
  EXPECT_TRUE(status.last_packet_time.has_value());
  EXPECT_TRUE(status.last_error.empty());
}

TEST(UdpReceiverTest, ReceivesLargePacketsCompletely)
{
  PacketQueue queue;
  UdpReceiver receiver(anyPort(), [&](Packet && packet) {queue.push(std::move(packet));});
  std::vector<std::uint8_t> payload(60000);
  for (std::size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<std::uint8_t>(i);
  }
  auto sender = senderTo(receiver);
  ASSERT_FALSE(sender.send(payload));
  ASSERT_TRUE(queue.waitFor(1));
  EXPECT_EQ(queue.packets()[0].payload, payload);
}

TEST(UdpReceiverTest, RequiresPacketCallback)
{
  EXPECT_THROW(UdpReceiver(anyPort(), {}), std::invalid_argument);
}

TEST(UdpReceiverTest, PortInUseThrows)
{
  UdpReceiver first(anyPort(), [](Packet &&) {});
  ReceiverConfig config;
  config.listen_port = first.localPort();
  EXPECT_THROW(UdpReceiver(config, [](Packet &&) {}), std::runtime_error);
}

TEST(UdpReceiverTest, CallbackExceptionIsCountedAndReceptionContinues)
{
  PacketQueue queue;
  std::vector<std::string> errors;
  bool first = true;
  UdpReceiver receiver(
    anyPort(),
    [&](Packet && packet) {
      if (first) {
        first = false;
        throw std::runtime_error("broken");
      }
      queue.push(std::move(packet));
    },
    [&](const std::string & error) {errors.push_back(error);});

  auto sender = senderTo(receiver);
  ASSERT_FALSE(sender.send(bytes("dropped")));
  ASSERT_FALSE(sender.send(bytes("delivered")));
  ASSERT_TRUE(queue.waitFor(1));
  EXPECT_EQ(queue.packets()[0].payload, bytes("delivered"));

  receiver.stop();
  const auto status = receiver.status();
  EXPECT_EQ(status.packets, 2u);
  EXPECT_EQ(status.errors, 1u);
  EXPECT_NE(status.last_error.find("broken"), std::string::npos);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0], status.last_error);
}

TEST(UdpReceiverTest, NoCallbackAfterStop)
{
  PacketQueue queue;
  UdpReceiver receiver(anyPort(), [&](Packet && packet) {queue.push(std::move(packet));});
  auto sender = senderTo(receiver);
  receiver.stop();
  receiver.stop();
  EXPECT_FALSE(receiver.status().receiving);

  (void)sender.send(bytes("late"));
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(queue.packets().empty());
}

TEST(UdpReceiverTest, DestructionWhileReceivingDoesNotHang)
{
  for (int round = 0; round < 20; ++round) {
    auto calls = std::make_shared<std::atomic<int>>(0);
    auto receiver = std::make_unique<UdpReceiver>(
      anyPort(), [calls](Packet &&) {
        ++*calls;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      });

    std::atomic<bool> sending{true};
    auto sender = senderTo(*receiver);
    std::thread flood([&] {
        while (sending) {
          (void)sender.send(bytes("flood"));
        }
      });
    while (*calls == 0) {
      std::this_thread::yield();
    }

    receiver.reset();
    const int calls_after_destruction = *calls;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_EQ(*calls, calls_after_destruction);
    sending = false;
    flood.join();
  }
}
