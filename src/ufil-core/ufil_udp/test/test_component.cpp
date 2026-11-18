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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <class_loader/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

#include "ufil_udp/udp_receiver.hpp"
#include "ufil_udp/udp_sender.hpp"

namespace
{

constexpr char kReceiverFactory[] =
  "rclcpp_components::NodeFactoryTemplate<ufil_udp::UdpReceiverNode>";
constexpr char kSenderFactory[] =
  "rclcpp_components::NodeFactoryTemplate<ufil_udp::UdpSenderNode>";
constexpr auto kTimeout = std::chrono::seconds(5);

bool isBuiltinParameter(const std::string & name)
{
  return name == "use_sim_time" || name == "start_type_description_service" ||
         name.rfind("qos_overrides.", 0) == 0;
}

/// Port that is free right now, chosen by the operating system.
std::uint16_t freePort()
{
  ufil_udp::ReceiverConfig config;
  config.listen_port = 0;
  return ufil_udp::UdpReceiver(config, [](ufil_udp::Packet &&) {}).localPort();
}

template<typename Predicate>
bool waitUntil(Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return true;
}

}  // namespace

class ComponentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  rclcpp_components::NodeInstanceWrapper create(
    const char * factory_name, const rclcpp::NodeOptions & options)
  {
    auto factory = loader_.createInstance<rclcpp_components::NodeFactory>(factory_name);
    return factory->create_node_instance(options);
  }

  static std::shared_ptr<rclcpp::Node> asNode(const rclcpp_components::NodeInstanceWrapper & w)
  {
    // The registered classes derive from rclcpp::Node only.
    return std::static_pointer_cast<rclcpp::Node>(w.get_node_instance());
  }

  /// default.yaml and the declared parameters match exactly.
  static void expectDefaultConfigMatches(const rclcpp::Node & node)
  {
    const auto & overrides =
      const_cast<rclcpp::Node &>(node).get_node_parameters_interface()->get_parameter_overrides();
    for (const auto & [name, value] : overrides) {
      if (!isBuiltinParameter(name)) {
        EXPECT_TRUE(node.has_parameter(name)) << name << " in default.yaml is not declared";
      }
    }
    for (const auto & name : node.list_parameters({}, 0).names) {
      if (!isBuiltinParameter(name)) {
        EXPECT_TRUE(overrides.count(name)) << name << " is declared but missing in default.yaml";
      }
    }
  }

  static rclcpp::NodeOptions defaultConfig()
  {
    return rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", DEFAULT_CONFIG, "-p", "use_sim_time:=true"});
  }

  class_loader::ClassLoader loader_{COMPONENT_LIBRARY};
};

TEST_F(ComponentTest, AreRegistered)
{
  const auto classes = loader_.getAvailableClasses<rclcpp_components::NodeFactory>();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kReceiverFactory), classes.end());
  EXPECT_NE(std::find(classes.begin(), classes.end(), kSenderFactory), classes.end());
}

TEST_F(ComponentTest, ReceiverLoadsWithDefaultConfiguration)
{
  auto wrapper = create(kReceiverFactory, defaultConfig());
  auto node = asNode(wrapper);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(std::string(node->get_name()), "udp_receiver");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_EQ(node->get_parameter("listen_port").as_int(), 5000);
  expectDefaultConfigMatches(*node);
  EXPECT_EQ(node->count_publishers("udp_output"), 1u);
}

TEST_F(ComponentTest, SenderLoadsWithDefaultConfiguration)
{
  auto wrapper = create(kSenderFactory, defaultConfig());
  auto node = asNode(wrapper);
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(std::string(node->get_name()), "udp_sender");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_EQ(node->get_parameter("target_address").as_string(), "127.0.0.1");
  EXPECT_EQ(node->get_parameter("target_port").as_int(), 5000);
  expectDefaultConfigMatches(*node);
  EXPECT_EQ(node->count_subscribers("udp_input"), 1u);
}

TEST_F(ComponentTest, InvalidParametersFailConstruction)
{
  EXPECT_THROW(
    create(
      kReceiverFactory,
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("listen_port", 0)})),
    std::invalid_argument);
  EXPECT_THROW(
    create(
      kSenderFactory,
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("target_address", "x")})),
    std::invalid_argument);
}

TEST_F(ComponentTest, ReceiverOnUsedPortFailsConstruction)
{
  ufil_udp::ReceiverConfig config;
  config.listen_port = 0;
  ufil_udp::UdpReceiver blocker(config, [](ufil_udp::Packet &&) {});
  EXPECT_THROW(
    create(
      kReceiverFactory,
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("listen_port", static_cast<int>(blocker.localPort()))})),
    std::runtime_error);
}

TEST_F(ComponentTest, ReceiverPublishesPacketsAndUnloadsWhileReceiving)
{
  const auto port = freePort();
  auto listener = std::make_shared<rclcpp::Node>("listener");
  std::mutex mutex;
  std::optional<udp_msgs::msg::UdpPacket> received;
  auto subscription = listener->create_subscription<udp_msgs::msg::UdpPacket>(
    "udp_output", rclcpp::QoS(100), [&](const udp_msgs::msg::UdpPacket & msg) {
      std::lock_guard<std::mutex> lock(mutex);
      if (!received) {
        received = msg;
      }
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  std::thread spinner([&] {executor.spin();});

  ufil_udp::SenderConfig sender_config;
  sender_config.target_port = port;
  ufil_udp::UdpSender sender(sender_config);

  for (int round = 0; round < 5; ++round) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      received.reset();
    }
    auto wrapper = create(
      kReceiverFactory,
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("listen_port", port)}));
    auto node = asNode(wrapper);
    executor.add_node(node);
    EXPECT_TRUE(waitUntil([&] {return listener->count_publishers("udp_output") == 1u;}));

    // Packets keep arriving while the component is unloaded.
    std::atomic<bool> sending{true};
    std::thread flood([&] {
        const std::vector<std::uint8_t> payload{'c', 'a', 'm'};
        while (sending) {
          (void)sender.send(payload);
          std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
      });
    const bool arrived = waitUntil(
      [&] {
        std::lock_guard<std::mutex> lock(mutex);
        return received.has_value();
      });
    executor.remove_node(node);
    node.reset();
    wrapper = {};
    sending = false;
    flood.join();

    EXPECT_TRUE(arrived);
    std::lock_guard<std::mutex> lock(mutex);
    if (received) {
      EXPECT_EQ(received->data, (std::vector<std::uint8_t>{'c', 'a', 'm'}));
      EXPECT_EQ(received->address, "127.0.0.1");
      EXPECT_NE(received->src_port, 0u);
      EXPECT_NE(rclcpp::Time(received->header.stamp).nanoseconds(), 0);
    }
  }

  executor.cancel();
  spinner.join();
}

TEST_F(ComponentTest, SenderSendsReceivedMessages)
{
  std::mutex mutex;
  std::vector<ufil_udp::Packet> packets;
  ufil_udp::ReceiverConfig receiver_config;
  receiver_config.listen_port = 0;
  ufil_udp::UdpReceiver receiver(
    receiver_config, [&](ufil_udp::Packet && packet) {
      std::lock_guard<std::mutex> lock(mutex);
      packets.push_back(std::move(packet));
    });

  auto wrapper = create(
    kSenderFactory,
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("target_port", static_cast<int>(receiver.localPort()))}));
  auto node = asNode(wrapper);
  auto talker = std::make_shared<rclcpp::Node>("talker");
  auto publisher = talker->create_publisher<udp_msgs::msg::UdpPacket>("udp_input", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&] {executor.spin();});
  EXPECT_TRUE(waitUntil([&] {return publisher->get_subscription_count() == 1u;}));

  udp_msgs::msg::UdpPacket msg;
  msg.address = "192.0.2.1";  // ignored, the sender has a fixed target
  msg.data = {0xca, 0xfe};
  publisher->publish(msg);
  const bool arrived = waitUntil(
    [&] {
      std::lock_guard<std::mutex> lock(mutex);
      return !packets.empty();
    });
  executor.cancel();
  spinner.join();

  EXPECT_TRUE(arrived);
  std::lock_guard<std::mutex> lock(mutex);
  for (const auto & packet : packets) {
    EXPECT_EQ(packet.payload, (std::vector<std::uint8_t>{0xca, 0xfe}));
  }
}
