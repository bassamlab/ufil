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
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <class_loader/class_loader.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_ros/json_ros.hpp>

#include "fake_mqtt_broker.hpp"

namespace
{

constexpr char kFactoryName[] =
  "rclcpp_components::NodeFactoryTemplate<ufil_mqtt::MqttMessageConverterNode>";

bool isBuiltinParameter(const std::string & name)
{
  return name == "use_sim_time" || name == "start_type_description_service" ||
         name.rfind("qos_overrides.", 0) == 0;
}

std::string value(const diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key)
{
  for (const auto & entry : status.values) {
    if (entry.key == key) {
      return entry.value;
    }
  }
  return {};
}

}  // namespace

class ComponentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  rclcpp::Node::SharedPtr create(const rclcpp::NodeOptions & options)
  {
    auto factory = loader_.createInstance<rclcpp_components::NodeFactory>(kFactoryName);
    wrapper_ = factory->create_node_instance(options);
    // The registered class derives from rclcpp::Node only.
    return std::static_pointer_cast<rclcpp::Node>(wrapper_->get_node_instance());
  }

  /// Spins the component and a test node until condition is true or 10 s passed.
  bool spinUntil(rclcpp::Node::SharedPtr node, const std::function<bool()> & condition)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(test_node_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(std::chrono::milliseconds(50));
      if (condition()) {
        return true;
      }
    }
    return false;
  }

  class_loader::ClassLoader loader_{COMPONENT_LIBRARY};
  std::optional<rclcpp_components::NodeInstanceWrapper> wrapper_;
  rclcpp::Node::SharedPtr test_node_ = std::make_shared<rclcpp::Node>("component_test");
};

TEST_F(ComponentTest, IsRegistered)
{
  const auto classes = loader_.getAvailableClasses<rclcpp_components::NodeFactory>();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kFactoryName), classes.end());
}

TEST_F(ComponentTest, LoadsWithDefaultConfiguration)
{
  auto node = create(
    rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", DEFAULT_CONFIG, "-p", "use_sim_time:=true"}));
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(std::string(node->get_name()), "mqtt_message_converter");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_EQ(node->get_parameter("broker.host").as_string(), "localhost");

  // default.yaml and the declared parameters match exactly.
  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, value] : overrides) {
    if (!isBuiltinParameter(name)) {
      EXPECT_TRUE(node->has_parameter(name)) << name << " in default.yaml is not declared";
    }
  }
  const auto declared = node->list_parameters({}, 0).names;
  for (const auto & name : declared) {
    if (!isBuiltinParameter(name)) {
      EXPECT_TRUE(overrides.count(name)) << name << " is declared but missing in default.yaml";
    }
  }

  EXPECT_EQ(node->count_subscribers("/input"), 1u);
}

TEST_F(ComponentTest, InvalidParametersFailConstruction)
{
  EXPECT_THROW(
    create(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("broker.port", 0)})),
    std::invalid_argument);
  EXPECT_THROW(
    create(
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("message_type", "image")})),
    std::invalid_argument);
}

TEST_F(ComponentTest, ReportsDisconnectedWithoutBroker)
{
  auto node = create(
    rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("broker.host", "127.0.0.1"),
    rclcpp::Parameter("broker.port", ufil_mqtt::test::unusedPort()),
  }));

  std::optional<diagnostic_msgs::msg::DiagnosticStatus> status;
  auto subscription = test_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10, [&status](const diagnostic_msgs::msg::DiagnosticArray & msg) {
      for (const auto & entry : msg.status) {
        if (entry.name.find("MQTT connection") != std::string::npos &&
        entry.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR)
        {
          status = entry;
        }
      }
    });
  ASSERT_TRUE(spinUntil(node, [&status] {return status.has_value();}));
  EXPECT_NE(value(*status, "Connection state"), "connected");
  EXPECT_EQ(value(*status, "Messages sent"), "0");
  EXPECT_FALSE(value(*status, "Last error").empty());
}

TEST_F(ComponentTest, ForwardsMessagesToBroker)
{
  ufil_mqtt::test::FakeMqttBroker broker;
  auto node = create(
    rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("broker.host", "127.0.0.1"),
    rclcpp::Parameter("broker.port", broker.port()),
    rclcpp::Parameter("mqtt_topic", "lab/objects"),
  }));

  ufil_msgs::msg::ObjectList msg;
  msg.header.frame_id = "map";
  msg.objects.resize(1);
  msg.objects[0].id = 42;
  auto publisher = test_node_->create_publisher<ufil_msgs::msg::ObjectList>("/input", 10);

  // Publish until the converter is connected and forwards the message.
  std::optional<ufil_mqtt::test::FakeMqttBroker::Publication> publication;
  ASSERT_TRUE(
    spinUntil(
      node, [&] {
        publisher->publish(msg);
        publication = broker.waitForPublication(std::chrono::milliseconds(50));
        return publication.has_value();
      }));
  EXPECT_EQ(publication->topic, "lab/objects");
  EXPECT_EQ(publication->payload, ufil_ros::to_json(msg));
}
