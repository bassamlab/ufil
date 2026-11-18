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

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "UfilMqttParameterRos.hpp"

using ufil_mqtt::MessageType;
using ufil_mqtt::UfilMqttParameterRos;

class ParameterRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static UfilMqttParameterRos load(const std::vector<rclcpp::Parameter> & overrides)
  {
    auto node = std::make_shared<rclcpp::Node>(
      "converter", "ns", rclcpp::NodeOptions().parameter_overrides(overrides));
    return UfilMqttParameterRos(
      node->get_node_parameters_interface(), node->get_fully_qualified_name());
  }
};

TEST_F(ParameterRosTest, Defaults)
{
  const auto parameters = load({});
  EXPECT_EQ(parameters.settings().message_type, MessageType::kObjectList);
  EXPECT_DOUBLE_EQ(parameters.settings().reconnect_interval, 2.0);
  EXPECT_EQ(parameters.config().protocol, "tcp");
  EXPECT_EQ(parameters.config().host, "localhost");
  EXPECT_EQ(parameters.config().port, 1883);
  EXPECT_EQ(parameters.config().topic, "ufil/object_list");
  // An empty client ID is replaced by the fully qualified node name.
  EXPECT_EQ(parameters.config().client_id, "ns/converter");
}

TEST_F(ParameterRosTest, ReadsValues)
{
  const auto parameters = load(
  {
    rclcpp::Parameter("message_type", "occupancy_grid"),
    rclcpp::Parameter("broker.protocol", "ssl"),
    rclcpp::Parameter("broker.host", "192.168.1.77"),
    rclcpp::Parameter("broker.port", 8883),
    rclcpp::Parameter("broker.client_id", "lab"),
    rclcpp::Parameter("broker.reconnect_interval", 0.5),
    rclcpp::Parameter("mqtt_topic", "lab/grid"),
  });
  EXPECT_EQ(parameters.settings().message_type, MessageType::kOccupancyGrid);
  EXPECT_DOUBLE_EQ(parameters.settings().reconnect_interval, 0.5);
  EXPECT_EQ(parameters.config().protocol, "ssl");
  EXPECT_EQ(parameters.config().host, "192.168.1.77");
  EXPECT_EQ(parameters.config().port, 8883);
  EXPECT_EQ(parameters.config().client_id, "lab");
  EXPECT_EQ(parameters.config().topic, "lab/grid");
}

TEST_F(ParameterRosTest, RejectsInvalidValues)
{
  EXPECT_THROW(load({rclcpp::Parameter("message_type", "image")}), std::invalid_argument);
  EXPECT_THROW(load({rclcpp::Parameter("broker.protocol", "udp")}), std::invalid_argument);
  EXPECT_THROW(load({rclcpp::Parameter("broker.host", "")}), std::invalid_argument);
  EXPECT_THROW(load({rclcpp::Parameter("broker.port", 70000)}), std::invalid_argument);
  EXPECT_THROW(load({rclcpp::Parameter("broker.reconnect_interval", 0.0)}), std::invalid_argument);
  EXPECT_THROW(load({rclcpp::Parameter("mqtt_topic", "lab/#")}), std::invalid_argument);
}

TEST_F(ParameterRosTest, RejectsWrongTypes)
{
  EXPECT_THROW(
    load({rclcpp::Parameter("broker.port", "1883")}),
    rclcpp::exceptions::InvalidParameterTypeException);
}
