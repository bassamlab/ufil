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
#include <memory>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "UfilUdpReceiverParameterRos.hpp"
#include "UfilUdpSenderParameterRos.hpp"

using ufil_udp::UfilUdpReceiverParameterRos;
using ufil_udp::UfilUdpSenderParameterRos;

class ParameterRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<rclcpp::Node> makeNode(const std::vector<rclcpp::Parameter> & overrides)
  {
    return std::make_shared<rclcpp::Node>(
      "parameter_test", rclcpp::NodeOptions().parameter_overrides(overrides));
  }
};

TEST_F(ParameterRosTest, ReceiverDefaultsAndOverrides)
{
  auto node = makeNode({});
  UfilUdpReceiverParameterRos defaults(node->get_node_parameters_interface());
  EXPECT_EQ(defaults.config().listen_port, 5000u);
  EXPECT_EQ(node->get_parameter("listen_port").as_int(), 5000);

  UfilUdpReceiverParameterRos parameters(
    makeNode({rclcpp::Parameter("listen_port", 65535)})->get_node_parameters_interface());
  EXPECT_EQ(parameters.config().listen_port, 65535u);
}

TEST_F(ParameterRosTest, ReceiverInvalidPortThrows)
{
  for (const std::int64_t port : {std::int64_t{0}, std::int64_t{-1}, std::int64_t{65536}}) {
    EXPECT_THROW(
      UfilUdpReceiverParameterRos(
        makeNode({rclcpp::Parameter("listen_port", port)})->get_node_parameters_interface()),
      std::invalid_argument) << port;
  }
}

TEST_F(ParameterRosTest, SenderDefaultsAndOverrides)
{
  auto node = makeNode({});
  UfilUdpSenderParameterRos defaults(node->get_node_parameters_interface());
  EXPECT_EQ(defaults.config().target_address, "127.0.0.1");
  EXPECT_EQ(defaults.config().target_port, 5000u);

  UfilUdpSenderParameterRos parameters(
    makeNode(
      {rclcpp::Parameter("target_address", "192.168.0.255"),
        rclcpp::Parameter("target_port", 4000)})->get_node_parameters_interface());
  EXPECT_EQ(parameters.config().target_address, "192.168.0.255");
  EXPECT_EQ(parameters.config().target_port, 4000u);
}

TEST_F(ParameterRosTest, SenderInvalidParametersThrow)
{
  EXPECT_THROW(
    UfilUdpSenderParameterRos(
      makeNode({rclcpp::Parameter("target_address", "not-an-address")})
      ->get_node_parameters_interface()),
    std::invalid_argument);
  EXPECT_THROW(
    UfilUdpSenderParameterRos(
      makeNode({rclcpp::Parameter("target_port", 0)})->get_node_parameters_interface()),
    std::invalid_argument);
  EXPECT_THROW(
    UfilUdpSenderParameterRos(
      makeNode({rclcpp::Parameter("target_port", 70000)})->get_node_parameters_interface()),
    std::invalid_argument);
}

TEST_F(ParameterRosTest, ParametersAreReadOnly)
{
  auto receiver_node = makeNode({});
  UfilUdpReceiverParameterRos receiver(receiver_node->get_node_parameters_interface());
  EXPECT_FALSE(receiver_node->set_parameter(rclcpp::Parameter("listen_port", 6000)).successful);

  auto sender_node = makeNode({});
  UfilUdpSenderParameterRos sender(sender_node->get_node_parameters_interface());
  EXPECT_FALSE(
    sender_node->set_parameter(rclcpp::Parameter("target_address", "10.0.0.1")).successful);
  EXPECT_FALSE(sender_node->set_parameter(rclcpp::Parameter("target_port", 6000)).successful);
}
