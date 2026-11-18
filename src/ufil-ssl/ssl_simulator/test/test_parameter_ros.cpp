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

#include "SslSimulatorParameterRos.hpp"

using ssl_simulator::SimulatorConfig;
using ssl_simulator::SslSimulatorParameterRos;

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

TEST_F(ParameterRosTest, Defaults)
{
  auto node = makeNode({});
  SslSimulatorParameterRos parameters(node->get_node_parameters_interface());

  const SimulatorConfig defaults;
  EXPECT_EQ(parameters.settings().node_id, 1);
  EXPECT_DOUBLE_EQ(parameters.settings().frame_period, 0.023);
  EXPECT_DOUBLE_EQ(parameters.config().width, defaults.width);
  EXPECT_DOUBLE_EQ(parameters.config().height, defaults.height);
  EXPECT_DOUBLE_EQ(parameters.config().resolution, defaults.resolution);
  EXPECT_DOUBLE_EQ(parameters.config().pose.z, 0.011);
  EXPECT_DOUBLE_EQ(parameters.config().pose.heading, 0.0);
  EXPECT_EQ(parameters.config().threshold, 10);
  EXPECT_DOUBLE_EQ(parameters.config().stale_duration, 0.5);
  EXPECT_DOUBLE_EQ(
    parameters.config().wheel_contact_size, ssl_simulator::kWheelContactSizeFullScale);
  EXPECT_FALSE(parameters.config().seed.has_value());
  EXPECT_TRUE(node->describe_parameter("heading").read_only);
  EXPECT_FALSE(node->describe_parameter("heading").description.empty());
}

TEST_F(ParameterRosTest, ReadsOverrides)
{
  auto node = makeNode(
  {
    rclcpp::Parameter("node_id", 3),
    rclcpp::Parameter("x", 205.0),
    rclcpp::Parameter("y", 27.92),
    rclcpp::Parameter("heading", 1.5),
    rclcpp::Parameter("width", 8.64),
    rclcpp::Parameter("height", 17.28),
    rclcpp::Parameter("threshold", 20),
    rclcpp::Parameter("stale_seconds", 1.0),
    rclcpp::Parameter("microscale", true),
  });
  SslSimulatorParameterRos parameters(node->get_node_parameters_interface());

  EXPECT_EQ(parameters.settings().node_id, 3);
  EXPECT_DOUBLE_EQ(parameters.config().pose.x, 205.0);
  EXPECT_DOUBLE_EQ(parameters.config().pose.y, 27.92);
  EXPECT_DOUBLE_EQ(parameters.config().pose.heading, 1.5);
  EXPECT_EQ(parameters.config().columns(), 44u);
  EXPECT_EQ(parameters.config().rows(), 87u);
  EXPECT_EQ(parameters.config().threshold, 20);
  EXPECT_DOUBLE_EQ(parameters.config().stale_duration, 1.0);
  EXPECT_DOUBLE_EQ(
    parameters.config().wheel_contact_size, ssl_simulator::kWheelContactSizeMicroscale);
}

TEST_F(ParameterRosTest, InvalidValuesThrow)
{
  const std::vector<rclcpp::Parameter> invalid{
    rclcpp::Parameter("node_id", -1),
    rclcpp::Parameter("frame_period", 0.0),
    rclcpp::Parameter("width", 0.0),
    rclcpp::Parameter("height", -5.0),
    rclcpp::Parameter("resolution", 0.0),
    rclcpp::Parameter("threshold", 0),
    rclcpp::Parameter("threshold", 4095),
    rclcpp::Parameter("stale_seconds", 0.0),
  };
  for (const auto & parameter : invalid) {
    auto node = makeNode({parameter});
    EXPECT_THROW(
      SslSimulatorParameterRos(node->get_node_parameters_interface()),
      std::invalid_argument) << parameter.get_name() << " = " << parameter.value_to_string();
  }
}

TEST_F(ParameterRosTest, WrongTypeThrows)
{
  auto node = makeNode({rclcpp::Parameter("threshold", 10.0)});
  EXPECT_THROW(
    SslSimulatorParameterRos(node->get_node_parameters_interface()),
    rclcpp::exceptions::InvalidParameterTypeException);
}
