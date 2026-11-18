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

#include "UfilOscParameterRos.hpp"

using ufil_osc::MessageType;
using ufil_osc::UfilOscParameterRos;

class ParameterRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static UfilOscParameterRos load(const std::vector<rclcpp::Parameter> & overrides)
  {
    auto node = std::make_shared<rclcpp::Node>(
      "converter", rclcpp::NodeOptions().parameter_overrides(overrides));
    return UfilOscParameterRos(node->get_node_parameters_interface());
  }
};

TEST_F(ParameterRosTest, Defaults)
{
  const auto parameters = load({});
  EXPECT_EQ(parameters.messageType(), MessageType::kObjectList);
  EXPECT_EQ(parameters.config().host, "127.0.0.1");
  EXPECT_EQ(parameters.config().port, 9000);
  EXPECT_EQ(parameters.config().address, "/objects");
}

TEST_F(ParameterRosTest, ReadsValues)
{
  const auto parameters = load(
  {
    rclcpp::Parameter("message_type", "occupancy_grid"),
    rclcpp::Parameter("receiver.host", "192.168.1.77"),
    rclcpp::Parameter("receiver.port", 8300),
    rclcpp::Parameter("osc_address", "/ssl1"),
  });
  EXPECT_EQ(parameters.messageType(), MessageType::kOccupancyGrid);
  EXPECT_EQ(parameters.config().host, "192.168.1.77");
  EXPECT_EQ(parameters.config().port, 8300);
  EXPECT_EQ(parameters.config().address, "/ssl1");
}

TEST_F(ParameterRosTest, RejectsInvalidValues)
{
  EXPECT_THROW(load({rclcpp::Parameter("message_type", "image")}), std::invalid_argument);
  EXPECT_THROW(load({rclcpp::Parameter("receiver.host", "")}), std::invalid_argument);
  EXPECT_THROW(load({rclcpp::Parameter("receiver.port", 0)}), std::invalid_argument);
  EXPECT_THROW(load({rclcpp::Parameter("receiver.port", 70000)}), std::invalid_argument);
  EXPECT_THROW(load({rclcpp::Parameter("osc_address", "objects")}), std::invalid_argument);
}

TEST_F(ParameterRosTest, RejectsWrongTypes)
{
  EXPECT_THROW(
    load({rclcpp::Parameter("receiver.port", "9000")}),
    rclcpp::exceptions::InvalidParameterTypeException);
}
