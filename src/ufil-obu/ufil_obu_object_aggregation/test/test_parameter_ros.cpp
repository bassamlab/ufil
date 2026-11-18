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
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "UfilObuObjectAggregationParameterRos.hpp"

using ufil_obu_object_aggregation::AggregatorConfig;
using ufil_obu_object_aggregation::UfilObuObjectAggregationParameterRos;

class ParameterRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  static std::shared_ptr<rclcpp::Node> makeNode(
    const std::vector<rclcpp::Parameter> & overrides = {})
  {
    return std::make_shared<rclcpp::Node>(
      "parameter_test", rclcpp::NodeOptions().parameter_overrides(overrides));
  }
};

TEST_F(ParameterRosTest, DeclaresAllParametersWithCoreDefaults)
{
  auto node = makeNode();
  UfilObuObjectAggregationParameterRos parameters(node->get_node_parameters_interface());

  const AggregatorConfig defaults;
  EXPECT_DOUBLE_EQ(node->get_parameter("object.length").as_double(), defaults.object_length);
  EXPECT_DOUBLE_EQ(node->get_parameter("object.width").as_double(), defaults.object_width);
  EXPECT_EQ(node->get_parameter("object.classification").as_int(), defaults.object_class);
  EXPECT_EQ(node->get_parameter("utm_zone").as_int(), -1);
}

TEST_F(ParameterRosTest, InvalidOverridesThrow)
{
  auto node = makeNode({rclcpp::Parameter("object.classification", 9)});
  EXPECT_THROW(
    UfilObuObjectAggregationParameterRos(node->get_node_parameters_interface()),
    std::invalid_argument);
}

TEST_F(ParameterRosTest, RuntimeChangesAreValidatedAndApplied)
{
  auto node = makeNode();
  UfilObuObjectAggregationParameterRos parameters(node->get_node_parameters_interface());
  int notifications = 0;
  parameters.onConfigChanged([&](const AggregatorConfig &) {++notifications;});

  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("object.length", 12.0)).successful);
  EXPECT_DOUBLE_EQ(parameters.config().object_length, 12.0);
  EXPECT_EQ(notifications, 1);

  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("object.width", 0.0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("utm_zone", 99)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("object.classification", "car")).successful);
  EXPECT_EQ(notifications, 1);
}
