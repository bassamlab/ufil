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
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "UfilExamplesCarlaAdapterParameterRos.hpp"

using ufil_examples_carla_adapter::UfilExamplesCarlaAdapterParameterRos;

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

  static std::vector<rclcpp::Parameter> cooperative(const std::vector<std::int64_t> & ids)
  {
    return {
      rclcpp::Parameter("cooperative_vehicles.enabled", true),
      rclcpp::Parameter("cooperative_vehicles.ids", ids),
    };
  }
};

TEST_F(ParameterRosTest, Defaults)
{
  auto node = makeNode({});
  UfilExamplesCarlaAdapterParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_EQ(parameters.targetFrame(), "map");
  EXPECT_FALSE(parameters.config().filter_cooperative_vehicles);
  EXPECT_TRUE(parameters.config().cooperative_vehicle_ids.empty());
  EXPECT_FALSE(node->has_parameter("cooperative_vehicles.ids"));

  // IDs are ignored while the filter is disabled.
  auto disabled = makeNode({rclcpp::Parameter("cooperative_vehicles.ids",
      std::vector<std::int64_t>{-1})});
  UfilExamplesCarlaAdapterParameterRos(disabled->get_node_parameters_interface());
  EXPECT_FALSE(disabled->has_parameter("cooperative_vehicles.ids"));
}

TEST_F(ParameterRosTest, ReadsParameters)
{
  auto overrides = cooperative({4, 9});
  overrides.emplace_back("target_frame", "utm_32n");
  auto node = makeNode(overrides);
  UfilExamplesCarlaAdapterParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_EQ(parameters.targetFrame(), "utm_32n");
  EXPECT_TRUE(parameters.config().filter_cooperative_vehicles);
  EXPECT_EQ(parameters.config().cooperative_vehicle_ids, (std::vector<std::int64_t>{4, 9}));
}

TEST_F(ParameterRosTest, InvalidParametersThrow)
{
  const auto construct = [](const std::vector<rclcpp::Parameter> & overrides) {
      UfilExamplesCarlaAdapterParameterRos(makeNode(overrides)->get_node_parameters_interface());
    };
  EXPECT_THROW(construct({rclcpp::Parameter("target_frame", "")}), std::invalid_argument);
  EXPECT_THROW(
    construct({rclcpp::Parameter("cooperative_vehicles.enabled", true)}), std::invalid_argument);
  EXPECT_THROW(construct(cooperative({1, -1})), std::invalid_argument);
  EXPECT_THROW(construct(cooperative({2, 2})), std::invalid_argument);
  EXPECT_THROW(
    construct({rclcpp::Parameter("target_frame", 1)}),
    rclcpp::exceptions::InvalidParameterTypeException);
}

TEST_F(ParameterRosTest, ParametersAreReadOnly)
{
  auto node = makeNode(cooperative({1}));
  UfilExamplesCarlaAdapterParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("target_frame", "odom")).successful);
  EXPECT_FALSE(
    node->set_parameter(
      rclcpp::Parameter("cooperative_vehicles.ids", std::vector<std::int64_t>{2})).successful);
  EXPECT_FALSE(
    node->set_parameter(rclcpp::Parameter("cooperative_vehicles.enabled", false)).successful);
}
