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

#include "UfilMessageFilterParameterRos.hpp"

using ufil_message_filter::UfilMessageFilterParameterRos;

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

  static std::vector<rclcpp::Parameter> area(const std::vector<double> & values)
  {
    return {
      rclcpp::Parameter("area_of_interest.enabled", true),
      rclcpp::Parameter("area_of_interest.vertices", values),
    };
  }
};

TEST_F(ParameterRosTest, DefaultDisablesFilter)
{
  auto node = makeNode({});
  UfilMessageFilterParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_TRUE(parameters.config().area_of_interest.empty());
  EXPECT_FALSE(node->get_parameter("area_of_interest.enabled").as_bool());
  EXPECT_FALSE(node->has_parameter("area_of_interest.vertices"));

  // Vertices are ignored while the area is disabled.
  auto disabled = makeNode({rclcpp::Parameter("area_of_interest.vertices",
      std::vector<double>{1.0})});
  UfilMessageFilterParameterRos(disabled->get_node_parameters_interface());
  EXPECT_FALSE(disabled->has_parameter("area_of_interest.vertices"));
}

TEST_F(ParameterRosTest, ReadsAreaOfInterest)
{
  auto node = makeNode(area({81.0, 30.0, 81.0, 46.0, 0.0, 46.0, 0.0, 30.0}));
  UfilMessageFilterParameterRos parameters(node->get_node_parameters_interface());
  const auto & vertices = parameters.config().area_of_interest;
  ASSERT_EQ(vertices.size(), 4u);
  EXPECT_DOUBLE_EQ(vertices[0].x, 81.0);
  EXPECT_DOUBLE_EQ(vertices[0].y, 30.0);
  EXPECT_DOUBLE_EQ(vertices[2].x, 0.0);
  EXPECT_DOUBLE_EQ(vertices[2].y, 46.0);
}

TEST_F(ParameterRosTest, InvalidParametersThrow)
{
  // Odd number of values.
  EXPECT_THROW(
    UfilMessageFilterParameterRos(
      makeNode(area({0.0, 0.0, 1.0, 0.0, 1.0}))->get_node_parameters_interface()),
    std::invalid_argument);
  // Too few vertices.
  EXPECT_THROW(
    UfilMessageFilterParameterRos(
      makeNode(area({0.0, 0.0, 1.0, 0.0}))->get_node_parameters_interface()),
    std::invalid_argument);
  // Self-intersecting.
  EXPECT_THROW(
    UfilMessageFilterParameterRos(
      makeNode(area({0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0}))->get_node_parameters_interface()),
    std::invalid_argument);
  // Enabled without vertices.
  EXPECT_THROW(
    UfilMessageFilterParameterRos(
      makeNode({rclcpp::Parameter("area_of_interest.enabled", true)})
      ->get_node_parameters_interface()),
    std::invalid_argument);
  // Wrong type.
  EXPECT_THROW(
    UfilMessageFilterParameterRos(
      makeNode(
        {rclcpp::Parameter("area_of_interest.enabled", true),
          rclcpp::Parameter("area_of_interest.vertices", "")})->get_node_parameters_interface()),
    rclcpp::exceptions::InvalidParameterTypeException);
}

TEST_F(ParameterRosTest, ParametersAreReadOnly)
{
  auto node = makeNode(area({0.0, 0.0, 1.0, 0.0, 1.0, 1.0}));
  UfilMessageFilterParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_FALSE(
    node->set_parameter(
      rclcpp::Parameter(
        "area_of_interest.vertices", std::vector<double>{0.0, 0.0, 2.0, 0.0, 2.0, 2.0}))
    .successful);
  EXPECT_FALSE(
    node->set_parameter(rclcpp::Parameter("area_of_interest.enabled", false)).successful);
}
