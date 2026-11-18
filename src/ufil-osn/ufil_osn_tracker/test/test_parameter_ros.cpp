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

#include "UfilOsnTrackerParameterRos.hpp"

using ufil_osn_tracker::TrackerConfig;
using ufil_osn_tracker::UfilOsnTrackerParameterRos;

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
  UfilOsnTrackerParameterRos parameters(node->get_node_parameters_interface());

  const TrackerConfig defaults;
  EXPECT_DOUBLE_EQ(node->get_parameter("association.threshold").as_double(),
    defaults.association_threshold);
  EXPECT_DOUBLE_EQ(node->get_parameter("track.history_duration").as_double(),
    defaults.history_duration);
  EXPECT_EQ(node->get_parameter("clustering.min_points").as_int(), defaults.cluster_min_points);
  EXPECT_DOUBLE_EQ(node->get_parameter("detector.position_covariance_multiplier").as_double(),
    defaults.position_covariance_multiplier);
  EXPECT_EQ(node->get_parameter("target_frame").as_string(), "map");
  EXPECT_DOUBLE_EQ(node->get_parameter("min_existence_probability").as_double(), 0.2);
  EXPECT_FALSE(node->has_parameter("time_jump_threshold"));
}

TEST_F(ParameterRosTest, InvalidOverridesThrow)
{
  auto node = makeNode(
    {rclcpp::Parameter("clustering.min_points", 100), rclcpp::Parameter(
        "clustering.max_points", 10)});
  EXPECT_THROW(
    UfilOsnTrackerParameterRos(node->get_node_parameters_interface()), std::invalid_argument);

  auto probability = makeNode({rclcpp::Parameter("min_existence_probability", 1.5)});
  EXPECT_THROW(
    UfilOsnTrackerParameterRos(probability->get_node_parameters_interface()),
    std::invalid_argument);
}

TEST_F(ParameterRosTest, RuntimeChangesAreValidatedAndApplied)
{
  auto node = makeNode();
  UfilOsnTrackerParameterRos parameters(node->get_node_parameters_interface());
  int notifications = 0;
  parameters.onConfigChanged([&](const TrackerConfig &) {++notifications;});

  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("clustering.tolerance", 0.25)).successful);
  EXPECT_DOUBLE_EQ(parameters.config().cluster_tolerance, 0.25);
  EXPECT_EQ(notifications, 1);

  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("association.threshold", -1.0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("clustering.min_points", 60000)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("target_frame", "odom")).successful);
  EXPECT_EQ(notifications, 1);
  EXPECT_DOUBLE_EQ(parameters.config().association_threshold, 4.0);
}
