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

#include "UfilPclPreprocessorParameterRos.hpp"

using ufil_pcl_preprocessor::PreprocessorConfig;
using ufil_pcl_preprocessor::UfilPclPreprocessorParameterRos;

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
  UfilPclPreprocessorParameterRos parameters(node->get_node_parameters_interface());

  const PreprocessorConfig defaults;
  EXPECT_DOUBLE_EQ(node->get_parameter("filter.voxel_size").as_double(),
    defaults.filter.voxel_size);
  EXPECT_EQ(node->get_parameter("filter.min_neighbors").as_int(), defaults.filter.min_neighbors);
  EXPECT_EQ(node->get_parameter("fov.num_bins").as_int(), defaults.fov.num_bins);
  EXPECT_DOUBLE_EQ(node->get_parameter("occlusion_grid.decay_rate").as_double(),
    defaults.occlusion_grid.decay_rate);
  EXPECT_EQ(node->get_parameter("target_frame").as_string(), "sensor_node_base");
  EXPECT_EQ(node->get_parameter("sensor_frame").as_string(), "sensor_node");
  EXPECT_FALSE(node->get_parameter("restamp_output").as_bool());
  EXPECT_EQ(node->get_parameter("static_object_file").as_string(), "");
}

TEST_F(ParameterRosTest, ReadsOverrides)
{
  auto node = makeNode(
    {rclcpp::Parameter("filter.voxel_size", 0.3), rclcpp::Parameter("fov.num_bins", 12),
      rclcpp::Parameter("target_frame", "map")});
  UfilPclPreprocessorParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_DOUBLE_EQ(parameters.config().filter.voxel_size, 0.3);
  EXPECT_EQ(parameters.config().fov.num_bins, 12);
  EXPECT_EQ(parameters.settings().target_frame, "map");
}

TEST_F(ParameterRosTest, InvalidOverridesThrow)
{
  auto node = makeNode(
    {rclcpp::Parameter("filter.z_min", 3.0), rclcpp::Parameter("filter.z_max", 1.0)});
  EXPECT_THROW(
    UfilPclPreprocessorParameterRos(node->get_node_parameters_interface()),
    std::invalid_argument);

  auto empty_frame = makeNode({rclcpp::Parameter("target_frame", "")});
  EXPECT_THROW(
    UfilPclPreprocessorParameterRos(empty_frame->get_node_parameters_interface()),
    std::invalid_argument);
}

TEST_F(ParameterRosTest, RuntimeChangesAreValidatedAndApplied)
{
  auto node = makeNode();
  UfilPclPreprocessorParameterRos parameters(node->get_node_parameters_interface());
  int notifications = 0;
  PreprocessorConfig notified;
  parameters.onConfigChanged(
    [&](const PreprocessorConfig & config) {
      ++notifications;
      notified = config;
    });

  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("filter.voxel_size", 0.25)).successful);
  EXPECT_EQ(notifications, 1);
  EXPECT_DOUBLE_EQ(notified.filter.voxel_size, 0.25);
  EXPECT_DOUBLE_EQ(parameters.config().filter.voxel_size, 0.25);

  // Invalid on its own.
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("filter.voxel_size", 0.0)).successful);
  // Invalid in combination with the current z_max.
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("filter.z_min", 10.0)).successful);
  // Valid when changed atomically together.
  EXPECT_TRUE(
    node->set_parameters_atomically(
      {rclcpp::Parameter("filter.z_min", 10.0), rclcpp::Parameter("filter.z_max", 20.0)})
    .successful);
  EXPECT_EQ(notifications, 2);
  EXPECT_DOUBLE_EQ(parameters.config().filter.voxel_size, 0.25);
  EXPECT_DOUBLE_EQ(parameters.config().filter.z_min, 10.0);

  // Wrong type and read-only parameters are rejected.
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("fov.num_bins", 1.5)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("target_frame", "map")).successful);
  EXPECT_EQ(notifications, 2);
}
