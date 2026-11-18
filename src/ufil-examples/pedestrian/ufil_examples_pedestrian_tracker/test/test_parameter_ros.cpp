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

#include "TrackerParameterRos.hpp"

using ufil_examples_pedestrian_tracker::TrackerConfig;
using ufil_examples_pedestrian_tracker::TrackerParameterRos;

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

TEST_F(ParameterRosTest, DeclaresCoreDefaults)
{
  auto node = makeNode({});
  TrackerParameterRos parameters(node->get_node_parameters_interface());
  const TrackerConfig defaults;
  EXPECT_DOUBLE_EQ(parameters.config().process_noise, defaults.process_noise);
  EXPECT_DOUBLE_EQ(parameters.config().track_timeout, defaults.track_timeout);
  EXPECT_DOUBLE_EQ(parameters.config().max_time_gap, defaults.max_time_gap);
  EXPECT_TRUE(node->describe_parameter("track_timeout").read_only);
  EXPECT_FALSE(node->describe_parameter("process_noise").description.empty());
}

TEST_F(ParameterRosTest, ReadsOverrides)
{
  auto node = makeNode(
  {
    rclcpp::Parameter("process_noise", 5.0),
    rclcpp::Parameter("track_timeout", 2.0),
    rclcpp::Parameter("max_time_gap", 3.0),
  });
  TrackerParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_DOUBLE_EQ(parameters.config().process_noise, 5.0);
  EXPECT_DOUBLE_EQ(parameters.config().track_timeout, 2.0);
  EXPECT_DOUBLE_EQ(parameters.config().max_time_gap, 3.0);
}

TEST_F(ParameterRosTest, RejectsInvalidValues)
{
  for (const auto & parameter : {
    rclcpp::Parameter("process_noise", 0.0),
    rclcpp::Parameter("track_timeout", -1.0),
    rclcpp::Parameter("max_time_gap", 0.0)})
  {
    SCOPED_TRACE(parameter.get_name());
    auto node = makeNode({parameter});
    EXPECT_THROW(
      TrackerParameterRos(node->get_node_parameters_interface()),
      std::invalid_argument);
  }
}
