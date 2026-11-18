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

#include "SimulatorParameterRos.hpp"

using ufil_examples_pedestrian_simulator::resolveScenarioFile;
using ufil_examples_pedestrian_simulator::SimulatorParameterRos;

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

TEST_F(ParameterRosTest, DeclaresDefaults)
{
  auto node = makeNode({});
  SimulatorParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_EQ(parameters.settings().scenario_file, "crossing.yaml");
  EXPECT_DOUBLE_EQ(parameters.settings().timestep, 0.1);
  EXPECT_EQ(parameters.settings().frame_id, "map");
  EXPECT_FALSE(node->describe_parameter("timestep").description.empty());
  EXPECT_TRUE(node->describe_parameter("scenario_file").read_only);
}

TEST_F(ParameterRosTest, ReadsOverrides)
{
  auto node = makeNode(
  {
    rclcpp::Parameter("scenario_file", "/tmp/scenario.yaml"),
    rclcpp::Parameter("timestep", 0.05),
    rclcpp::Parameter("frame_id", "odom"),
  });
  SimulatorParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_EQ(parameters.settings().scenario_file, "/tmp/scenario.yaml");
  EXPECT_DOUBLE_EQ(parameters.settings().timestep, 0.05);
  EXPECT_EQ(parameters.settings().frame_id, "odom");
}

TEST_F(ParameterRosTest, RejectsInvalidValues)
{
  for (const auto & parameter : {
    rclcpp::Parameter("scenario_file", ""),
    rclcpp::Parameter("timestep", 0.0),
    rclcpp::Parameter("timestep", -0.1),
    rclcpp::Parameter("frame_id", "")})
  {
    SCOPED_TRACE(parameter.get_name());
    auto node = makeNode({parameter});
    EXPECT_THROW(
      SimulatorParameterRos(node->get_node_parameters_interface()),
      std::invalid_argument);
  }
}

TEST_F(ParameterRosTest, RejectsWrongTypes)
{
  auto node = makeNode({rclcpp::Parameter("timestep", 100)});
  EXPECT_ANY_THROW(
    SimulatorParameterRos(node->get_node_parameters_interface()));
}

TEST(ResolveScenarioFileTest, ResolvesFileNamesInTheScenarioDirectory)
{
  EXPECT_EQ(resolveScenarioFile("crossing.yaml", "/share/config"), "/share/config/crossing.yaml");
  EXPECT_EQ(resolveScenarioFile("/data/my.yaml", "/share/config"), "/data/my.yaml");
}
