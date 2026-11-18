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

#include "UfilCentralFusionParameterRos.hpp"

using ufil_central_fusion::FusionConfig;
using ufil_central_fusion::UfilCentralFusionParameterRos;

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

  static std::vector<rclcpp::Parameter> twoSensors()
  {
    return {
      rclcpp::Parameter("sensor_names", std::vector<std::string>{"osn", "cam"}),
      rclcpp::Parameter("osn.topic", "/osn/object_list"),
      rclcpp::Parameter("osn.fov_topic", "/osn/sensor_fov"),
      rclcpp::Parameter("osn.trust.pedestrian", 0.6),
      rclcpp::Parameter("cam.topic", "/rsu/cam/object_list"),
      rclcpp::Parameter("cam.is_cam", true),
    };
  }
};

TEST_F(ParameterRosTest, DeclaresSensorsAndCoreDefaults)
{
  auto node = makeNode(twoSensors());
  UfilCentralFusionParameterRos parameters(node->get_node_parameters_interface());

  const FusionConfig defaults;
  EXPECT_DOUBLE_EQ(parameters.config().association_threshold, defaults.association_threshold);
  EXPECT_DOUBLE_EQ(node->get_parameter("fusion.period").as_double(), 0.02);
  EXPECT_EQ(parameters.settings().target_frame, "map");

  ASSERT_EQ(parameters.sensors().size(), 2u);
  EXPECT_EQ(parameters.sensors()[0].name, "osn");
  EXPECT_FALSE(parameters.sensors()[0].is_cam);
  EXPECT_DOUBLE_EQ(parameters.sensors()[0].trust.pedestrian, 0.6);
  EXPECT_DOUBLE_EQ(parameters.sensors()[0].trust.car, 0.8);
  EXPECT_TRUE(parameters.sensors()[1].is_cam);
  EXPECT_EQ(parameters.topics()[0].fov, "/osn/sensor_fov");
  EXPECT_EQ(parameters.topics()[1].fov, "");
  EXPECT_TRUE(node->has_parameter("cam.trust.existence"));
}

TEST_F(ParameterRosTest, InvalidParametersThrow)
{
  // No sensors.
  EXPECT_THROW(
    UfilCentralFusionParameterRos(makeNode({})->get_node_parameters_interface()),
    std::invalid_argument);

  // Sensor without topic.
  EXPECT_THROW(
    UfilCentralFusionParameterRos(
      makeNode({rclcpp::Parameter("sensor_names", std::vector<std::string>{"osn"})})
      ->get_node_parameters_interface()),
    std::invalid_argument);

  // Invalid trust and fusion values.
  auto overrides = twoSensors();
  overrides.push_back(rclcpp::Parameter("cam.trust.car", 2.0));
  EXPECT_THROW(
    UfilCentralFusionParameterRos(makeNode(overrides)->get_node_parameters_interface()),
    std::invalid_argument);
  overrides = twoSensors();
  overrides.push_back(rclcpp::Parameter("existence.min_weight", 0.9));
  EXPECT_THROW(
    UfilCentralFusionParameterRos(makeNode(overrides)->get_node_parameters_interface()),
    std::invalid_argument);
}

TEST_F(ParameterRosTest, ParametersAreReadOnly)
{
  auto node = makeNode(twoSensors());
  UfilCentralFusionParameterRos parameters(node->get_node_parameters_interface());
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("association.threshold", 5.0)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("osn.topic", "/other")).successful);
}
