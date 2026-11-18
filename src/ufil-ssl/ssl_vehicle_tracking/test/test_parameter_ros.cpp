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

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "SslVehicleTrackingParameterRos.hpp"

using ssl_vehicle_tracking::SslVehicleTrackingParameterRos;
using ssl_vehicle_tracking::TrackerConfig;

namespace
{

std::string templatePath(const std::string & name)
{
  return std::string(TEMPLATE_DIR) + "/" + name;
}

}  // namespace

class ParameterRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  /// The installed default template is not available in the build tree, so every node gets an
  /// explicit template unless the test sets its own.
  static std::shared_ptr<rclcpp::Node> makeNode(std::vector<rclcpp::Parameter> overrides)
  {
    const bool has_templates = std::any_of(
      overrides.begin(), overrides.end(),
      [](const rclcpp::Parameter & p) {return p.get_name() == "vehicle_templates";});
    if (!has_templates) {
      overrides.emplace_back(
        "vehicle_templates", std::vector<std::string>{templatePath("muCar_scaled_lab.json")});
    }
    return std::make_shared<rclcpp::Node>(
      "parameter_test", rclcpp::NodeOptions().parameter_overrides(overrides));
  }
};

TEST_F(ParameterRosTest, Defaults)
{
  auto node = makeNode({});
  SslVehicleTrackingParameterRos parameters(node->get_node_parameters_interface());

  const TrackerConfig defaults;
  EXPECT_DOUBLE_EQ(parameters.tracker().search_width, defaults.search_width);
  EXPECT_EQ(parameters.tracker().considered_rotation_pairs, defaults.considered_rotation_pairs);
  EXPECT_DOUBLE_EQ(parameters.tracker().dropping_threshold, defaults.dropping_threshold);
  EXPECT_DOUBLE_EQ(parameters.settings().publishing_threshold, 0.0);
  ASSERT_EQ(parameters.templates().size(), 1u);
  EXPECT_EQ(parameters.templates()[0].description().name, "muCar_scaled_lab");
  EXPECT_EQ(parameters.templatePaths()[0], templatePath("muCar_scaled_lab.json"));

  EXPECT_TRUE(node->describe_parameter("search_width").read_only);
  EXPECT_FALSE(node->describe_parameter("vehicle_templates").description.empty());
}

TEST_F(ParameterRosTest, ReadsOverrides)
{
  auto node = makeNode(
  {
    rclcpp::Parameter(
      "vehicle_templates",
      std::vector<std::string>{templatePath("car_carla.json"), templatePath("truck_carla.json")}),
    rclcpp::Parameter("search_width", 0.5),
    rclcpp::Parameter("considered_rotation_pairs", 3),
    rclcpp::Parameter("dropping_threshold", 0.1),
    rclcpp::Parameter("publishing_threshold", 0.5),
  });
  SslVehicleTrackingParameterRos parameters(node->get_node_parameters_interface());

  ASSERT_EQ(parameters.templates().size(), 2u);
  EXPECT_EQ(parameters.templates()[1].wheelCount(), 6u);
  EXPECT_DOUBLE_EQ(parameters.tracker().search_width, 0.5);
  EXPECT_EQ(parameters.tracker().considered_rotation_pairs, 3u);
  EXPECT_DOUBLE_EQ(parameters.tracker().dropping_threshold, 0.1);
  EXPECT_DOUBLE_EQ(parameters.settings().publishing_threshold, 0.5);
}

TEST_F(ParameterRosTest, ResolvesTemplateNames)
{
  EXPECT_EQ(
    SslVehicleTrackingParameterRos::resolveTemplatePath("/some/dir/car.json"),
    "/some/dir/car.json");
  EXPECT_EQ(
    SslVehicleTrackingParameterRos::resolveTemplatePath("relative/car.json"), "relative/car.json");
}

TEST_F(ParameterRosTest, InvalidValuesThrow)
{
  const std::vector<rclcpp::Parameter> invalid{
    rclcpp::Parameter("vehicle_templates", std::vector<std::string>{}),
    rclcpp::Parameter("vehicle_templates", std::vector<std::string>{"/nonexistent.json"}),
    rclcpp::Parameter("search_width", 0.0),
    rclcpp::Parameter("considered_rotation_pairs", 0),
    rclcpp::Parameter("dropping_threshold", -0.1),
    rclcpp::Parameter("dropping_threshold", 1.1),
    rclcpp::Parameter("publishing_threshold", -0.1),
    rclcpp::Parameter("publishing_threshold", 1.1),
  };
  for (const auto & parameter : invalid) {
    auto node = makeNode({parameter});
    EXPECT_THROW(
      SslVehicleTrackingParameterRos(node->get_node_parameters_interface()),
      std::invalid_argument) << parameter.get_name() << " = " << parameter.value_to_string();
  }
}

TEST_F(ParameterRosTest, WrongTypeThrows)
{
  auto node = makeNode({rclcpp::Parameter("considered_rotation_pairs", 2.0)});
  EXPECT_THROW(
    SslVehicleTrackingParameterRos(node->get_node_parameters_interface()),
    rclcpp::exceptions::InvalidParameterTypeException);
}
