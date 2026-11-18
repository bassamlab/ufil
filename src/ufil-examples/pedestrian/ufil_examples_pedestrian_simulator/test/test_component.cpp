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

#include <class_loader/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>

namespace
{

constexpr char kFactoryName[] =
  "rclcpp_components::NodeFactoryTemplate<"
  "ufil_examples_pedestrian_simulator::PedestrianSimulatorNode>";

constexpr char kScenario[] = SCENARIO_DIRECTORY "/crossing.yaml";

bool isBuiltinParameter(const std::string & name)
{
  return name == "use_sim_time" || name == "start_type_description_service" ||
         name.rfind("qos_overrides.", 0) == 0;
}

}  // namespace

class ComponentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  class_loader::ClassLoader loader_{COMPONENT_LIBRARY};
};

TEST_F(ComponentTest, IsRegistered)
{
  const auto classes = loader_.getAvailableClasses<rclcpp_components::NodeFactory>();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kFactoryName), classes.end());
}

TEST_F(ComponentTest, LoadsWithDefaultConfiguration)
{
  auto factory = loader_.createInstance<rclcpp_components::NodeFactory>(kFactoryName);
  // The package is not installed while testing, so the scenario is given as absolute path.
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "--params-file", DEFAULT_CONFIG, "-p", "use_sim_time:=true", "-p",
      std::string("scenario_file:=") + kScenario});
  auto wrapper = factory->create_node_instance(options);
  // The registered class derives from rclcpp::Node only.
  auto node = std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(std::string(node->get_name()), "pedestrian_simulator");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_DOUBLE_EQ(node->get_parameter("timestep").as_double(), 0.1);
  EXPECT_EQ(node->get_parameter("frame_id").as_string(), "map");

  // default.yaml and the declared parameters match exactly.
  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, value] : overrides) {
    if (!isBuiltinParameter(name)) {
      EXPECT_TRUE(node->has_parameter(name)) << name << " in default.yaml is not declared";
    }
  }
  const auto declared = node->list_parameters({}, 0).names;
  for (const auto & name : declared) {
    if (!isBuiltinParameter(name)) {
      EXPECT_TRUE(overrides.count(name)) << name << " is declared but missing in default.yaml";
    }
  }

  EXPECT_EQ(node->count_publishers("measured_objects"), 1u);
  EXPECT_EQ(node->count_publishers("reference_objects"), 1u);
}

TEST_F(ComponentTest, InvalidParametersFailConstruction)
{
  auto factory = loader_.createInstance<rclcpp_components::NodeFactory>(kFactoryName);
  auto options = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("scenario_file", kScenario), rclcpp::Parameter("timestep", -1.0)});
  EXPECT_THROW(factory->create_node_instance(options), std::invalid_argument);
}

TEST_F(ComponentTest, MissingScenarioFailsConstruction)
{
  auto factory = loader_.createInstance<rclcpp_components::NodeFactory>(kFactoryName);
  auto options = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("scenario_file", "/nonexistent/scenario.yaml")});
  EXPECT_THROW(factory->create_node_instance(options), std::runtime_error);
}
