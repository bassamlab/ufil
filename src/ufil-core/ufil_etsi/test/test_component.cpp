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

#include <class_loader/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>

namespace
{

struct ComponentInfo
{
  std::string plugin;
  std::string node_name;
  std::string config;
  std::vector<std::string> subscriptions;
  std::vector<std::string> publications;
  /// A parameter override that makes the construction fail.
  rclcpp::Parameter invalid_parameter;
};

std::vector<ComponentInfo> components()
{
  return {
    {"ufil_etsi::VehicleCamGeneratorNode", "vehicle_cam_generator_node",
      "vehicle_cam_generator.yaml", {"/object"}, {"/cam", "/diagnostics"},
      rclcpp::Parameter("target_frame", "map")},
    {"ufil_etsi::VehicleCamReceiverNode", "vehicle_cam_receiver_node",
      "vehicle_cam_receiver.yaml", {"/cam"}, {"/object_list", "/diagnostics"},
      rclcpp::Parameter("target_frame", "")},
    {"ufil_etsi::RsuCamGeneratorNode", "rsu_cam_generator_node", "rsu_cam_generator.yaml", {},
      {"/cam"}, rclcpp::Parameter("position.latitude", 100.0)},
  };
}

std::string factoryName(const ComponentInfo & component)
{
  return "rclcpp_components::NodeFactoryTemplate<" + component.plugin + ">";
}

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

TEST_F(ComponentTest, AllComponentsAreRegistered)
{
  const auto classes = loader_.getAvailableClasses<rclcpp_components::NodeFactory>();
  for (const auto & component : components()) {
    EXPECT_NE(std::find(classes.begin(), classes.end(), factoryName(component)), classes.end())
      << component.plugin;
  }
}

TEST_F(ComponentTest, LoadWithDefaultConfiguration)
{
  for (const auto & component : components()) {
    SCOPED_TRACE(component.plugin);
    auto factory =
      loader_.createInstance<rclcpp_components::NodeFactory>(factoryName(component));
    const std::string config = std::string(CONFIG_DIR) + "/" + component.config;
    auto options = rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", config, "-p", "use_sim_time:=true"});
    auto wrapper = factory->create_node_instance(options);
    // The registered classes derive from rclcpp::Node only.
    auto node = std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(std::string(node->get_name()), component.node_name);
    EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());

    // The default configuration and the declared parameters match exactly.
    const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
    for (const auto & [name, value] : overrides) {
      if (!isBuiltinParameter(name)) {
        EXPECT_TRUE(node->has_parameter(name)) << name << " in the config is not declared";
      }
    }
    for (const auto & name : node->list_parameters({}, 0).names) {
      if (!isBuiltinParameter(name)) {
        EXPECT_TRUE(overrides.count(name)) << name << " is declared but missing in the config";
      }
    }

    for (const auto & topic : component.subscriptions) {
      EXPECT_EQ(node->count_subscribers(topic), 1u) << topic;
    }
    for (const auto & topic : component.publications) {
      EXPECT_GE(node->count_publishers(topic), 1u) << topic;
    }
  }
}

TEST_F(ComponentTest, InvalidParametersFailConstruction)
{
  for (const auto & component : components()) {
    SCOPED_TRACE(component.plugin);
    auto factory =
      loader_.createInstance<rclcpp_components::NodeFactory>(factoryName(component));
    auto options = rclcpp::NodeOptions().parameter_overrides({component.invalid_parameter});
    EXPECT_THROW(factory->create_node_instance(options), std::invalid_argument);
  }
}
