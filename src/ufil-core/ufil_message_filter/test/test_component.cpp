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
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <class_loader/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <ufil_msgs/msg/object_list.hpp>

namespace
{

constexpr char kFactoryName[] =
  "rclcpp_components::NodeFactoryTemplate<ufil_message_filter::ObjectListPositionFilterNode>";

bool isBuiltinParameter(const std::string & name)
{
  return name == "use_sim_time" || name == "start_type_description_service" ||
         name.rfind("qos_overrides.", 0) == 0;
}

ufil_msgs::msg::Object makeObject(std::uint32_t id, double x, double y)
{
  ufil_msgs::msg::Object object;
  object.id = id;
  object.state.state.x = x;
  object.state.state.y = y;
  return object;
}

}  // namespace

class ComponentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  rclcpp_components::NodeInstanceWrapper create(const rclcpp::NodeOptions & options)
  {
    auto factory = loader_.createInstance<rclcpp_components::NodeFactory>(kFactoryName);
    return factory->create_node_instance(options);
  }

  class_loader::ClassLoader loader_{COMPONENT_LIBRARY};
};

TEST_F(ComponentTest, IsRegistered)
{
  const auto classes = loader_.getAvailableClasses<rclcpp_components::NodeFactory>();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kFactoryName), classes.end());
}

TEST_F(ComponentTest, LoadsWithDefaultConfiguration)
{
  auto wrapper = create(
    rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", DEFAULT_CONFIG, "-p", "use_sim_time:=true"}));
  // The registered class derives from rclcpp::Node only.
  auto node = std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(std::string(node->get_name()), "object_list_position_filter");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_FALSE(node->get_parameter("area_of_interest.enabled").as_bool());

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

  EXPECT_EQ(node->count_subscribers("objects"), 1u);
  EXPECT_EQ(node->count_publishers("objects_filtered"), 1u);
}

TEST_F(ComponentTest, ForwardsObjectsInsideArea)
{
  auto wrapper = create(
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("area_of_interest.enabled", true),
        rclcpp::Parameter(
          "area_of_interest.vertices",
          std::vector<double>{0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0})}));
  auto filter = wrapper.get_node_base_interface();

  auto io = std::make_shared<rclcpp::Node>("component_test_io");
  std::vector<ufil_msgs::msg::ObjectList> received;
  auto subscription = io->create_subscription<ufil_msgs::msg::ObjectList>(
    "objects_filtered", 10,
    [&received](const ufil_msgs::msg::ObjectList & msg) {received.push_back(msg);});
  auto publisher = io->create_publisher<ufil_msgs::msg::ObjectList>("objects", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(filter);
  executor.add_node(io);
  const auto spinUntil = [&executor](const std::function<bool()> & done) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (!done() && std::chrono::steady_clock::now() < deadline) {
        executor.spin_some(std::chrono::milliseconds(10));
      }
    };
  spinUntil(
    [&] {
      return publisher->get_subscription_count() > 0 && subscription->get_publisher_count() > 0;
    });

  // Lists without an object inside the area are not published.
  ufil_msgs::msg::ObjectList outside;
  outside.header.frame_id = "map";
  outside.objects = {makeObject(1, 20.0, 5.0)};
  publisher->publish(outside);

  ufil_msgs::msg::ObjectList mixed;
  mixed.header.frame_id = "map";
  mixed.header.stamp.sec = 7;
  mixed.objects = {makeObject(2, 5.0, 5.0), makeObject(3, -1.0, 5.0), makeObject(4, 9.0, 1.0)};
  publisher->publish(mixed);

  spinUntil([&received] {return !received.empty();});
  // Give a wrongly published empty list the chance to arrive as well.
  const auto settle = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (std::chrono::steady_clock::now() < settle) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].header, mixed.header);
  ASSERT_EQ(received[0].objects.size(), 2u);
  EXPECT_EQ(received[0].objects[0].id, 2u);
  EXPECT_EQ(received[0].objects[1].id, 4u);
}

TEST_F(ComponentTest, InvalidParametersFailConstruction)
{
  EXPECT_THROW(
    create(
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("area_of_interest.enabled", true),
          rclcpp::Parameter("area_of_interest.vertices", std::vector<double>{0.0, 0.0, 1.0})})),
    std::invalid_argument);
}
