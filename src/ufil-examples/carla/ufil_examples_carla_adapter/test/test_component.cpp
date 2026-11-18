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
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <class_loader/class_loader.hpp>

#include <carla_msgs/msg/carla_actor_list.hpp>
#include <derived_object_msgs/msg/object_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <ufil_msgs/msg/object_list.hpp>

namespace
{

constexpr double kPi = 3.14159265358979323846;

constexpr char kFactoryName[] =
  "rclcpp_components::NodeFactoryTemplate<ufil_examples_carla_adapter::CarlaAdapterNode>";

bool isBuiltinParameter(const std::string & name)
{
  return name == "use_sim_time" || name == "start_type_description_service" ||
         name.rfind("qos_overrides.", 0) == 0;
}

carla_msgs::msg::CarlaActorInfo makeActor(
  std::uint32_t id, const std::string & type, const std::string & role_name)
{
  carla_msgs::msg::CarlaActorInfo actor;
  actor.id = id;
  actor.type = type;
  actor.rolename = role_name;
  return actor;
}

derived_object_msgs::msg::Object makeObject(std::uint32_t id, double x, double y)
{
  derived_object_msgs::msg::Object object;
  object.id = id;
  object.pose.position.x = x;
  object.pose.position.y = y;
  object.pose.orientation.w = 1.0;
  object.shape.dimensions = {4.0, 2.0, 1.5};
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

  /// Runs the adapter with the given parameters and returns the received object lists.
  std::vector<ufil_msgs::msg::ObjectList> run(
    const std::vector<rclcpp::Parameter> & parameters, bool send_actor_list)
  {
    auto wrapper = create(rclcpp::NodeOptions().parameter_overrides(parameters));
    auto io = std::make_shared<rclcpp::Node>("component_test_io");

    // map -> world: 100 m in x, rotated by 90 degrees.
    tf2_ros::StaticTransformBroadcaster broadcaster(io);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.child_frame_id = "map";
    transform.transform.translation.x = 100.0;
    transform.transform.rotation.z = std::sqrt(0.5);
    transform.transform.rotation.w = std::sqrt(0.5);
    broadcaster.sendTransform(transform);

    std::vector<ufil_msgs::msg::ObjectList> received;
    auto subscription = io->create_subscription<ufil_msgs::msg::ObjectList>(
      "object_list", 10,
      [&received](const ufil_msgs::msg::ObjectList & msg) {received.push_back(msg);});
    auto actor_publisher = io->create_publisher<carla_msgs::msg::CarlaActorList>("actor_list", 10);
    auto object_publisher =
      io->create_publisher<derived_object_msgs::msg::ObjectArray>("objects", 10);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(wrapper.get_node_base_interface());
    executor.add_node(io);
    const auto spinFor = [&executor](
      const std::function<bool()> & done, std::chrono::milliseconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
          executor.spin_some(std::chrono::milliseconds(10));
        }
      };
    spinFor(
      [&] {
        return actor_publisher->get_subscription_count() > 0 &&
               object_publisher->get_subscription_count() > 0 &&
               subscription->get_publisher_count() > 0;
      }, std::chrono::seconds(5));

    if (send_actor_list) {
      carla_msgs::msg::CarlaActorList actors;
      actors.actors = {
        makeActor(10, "vehicle.tesla.model3", "vehicle_1"),
        makeActor(11, "vehicle.carlamotors.european_hgv", "vehicle_2"),
        makeActor(12, "walker.pedestrian.0001", "")};
      actor_publisher->publish(actors);
      spinFor([] {return false;}, std::chrono::milliseconds(200));
    }

    derived_object_msgs::msg::ObjectArray objects;
    objects.header.frame_id = "map";
    objects.header.stamp.sec = 5;
    objects.objects = {makeObject(10, 1.0, 2.0), makeObject(11, 3.0, 4.0), makeObject(12, 0, 0)};
    // Repeat until the transform listener has the static transform.
    for (int i = 0; i < 20 && received.empty(); ++i) {
      object_publisher->publish(objects);
      spinFor([&received] {return !received.empty();}, std::chrono::milliseconds(100));
    }
    return received;
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
  EXPECT_EQ(std::string(node->get_name()), "carla_adapter");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_EQ(node->get_parameter("target_frame").as_string(), "map");
  EXPECT_FALSE(node->get_parameter("cooperative_vehicles.enabled").as_bool());

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
  EXPECT_EQ(node->count_subscribers("actor_list"), 1u);
  EXPECT_EQ(node->count_publishers("object_list"), 1u);
}

TEST_F(ComponentTest, ConvertsAllObjects)
{
  const auto received = run({}, true);
  ASSERT_FALSE(received.empty());
  const auto & list = received.front();
  EXPECT_EQ(list.header.frame_id, "map");
  EXPECT_EQ(list.header.stamp.sec, 5);
  ASSERT_EQ(list.objects.size(), 3u);
  EXPECT_EQ(list.objects[0].id, 10u);
  EXPECT_DOUBLE_EQ(list.objects[0].state.state.x, 1.0);
  EXPECT_DOUBLE_EQ(list.objects[0].state.state.y, 2.0);
  EXPECT_FLOAT_EQ(
    list.objects[0].classification.classification[ufil_msgs::msg::Classification::CAR], 1.0F);
  EXPECT_EQ(list.objects[1].axles.size(), 3u);
  EXPECT_FLOAT_EQ(
    list.objects[2].classification.classification[ufil_msgs::msg::Classification::OTHER], 1.0F);
}

TEST_F(ComponentTest, FiltersAndTransformsObjects)
{
  const auto received = run(
    {rclcpp::Parameter("target_frame", "world"),
      rclcpp::Parameter("cooperative_vehicles.enabled", true),
      rclcpp::Parameter("cooperative_vehicles.ids", std::vector<std::int64_t>{2})}, true);
  ASSERT_FALSE(received.empty());
  const auto & list = received.front();
  EXPECT_EQ(list.header.frame_id, "world");
  ASSERT_EQ(list.objects.size(), 1u);
  EXPECT_EQ(list.objects[0].id, 11u);
  EXPECT_NEAR(list.objects[0].state.state.x, 100.0 - 4.0, 1e-6);
  EXPECT_NEAR(list.objects[0].state.state.y, 3.0, 1e-6);
  EXPECT_NEAR(list.objects[0].state.state.yaw, kPi / 2.0, 1e-6);
}

TEST_F(ComponentTest, WaitsForActorList)
{
  EXPECT_TRUE(run({}, false).empty());
}

TEST_F(ComponentTest, DropsObjectsWithoutTransform)
{
  EXPECT_TRUE(run({rclcpp::Parameter("target_frame", "unknown_frame")}, true).empty());
}

TEST_F(ComponentTest, InvalidParametersFailConstruction)
{
  EXPECT_THROW(
    create(
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("cooperative_vehicles.enabled", true)})),
    std::invalid_argument);
}
