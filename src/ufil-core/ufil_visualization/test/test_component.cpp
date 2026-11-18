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
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <class_loader/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{

using MarkerArray = visualization_msgs::msg::MarkerArray;

constexpr char kLaneletMapFactory[] =
  "rclcpp_components::NodeFactoryTemplate<ufil_visualization::LaneletMapMarkerArrayNode>";
constexpr char kObjectListFactory[] =
  "rclcpp_components::NodeFactoryTemplate<ufil_visualization::ObjectListMarkerArrayNode>";

bool isBuiltinParameter(const std::string & name)
{
  return name == "use_sim_time" || name == "start_type_description_service" ||
         name.rfind("qos_overrides.", 0) == 0;
}

/**
 * @brief The parameters of the configuration file and the declared parameters match exactly.
 *
 * Declared parameters in may_be_missing need not be in the file, e.g. empty arrays, which ROS 2
 * parameter files cannot express.
 */
void expectConfigMatchesDeclaration(
  rclcpp::Node & node, const std::set<std::string> & may_be_missing = {})
{
  const auto & overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, value] : overrides) {
    if (!isBuiltinParameter(name)) {
      EXPECT_TRUE(node.has_parameter(name)) << name << " in the config file is not declared";
    }
  }
  for (const auto & name : node.list_parameters({}, 0).names) {
    if (!isBuiltinParameter(name) && !may_be_missing.count(name)) {
      EXPECT_TRUE(overrides.count(name)) << name << " is declared but missing in the config file";
    }
  }
}

/// Spins the nodes until the predicate holds or the timeout expires.
template<typename Predicate>
bool spinUntil(
  const std::vector<rclcpp::Node::SharedPtr> & nodes, Predicate predicate,
  std::chrono::seconds timeout = std::chrono::seconds(5))
{
  rclcpp::executors::SingleThreadedExecutor executor;
  for (const auto & node : nodes) {
    executor.add_node(node);
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  return predicate();
}

}  // namespace

class ComponentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  rclcpp_components::NodeInstanceWrapper create(
    const char * factory_name, const rclcpp::NodeOptions & options)
  {
    auto factory = loader_.createInstance<rclcpp_components::NodeFactory>(factory_name);
    return factory->create_node_instance(options);
  }

  static rclcpp::Node::SharedPtr node(const rclcpp_components::NodeInstanceWrapper & wrapper)
  {
    // The registered classes derive from rclcpp::Node only.
    return std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());
  }

  class_loader::ClassLoader loader_{COMPONENT_LIBRARY};
};

TEST_F(ComponentTest, AreRegistered)
{
  const auto classes = loader_.getAvailableClasses<rclcpp_components::NodeFactory>();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kLaneletMapFactory), classes.end());
  EXPECT_NE(std::find(classes.begin(), classes.end(), kObjectListFactory), classes.end());
}

TEST_F(ComponentTest, ObjectListLoadsWithDefaultConfiguration)
{
  auto wrapper = create(
    kObjectListFactory, rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", OBJECT_LIST_CONFIG, "-p", "use_sim_time:=true"}));
  auto object_list = node(wrapper);
  ASSERT_NE(object_list, nullptr);
  EXPECT_EQ(std::string(object_list->get_name()), "object_list_marker_array");
  EXPECT_TRUE(object_list->get_parameter("use_sim_time").as_bool());
  expectConfigMatchesDeclaration(*object_list, {"sources"});
  EXPECT_EQ(object_list->count_subscribers("object_list"), 1u);
  EXPECT_EQ(object_list->count_publishers("object_markers"), 1u);
}

TEST_F(ComponentTest, ObjectListLoadsUfilSources)
{
  auto wrapper = create(
    kObjectListFactory, rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", OBJECT_LIST_CONFIG, "--params-file", UFIL_SOURCES_CONFIG}));
  auto object_list = node(wrapper);
  expectConfigMatchesDeclaration(*object_list);
  for (const auto & source : {"osn", "obu", "rsu/cam", "rsu", "ssl"}) {
    const std::string prefix = std::string("/") + source;
    EXPECT_EQ(object_list->count_subscribers(prefix + "/object_list"), 1u) << source;
    EXPECT_EQ(object_list->count_publishers(prefix + "/object_markers"), 1u) << source;
  }
  EXPECT_EQ(object_list->count_subscribers("object_list"), 0u);
}

TEST_F(ComponentTest, ObjectListPublishesEverySourceOnItsOwnTopic)
{
  auto wrapper = create(
    kObjectListFactory, rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("sources", std::vector<std::string>{"first", "second"}),
    rclcpp::Parameter("first.color", std::vector<double>{1.0, 0.0, 0.0, 1.0}),
    rclcpp::Parameter("second.input_topic", "/other/objects"),
    rclcpp::Parameter("second.output_topic", "/other/markers"),
    rclcpp::Parameter("second.color", std::vector<double>{0.0, 0.0, 1.0, 1.0}),
  }));
  auto test_node = std::make_shared<rclcpp::Node>("object_list_test");
  MarkerArray::SharedPtr first;
  MarkerArray::SharedPtr second;
  auto first_subscription = test_node->create_subscription<MarkerArray>(
    "first/object_markers", 10, [&first](MarkerArray::SharedPtr msg) {first = msg;});
  auto second_subscription = test_node->create_subscription<MarkerArray>(
    "/other/markers", 10, [&second](MarkerArray::SharedPtr msg) {second = msg;});
  auto publisher = test_node->create_publisher<ufil_msgs::msg::ObjectList>(
    "/other/objects", 10);

  ufil_msgs::msg::ObjectList objects;
  objects.header.frame_id = "map";
  objects.objects.resize(1);
  ASSERT_TRUE(
    spinUntil(
      {node(wrapper), test_node}, [&] {
        if (!second) {
          publisher->publish(objects);
        }
        return second != nullptr;
      }));

  EXPECT_EQ(first, nullptr);
  ASSERT_GT(second->markers.size(), 1u);
  EXPECT_FLOAT_EQ(second->markers.back().color.r, 0.0F);
  EXPECT_FLOAT_EQ(second->markers.back().color.b, 1.0F);
}

TEST_F(ComponentTest, ObjectListRejectsSharedOutputTopic)
{
  // first/object_markers and /first/object_markers are the same topic in the root namespace.
  EXPECT_THROW(
    create(
      kObjectListFactory, rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("sources", std::vector<std::string>{"first", "second"}),
    rclcpp::Parameter("second.output_topic", "/first/object_markers"),
  })), std::invalid_argument);
}

TEST_F(ComponentTest, ObjectListPublishesMarkers)
{
  auto wrapper = create(kObjectListFactory, rclcpp::NodeOptions());
  auto test_node = std::make_shared<rclcpp::Node>("object_list_test");
  MarkerArray::SharedPtr received;
  auto subscription = test_node->create_subscription<MarkerArray>(
    "object_markers", 10, [&received](MarkerArray::SharedPtr msg) {received = msg;});
  auto publisher = test_node->create_publisher<ufil_msgs::msg::ObjectList>("object_list", 10);

  ufil_msgs::msg::ObjectList objects;
  objects.header.frame_id = "map";
  objects.objects.resize(2);
  objects.objects[1].id = 9;
  ASSERT_TRUE(
    spinUntil(
      {node(wrapper), test_node}, [&] {
        if (!received) {
          publisher->publish(objects);
        }
        return received != nullptr;
      }));

  ASSERT_FALSE(received->markers.empty());
  EXPECT_EQ(received->markers.front().action, visualization_msgs::msg::Marker::DELETEALL);
  EXPECT_EQ(received->markers.back().header.frame_id, "map");
  EXPECT_EQ(received->markers.back().ns, "label");
  EXPECT_EQ(received->markers.back().text, "\U0001F697 #9  \u22030%");
}

TEST_F(ComponentTest, LaneletMapLoadsWithDefaultConfiguration)
{
  auto wrapper = create(
    kLaneletMapFactory, rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", LANELET_MAP_CONFIG, "-p", "use_sim_time:=true"}));
  auto lanelet_map = node(wrapper);
  ASSERT_NE(lanelet_map, nullptr);
  EXPECT_EQ(std::string(lanelet_map->get_name()), "lanelet_map_marker_array");
  EXPECT_TRUE(lanelet_map->get_parameter("use_sim_time").as_bool());
  expectConfigMatchesDeclaration(*lanelet_map);
  EXPECT_EQ(lanelet_map->count_publishers("map_markers"), 1u);
}

TEST_F(ComponentTest, LaneletMapPublishesLatchedMarkers)
{
  auto wrapper = create(
    kLaneletMapFactory, rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("map_path", TEST_MAP), rclcpp::Parameter("frame_id", "map_utm")}));

  // Subscribed after the map was published: received because the publisher is transient local.
  auto test_node = std::make_shared<rclcpp::Node>("lanelet_map_test");
  MarkerArray::SharedPtr received;
  auto subscription = test_node->create_subscription<MarkerArray>(
    "map_markers", rclcpp::QoS(1).reliable().transient_local(),
    [&received](MarkerArray::SharedPtr msg) {received = msg;});
  ASSERT_TRUE(spinUntil({node(wrapper), test_node}, [&] {return received != nullptr;}));

  ASSERT_FALSE(received->markers.empty());
  EXPECT_EQ(received->markers.front().header.frame_id, "map_utm");
  EXPECT_EQ(received->markers.front().ns, "lanelet_fill");
  EXPECT_TRUE(received->markers.front().frame_locked);
}

TEST_F(ComponentTest, InvalidParametersFailConstruction)
{
  EXPECT_THROW(
    create(
      kObjectListFactory, rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("covariance.line_width", -1.0)})),
    std::invalid_argument);
  EXPECT_THROW(
    create(
      kLaneletMapFactory, rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("line_width", 0.0)})),
    std::invalid_argument);
}

TEST_F(ComponentTest, UnreadableMapFailsConstruction)
{
  EXPECT_THROW(
    create(
      kLaneletMapFactory, rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("map_path", "/nonexistent/map.osm")})),
    std::runtime_error);
}
