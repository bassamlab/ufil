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

#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <class_loader/class_loader.hpp>

#include <cpm_lab_lab_msgs/msg/vehicle_state.hpp>
#include <cpm_lab_map_msgs/msg/lane_control_light.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "udp_receiver.hpp"

namespace
{

constexpr char kAdapterFactory[] =
  "rclcpp_components::NodeFactoryTemplate<ufil_examples_cpm_lab_adapter::CpmLabAdapterNode>";
constexpr char kSensorFactory[] =
  "rclcpp_components::NodeFactoryTemplate<ufil_examples_cpm_lab_adapter::CpmLabSensorNode>";
constexpr char kOscFactory[] =
  "rclcpp_components::NodeFactoryTemplate<"
  "ufil_examples_cpm_lab_adapter::OscLaneControlLightNode>";

bool isBuiltinParameter(const std::string & name)
{
  return name == "use_sim_time" || name == "start_type_description_service" ||
         name.rfind("qos_overrides.", 0) == 0;
}

/// The parameters of the configuration file and the declared parameters match exactly.
void expectConfigMatchesDeclaration(rclcpp::Node & node)
{
  const auto & overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, value] : overrides) {
    if (!isBuiltinParameter(name)) {
      EXPECT_TRUE(node.has_parameter(name)) << name << " in the config file is not declared";
    }
  }
  for (const auto & name : node.list_parameters({}, 0).names) {
    if (!isBuiltinParameter(name)) {
      EXPECT_TRUE(overrides.count(name)) << name << " is declared but missing in the config file";
    }
  }
}

rclcpp::NodeOptions withConfig(const char * config_file)
{
  return rclcpp::NodeOptions().arguments(
    {"--ros-args", "--params-file", config_file, "-p", "use_sim_time:=true"});
}

}  // namespace

class ComponentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  rclcpp::Node::SharedPtr create(const char * factory_name, const rclcpp::NodeOptions & options)
  {
    auto factory = loader_.createInstance<rclcpp_components::NodeFactory>(factory_name);
    wrappers_.push_back(factory->create_node_instance(options));
    // The registered classes derive from rclcpp::Node only.
    return std::static_pointer_cast<rclcpp::Node>(wrappers_.back().get_node_instance());
  }

  /// The lab frame is the map frame shifted by (10, 20).
  void publishLabFrame()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = test_node_->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "lab";
    transform.transform.translation.x = 10.0;
    transform.transform.translation.y = 20.0;
    transform.transform.rotation.w = 1.0;
    tf_broadcaster_.sendTransform(transform);
  }

  cpm_lab_lab_msgs::msg::VehicleState vehicleState()
  {
    cpm_lab_lab_msgs::msg::VehicleState msg;
    msg.vehicle_id = 3;
    msg.header.stamp = test_node_->now();
    msg.header.frame_id = "lab";
    msg.pose.position.x = 1.0;
    msg.pose.position.y = 2.0;
    msg.pose.orientation.w = 1.0;
    msg.twist.linear.x = 0.5;
    return msg;
  }

  /// Spins the nodes, calling step in every iteration, until done() holds or 10 s passed.
  template<typename Step, typename Done>
  bool spinUntil(rclcpp::Node::SharedPtr node, Step step, Done done)
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(test_node_);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      step();
      executor.spin_some(std::chrono::milliseconds(50));
      if (done()) {
        return true;
      }
    }
    return false;
  }

  class_loader::ClassLoader loader_{COMPONENT_LIBRARY};
  std::vector<rclcpp_components::NodeInstanceWrapper> wrappers_;
  rclcpp::Node::SharedPtr test_node_ = std::make_shared<rclcpp::Node>("component_test");
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_{test_node_};
};

TEST_F(ComponentTest, AllComponentsAreRegistered)
{
  const auto classes = loader_.getAvailableClasses<rclcpp_components::NodeFactory>();
  for (const char * name : {kAdapterFactory, kSensorFactory, kOscFactory}) {
    EXPECT_NE(std::find(classes.begin(), classes.end(), name), classes.end()) << name;
  }
}

TEST_F(ComponentTest, AdapterLoadsWithDefaultConfiguration)
{
  auto node = create(kAdapterFactory, withConfig(ADAPTER_CONFIG));
  EXPECT_EQ(std::string(node->get_name()), "cpm_lab_adapter");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_EQ(node->get_parameter("vehicle_ids").as_integer_array(), std::vector<std::int64_t>{1});
  expectConfigMatchesDeclaration(*node);
  EXPECT_EQ(node->count_subscribers("/lab/vehicle_1/vehicleState"), 1u);
  EXPECT_EQ(node->count_publishers("/vehicle_1/object_list"), 1u);
  EXPECT_EQ(node->count_publishers("/object_list"), 1u);
}

TEST_F(ComponentTest, SensorLoadsWithDefaultConfiguration)
{
  auto node = create(kSensorFactory, withConfig(SENSOR_CONFIG));
  EXPECT_EQ(std::string(node->get_name()), "cpm_lab_sensor");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_DOUBLE_EQ(node->get_parameter("gnss.frequency").as_double(), 5.0);
  expectConfigMatchesDeclaration(*node);
  EXPECT_EQ(node->count_subscribers("/vehicle_state"), 1u);
  for (const char * topic : {"/gps", "/imu", "/wheel"}) {
    EXPECT_EQ(node->count_publishers(topic), 1u) << topic;
  }
}

TEST_F(ComponentTest, OscLoadsWithDefaultConfiguration)
{
  auto node = create(kOscFactory, withConfig(OSC_CONFIG));
  EXPECT_EQ(std::string(node->get_name()), "osc_lane_control_light");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());
  EXPECT_EQ(node->get_parameter("receiver.port").as_int(), 9000);
  expectConfigMatchesDeclaration(*node);
  EXPECT_EQ(node->count_subscribers("/lane_control_lights"), 1u);
}

TEST_F(ComponentTest, InvalidParametersFailConstruction)
{
  EXPECT_THROW(
    create(
      kAdapterFactory,
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("vehicle_ids", std::vector<std::int64_t>{256})})),
    std::invalid_argument);
  EXPECT_THROW(
    create(
      kSensorFactory,
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("gnss.frequency", 0.0)})),
    std::invalid_argument);
  EXPECT_THROW(
    create(
      kOscFactory,
      rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("osc_address", "x")})),
    std::invalid_argument);
  EXPECT_THROW(
    create(
      kOscFactory,
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("receiver.host", "no-such-host.invalid")})),
    std::runtime_error);
}

TEST_F(ComponentTest, AdapterPublishesTransformedObjects)
{
  auto node = create(
    kAdapterFactory,
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("vehicle_ids", std::vector<std::int64_t>{3, 5})}));
  publishLabFrame();

  auto publisher_3 = test_node_->create_publisher<cpm_lab_lab_msgs::msg::VehicleState>(
    "/lab/vehicle_3/vehicleState", rclcpp::QoS(10).best_effort());
  auto publisher_5 = test_node_->create_publisher<cpm_lab_lab_msgs::msg::VehicleState>(
    "/lab/vehicle_5/vehicleState", rclcpp::QoS(10).best_effort());
  std::optional<ufil_msgs::msg::ObjectList> vehicle_3;
  std::optional<ufil_msgs::msg::ObjectList> all;
  auto vehicle_subscription = test_node_->create_subscription<ufil_msgs::msg::ObjectList>(
    "/vehicle_3/object_list", 10,
    [&vehicle_3](const ufil_msgs::msg::ObjectList & msg) {vehicle_3 = msg;});
  auto all_subscription = test_node_->create_subscription<ufil_msgs::msg::ObjectList>(
    "/object_list", 10, [&all](const ufil_msgs::msg::ObjectList & msg) {
      if (msg.objects.size() == 2u) {
        all = msg;
      }
    });

  ASSERT_TRUE(
    spinUntil(
      node,
      [&]() {
        publisher_3->publish(vehicleState());
        auto state = vehicleState();
        state.pose.position.x = 2.0;
        publisher_5->publish(state);
      },
      [&]() {return vehicle_3 && all;}));
  EXPECT_EQ(vehicle_3->header.frame_id, "map");
  ASSERT_EQ(vehicle_3->objects.size(), 1u);
  EXPECT_EQ(vehicle_3->objects[0].id, 3u);
  // Position scaled by 18 in the lab frame, then shifted by the lab frame offset.
  EXPECT_NEAR(vehicle_3->objects[0].state.state.x, 10.0 + 18.0, 1e-9);
  EXPECT_NEAR(vehicle_3->objects[0].state.state.y, 20.0 + 36.0, 1e-9);

  // All vehicles in one list, ordered by id; the object id comes from vehicle_ids, not the state.
  EXPECT_EQ(all->header.frame_id, "map");
  EXPECT_EQ(all->objects[0].id, 3u);
  EXPECT_EQ(all->objects[1].id, 5u);
  EXPECT_NEAR(all->objects[1].state.state.x, 10.0 + 36.0, 1e-9);
}

TEST_F(ComponentTest, AdapterLeavesOutVehiclesWithoutRecentState)
{
  auto node = create(
    kAdapterFactory,
    rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("vehicle_ids", std::vector<std::int64_t>{3, 5}),
    rclcpp::Parameter("state_timeout", 0.2),
  }));
  publishLabFrame();

  auto publisher = test_node_->create_publisher<cpm_lab_lab_msgs::msg::VehicleState>(
    "/lab/vehicle_3/vehicleState", rclcpp::QoS(10).best_effort());
  std::vector<std::size_t> sizes;
  auto subscription = test_node_->create_subscription<ufil_msgs::msg::ObjectList>(
    "/object_list", 10,
    [&sizes](const ufil_msgs::msg::ObjectList & msg) {sizes.push_back(msg.objects.size());});

  // Vehicle 3 appears in the list (vehicle 5 never sends), then times out after the last state.
  bool sending = true;
  ASSERT_TRUE(
    spinUntil(
      node,
      [&]() {
        if (sending) {
          publisher->publish(vehicleState());
        }
      },
      [&]() {
        if (sending && !sizes.empty() && sizes.back() == 1u) {
          sending = false;
          sizes.clear();
        }
        return !sending && !sizes.empty() && sizes.back() == 0u;
      }));
  EXPECT_LE(*std::max_element(sizes.begin(), sizes.end()), 1u);
}

TEST_F(ComponentTest, SensorPublishesMeasurements)
{
  auto node = create(
    kSensorFactory,
    rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("vehicle_id", 3),
    rclcpp::Parameter("gnss.artificial_errors", false),
  }));
  publishLabFrame();

  auto publisher = test_node_->create_publisher<cpm_lab_lab_msgs::msg::VehicleState>(
    "/vehicle_state", rclcpp::QoS(10).best_effort());
  std::optional<sensor_msgs::msg::NavSatFix> fix;
  std::optional<sensor_msgs::msg::Imu> imu;
  std::optional<nav_msgs::msg::Odometry> odometry;
  auto gnss_subscription = test_node_->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps", 10, [&fix](const sensor_msgs::msg::NavSatFix & msg) {fix = msg;});
  auto imu_subscription = test_node_->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10, [&imu](const sensor_msgs::msg::Imu & msg) {imu = msg;});
  auto odometry_subscription = test_node_->create_subscription<nav_msgs::msg::Odometry>(
    "/wheel", 10, [&odometry](const nav_msgs::msg::Odometry & msg) {odometry = msg;});

  ASSERT_TRUE(
    spinUntil(
      node, [&]() {publisher->publish(vehicleState());},
      [&]() {return fix && imu && odometry;}));
  EXPECT_EQ(fix->header.frame_id, "gnss_3");
  EXPECT_DOUBLE_EQ(fix->position_covariance[0], 0.1);
  // (11, 22) lab metres from the origin are about (198, 396) m: north-east of the CPM Lab.
  EXPECT_GT(fix->latitude, 48.079772);
  EXPECT_GT(fix->longitude, 11.635734);
  EXPECT_EQ(imu->header.frame_id, "imu_3");
  EXPECT_EQ(odometry->header.frame_id, "odom_3");
  EXPECT_EQ(odometry->child_frame_id, "base_link_3");
  EXPECT_DOUBLE_EQ(odometry->twist.twist.linear.x, 9.0);
}

TEST_F(ComponentTest, OscForwardsLaneControlLights)
{
  ufil_examples_cpm_lab_adapter::test::UdpReceiver receiver;
  auto node = create(
    kOscFactory,
    rclcpp::NodeOptions().parameter_overrides(
  {
    rclcpp::Parameter("receiver.port", receiver.port()),
    rclcpp::Parameter("osc_address", "/lights"),
  }));

  cpm_lab_map_msgs::msg::LaneControlLight msg;
  msg.header.stamp.sec = 2;
  msg.header.frame_id = "map";
  msg.segments.resize(1);
  msg.segments[0].segment_index = 4;
  msg.segments[0].image_name = "282-black";
  auto publisher =
    test_node_->create_publisher<cpm_lab_map_msgs::msg::LaneControlLight>("/lane_control_lights",
    10);

  std::optional<ufil_examples_cpm_lab_adapter::test::OscMessage> message;
  ASSERT_TRUE(
    spinUntil(
      node, [&]() {publisher->publish(msg);},
      [&]() {
        message = receiver.receive(std::chrono::milliseconds(50));
        return message.has_value();
      }));
  EXPECT_EQ(message->address, "/lights");
  EXPECT_EQ(
    message->argument,
    R"({"lane_control_light":{"header":{"seconds":2.000000,"frame_id":"map"},"segments":[)"
    R"({"segment_index":4,"image_name":"282-black"}]}})");
}
