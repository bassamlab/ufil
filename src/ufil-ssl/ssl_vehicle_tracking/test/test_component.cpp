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
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <class_loader/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <ssl_vehicle_tracking_msgs/msg/track_array.hpp>
#include <ufil_msgs/msg/object_list.hpp>

namespace
{

constexpr char kFactoryName[] =
  "rclcpp_components::NodeFactoryTemplate<ssl_vehicle_tracking::SslVehicleTrackingNode>";

bool isBuiltinParameter(const std::string & name)
{
  return name == "use_sim_time" || name == "start_type_description_service" ||
         name.rfind("qos_overrides.", 0) == 0;
}

/// The installed default template is not available in the build tree.
rclcpp::Parameter templateParameter()
{
  return rclcpp::Parameter(
    "vehicle_templates",
    std::vector<std::string>{std::string(TEMPLATE_DIR) + "/muCar_scaled_lab.json"});
}

}  // namespace

class ComponentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  rclcpp_components::NodeInstanceWrapper load(const rclcpp::NodeOptions & options)
  {
    factory_ = loader_.createInstance<rclcpp_components::NodeFactory>(kFactoryName);
    return factory_->create_node_instance(options);
  }

  class_loader::ClassLoader loader_{COMPONENT_LIBRARY};
  std::shared_ptr<rclcpp_components::NodeFactory> factory_;
};

TEST_F(ComponentTest, IsRegistered)
{
  const auto classes = loader_.getAvailableClasses<rclcpp_components::NodeFactory>();
  EXPECT_NE(std::find(classes.begin(), classes.end(), kFactoryName), classes.end());
}

TEST_F(ComponentTest, LoadsWithDefaultConfiguration)
{
  auto wrapper = load(
    rclcpp::NodeOptions()
    .arguments({"--ros-args", "--params-file", DEFAULT_CONFIG, "-p", "use_sim_time:=true"})
    .parameter_overrides({templateParameter()}));
  // The registered class derives from rclcpp::Node only.
  auto node = std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(std::string(node->get_name()), "ssl_vehicle_tracking");
  EXPECT_TRUE(node->get_parameter("use_sim_time").as_bool());

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

  EXPECT_EQ(node->count_subscribers("wheel_tracks"), 1u);
  EXPECT_EQ(node->count_publishers("vehicles"), 1u);
}

TEST_F(ComponentTest, InvalidParametersFailConstruction)
{
  EXPECT_THROW(
    load(
      rclcpp::NodeOptions().parameter_overrides(
        {templateParameter(), rclcpp::Parameter("search_width", -1.0)})),
    std::invalid_argument);
}

TEST_F(ComponentTest, PublishesDetectedVehicles)
{
  auto wrapper = load(rclcpp::NodeOptions().parameter_overrides({templateParameter()}));
  auto node = std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());

  auto io = std::make_shared<rclcpp::Node>("component_test_io");
  auto publisher = io->create_publisher<ssl_vehicle_tracking_msgs::msg::TrackArray>(
    "/wheel_tracks", rclcpp::QoS(10).best_effort());
  std::vector<ufil_msgs::msg::ObjectList> received;
  auto subscription = io->create_subscription<ufil_msgs::msg::ObjectList>(
    "/vehicles", 10,
    [&](const ufil_msgs::msg::ObjectList & msg) {received.push_back(msg);});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(io);
  const auto spin_for = [&](std::chrono::milliseconds duration) {
      const auto end = std::chrono::steady_clock::now() + duration;
      while (std::chrono::steady_clock::now() < end) {
        executor.spin_some(std::chrono::milliseconds(5));
      }
    };
  for (int i = 0; i < 200 && (publisher->get_subscription_count() == 0 ||
    subscription->get_publisher_count() == 0); ++i)
  {
    spin_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(publisher->get_subscription_count(), 1u);

  // Wheels of muCar_scaled_lab at (10, 5), driving along x.
  const std::vector<std::vector<float>> wheels{
    {11.61f, 4.1f}, {11.61f, 5.9f}, {8.74f, 4.055f}, {8.74f, 5.945f}};
  ssl_vehicle_tracking_msgs::msg::TrackArray msg;
  msg.header.frame_id = "map";
  msg.header.stamp.sec = 12;
  for (std::size_t i = 0; i < wheels.size(); ++i) {
    ssl_vehicle_tracking_msgs::msg::Track track;
    track.track_id = std::to_string(i + 1);
    ssl_vehicle_tracking_msgs::msg::TrackState state;
    state.x = wheels[i][0];
    state.y = wheels[i][1];
    state.vx = 2.0f;
    state.state = "matched";
    track.states.push_back(state);
    msg.tracks.push_back(track);
  }
  publisher->publish(msg);
  spin_for(std::chrono::milliseconds(300));

  ASSERT_FALSE(received.empty());
  const auto & objects = received.back();
  EXPECT_EQ(objects.header.frame_id, "map");
  EXPECT_EQ(objects.header.stamp.sec, 12);
  ASSERT_EQ(objects.objects.size(), 1u);
  EXPECT_NEAR(objects.objects[0].state.state.x, (11.61 + 8.74) / 2.0, 1e-5);
  EXPECT_NEAR(objects.objects[0].state.state.y, 5.0, 1e-5);
  EXPECT_NEAR(objects.objects[0].state.state.yaw, 0.0, 1e-9);
  EXPECT_EQ(objects.objects[0].axles.size(), 2u);
}
