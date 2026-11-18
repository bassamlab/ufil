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
#include <stdexcept>
#include <string>
#include <vector>

#include <class_loader/class_loader.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <ssl_vehicle_tracking_msgs/msg/track_array.hpp>
#include <ssl_vehicle_tracking_msgs/msg/wheel_array.hpp>

namespace
{

constexpr char kFactoryName[] =
  "rclcpp_components::NodeFactoryTemplate<ssl_wheel_tracker::SslWheelTrackerNode>";

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
    rclcpp::NodeOptions().arguments(
      {"--ros-args", "--params-file", DEFAULT_CONFIG, "-p", "use_sim_time:=true"}));
  // The registered class derives from rclcpp::Node only.
  auto node = std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(std::string(node->get_name()), "ssl_wheel_tracker");
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

  EXPECT_EQ(node->count_subscribers("ssl_1/map"), 1u);
  EXPECT_EQ(node->count_publishers("wheel_tracks"), 1u);
  EXPECT_EQ(node->count_publishers("wheel_detections"), 1u);
}

TEST_F(ComponentTest, InvalidParametersFailConstruction)
{
  EXPECT_THROW(
    load(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("update_period", -1.0)})),
    std::invalid_argument);
}

TEST_F(ComponentTest, TracksAWheelOnTheMat)
{
  auto wrapper = load(
    rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("input_topics", std::vector<std::string>{"/component_test/grid"})}));
  auto node = std::static_pointer_cast<rclcpp::Node>(wrapper.get_node_instance());

  auto io = std::make_shared<rclcpp::Node>("component_test_io");
  const auto qos = rclcpp::QoS(10).best_effort();
  auto publisher = io->create_publisher<nav_msgs::msg::OccupancyGrid>("/component_test/grid", qos);
  std::vector<ssl_vehicle_tracking_msgs::msg::WheelArray> detections;
  std::vector<ssl_vehicle_tracking_msgs::msg::TrackArray> tracks;
  auto detection_subscription = io->create_subscription<ssl_vehicle_tracking_msgs::msg::WheelArray>(
    "/wheel_detections", qos,
    [&](const ssl_vehicle_tracking_msgs::msg::WheelArray & msg) {detections.push_back(msg);});
  auto track_subscription = io->create_subscription<ssl_vehicle_tracking_msgs::msg::TrackArray>(
    "/wheel_tracks", qos,
    [&](const ssl_vehicle_tracking_msgs::msg::TrackArray & msg) {tracks.push_back(msg);});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(io);
  const auto spin_for = [&](std::chrono::milliseconds duration) {
      const auto end = std::chrono::steady_clock::now() + duration;
      while (std::chrono::steady_clock::now() < end) {
        executor.spin_some(std::chrono::milliseconds(5));
      }
    };
  // Wait for discovery.
  for (int i = 0; i < 200 && publisher->get_subscription_count() == 0; ++i) {
    spin_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(publisher->get_subscription_count(), 1u);

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = 10;
  grid.info.height = 8;
  grid.info.resolution = 0.2f;
  grid.data.assign(80, 0);
  grid.data[3 * 10 + 4] = 20;
  for (int frame = 0; frame < 6; ++frame) {
    grid.header.stamp = rclcpp::Time(100, static_cast<std::uint32_t>(frame) * 23000000U);
    publisher->publish(grid);
    spin_for(std::chrono::milliseconds(50));
  }
  spin_for(std::chrono::milliseconds(100));

  ASSERT_FALSE(detections.empty());
  ASSERT_EQ(detections.back().wheels.size(), 1u);
  EXPECT_NEAR(detections.back().wheels[0].x, 0.9, 1e-5);
  EXPECT_NEAR(detections.back().wheels[0].y, 0.7, 1e-5);

  ASSERT_FALSE(tracks.empty());
  ASSERT_EQ(tracks.back().tracks.size(), 1u);
  EXPECT_EQ(tracks.back().header.frame_id, "map");
  EXPECT_EQ(tracks.back().tracks[0].track_id, "1");
  EXPECT_NEAR(tracks.back().tracks[0].states.back().x, 0.9, 0.01);
}
