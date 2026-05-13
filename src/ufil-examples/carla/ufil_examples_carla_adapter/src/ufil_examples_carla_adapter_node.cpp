// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "tf2_ros/static_transform_broadcaster.h"

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <derived_object_msgs/msg/object_array.hpp>
#include <carla_msgs/msg/carla_actor_list.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ufil_examples_carla_adapter/msg/carla_actor_list_stamped.hpp>

#include <ufil_examples_carla_adapter/message_converter.hpp>

#include <GeographicLib/UTMUPS.hpp>

using namespace std::chrono_literals;
using SyncPolicy =
  message_filters::sync_policies::ApproximateTime<
  derived_object_msgs::msg::ObjectArray,
  ufil_examples_carla_adapter::msg::CarlaActorListStamped
  >;

class CarlaAdapterNode : public rclcpp::Node
{
private:
  // Axle Geometries
  std::map<std::string, std::vector<ufil_msgs::msg::Axle>> axle_geometries_;

  std::vector<int64_t> vehicle_ids_;
  int n_vehicles_ = 0;

  // Subscribers
  message_filters::Subscriber<derived_object_msgs::msg::ObjectArray> object_array_subscriber_;
  message_filters::Subscriber<ufil_examples_carla_adapter::msg::CarlaActorListStamped>
  actor_list_stamped_subscriber_;
  rclcpp::Subscription<carla_msgs::msg::CarlaActorList>::SharedPtr actor_list_subscriber_;

  // Synchronizer
  std::map<int64_t, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odometry_publishers_;

  // Publishers
  rclcpp::Publisher<ufil_examples_carla_adapter::msg::CarlaActorListStamped>::SharedPtr
    actor_list_stamped_publisher_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr object_list_publisher_;

  // TF Listeners
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Synchronizer
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Static Transform Publisher
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  // Offsets for UTM <-> Local Cartesian
  double offset_x_ = 0;
  double offset_y_ = 0;
  // Other UTM parameters
  int zone_ = 0;
  bool northp_ = false;

  void onSynchronizedMessages(
    const derived_object_msgs::msg::ObjectArray::ConstSharedPtr & obj_msg,
    const ufil_examples_carla_adapter::msg::CarlaActorListStamped::ConstSharedPtr & actor_msg
  )
  {
    std::string target_frame = this->get_parameter("target_frame").as_string();
    auto object_list = ufil_examples_carla_adapter::objectsFromCarla(
      *obj_msg, actor_msg->actor_list, this->vehicle_ids_, this->n_vehicles_,
      this->axle_geometries_);

    try {
      auto transform_msg = this->tf_buffer_->lookupTransform(
        target_frame, object_list.header.frame_id, tf2::TimePointZero);
      auto transformed_object_list = ufil_examples_carla_adapter::transformObjectList(
        object_list, transform_msg);
      this->object_list_publisher_->publish(transformed_object_list);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }
  }

public:
  CarlaAdapterNode()
  : Node("ufil_examples_carla_adapter_node")
  {
    // Declare parameters
    this->declare_parameter("target_frame", "map");

    auto vehicle_ids_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    vehicle_ids_param_desc.description = "IDs of the cooperative vehicles. ";
    vehicle_ids_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
    this->vehicle_ids_ = this->declare_parameter("vehicle_ids", std::vector<int64_t>(),
      vehicle_ids_param_desc);

    // Declare n_vehicles parameter (integer count)
    auto n_vehicles_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    n_vehicles_param_desc.description =
      "Number of cooperative vehicles. Set to -1 to publish data for all cars.";
    n_vehicles_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    // Default to 0
    this->n_vehicles_ = this->declare_parameter("n_vehicles", 0, n_vehicles_param_desc);

    // Check if both parameters are set, if so log an error
    if (!vehicle_ids_.empty() && n_vehicles_ > 0) {
      RCLCPP_INFO(this->get_logger(), "Ignoring vehicle_ids_ because n_vehicles > 0");
    }

    // Process the parameters
    if (n_vehicles_ < -1) {} else if (n_vehicles_ > 0) {
      vehicle_ids_ = {};
      for (int64_t i = 1; i <= n_vehicles_; ++i) {
        vehicle_ids_.push_back(i);
      }
    } else if (vehicle_ids_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No vehicle IDs or number of vehicles provided.");
    }

    // this->vehicle_ids_ = {1,2,3,4,5, 6,7, 8};
    if (n_vehicles_ < 0) {
      RCLCPP_INFO(this->get_logger(), "Vehicle IDs: all");
    } else {
      std::ostringstream oss;
      for (const auto & val : vehicle_ids_) {
        oss << val << " ";
      }
      auto vehicle_ids_str = oss.str();
      RCLCPP_INFO(this->get_logger(), "Vehicle IDs: %s", vehicle_ids_str.c_str());
    }

    auto sub_qos = rclcpp::QoS(1).best_effort().keep_last(1);
    auto pub_qos = rclcpp::SystemDefaultsQoS();

    // Subscribers
    object_array_subscriber_.subscribe(this, "/carla/objects", sub_qos.get_rmw_qos_profile());
    actor_list_stamped_subscriber_.subscribe(this, "/carla/actor_list_stamped",
      sub_qos.get_rmw_qos_profile());
    actor_list_subscriber_ = this->create_subscription<carla_msgs::msg::CarlaActorList>(
    "/carla/actor_list", sub_qos,
      [this](const carla_msgs::msg::CarlaActorList::SharedPtr msg) {
        ufil_examples_carla_adapter::msg::CarlaActorListStamped stamped_msg;
        stamped_msg.header.stamp = this->now();
        stamped_msg.header.frame_id = "map";
        stamped_msg.actor_list = *msg;
        actor_list_stamped_publisher_->publish(stamped_msg);
    });

    // Publishers
    actor_list_stamped_publisher_ =
      this->create_publisher<ufil_examples_carla_adapter::msg::CarlaActorListStamped>(
    "/carla/actor_list_stamped", pub_qos);
    object_list_publisher_ =
      this->create_publisher<ufil_msgs::msg::ObjectList>("object_list", pub_qos);

    // TF listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10),
      object_array_subscriber_,
      actor_list_stamped_subscriber_
    );
    sync_->registerCallback(
      std::bind(
        &CarlaAdapterNode::onSynchronizedMessages, this,
        std::placeholders::_1,
        std::placeholders::_2
      )
    );

    // Calculate offset in UTM for given lab coordinates
    GeographicLib::UTMUPS::Forward(
      50.70720364259, 6.90096282914, this->zone_, this->northp_, this->offset_x_,
      this->offset_y_);  // Coordinates of the physical lab

    // Transforms
    this->tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Axle Geometries

    // Tesla
    ufil_msgs::msg::Axle tesla_axle_1;
    tesla_axle_1.track_width = 2.163450002670288;
    tesla_axle_1.track_width_variance = 1e-9;
    tesla_axle_1.center_to_axle = 4.791779518127441 / 2.0;
    tesla_axle_1.center_to_axle_variance = 1e-9;
    tesla_axle_1.wheel_detected = {true, true};

    ufil_msgs::msg::Axle tesla_axle_2;
    tesla_axle_2.track_width = 2.163450002670288;
    tesla_axle_2.track_width_variance = 1e-9;
    tesla_axle_2.center_to_axle = -4.791779518127441 / 2.0;
    tesla_axle_2.center_to_axle_variance = 1e-9;
    tesla_axle_2.wheel_detected = {true, true};

    axle_geometries_["vehicle.tesla.model3"] = {tesla_axle_1, tesla_axle_2};

    // Kawasaki
    ufil_msgs::msg::Axle kawasaki_axle_1;
    kawasaki_axle_1.center_to_axle = 2.043684244155884 / 2.0;
    kawasaki_axle_1.center_to_axle_variance = 1e-9;
    kawasaki_axle_1.wheel_detected = {true};

    ufil_msgs::msg::Axle kawasaki_axle_2;
    kawasaki_axle_2.center_to_axle = -2.043684244155884 / 2.0;
    kawasaki_axle_2.center_to_axle_variance = 1e-9;
    kawasaki_axle_2.wheel_detected = {true};

    axle_geometries_["vehicle.kawasaki.ninja"] = {kawasaki_axle_1, kawasaki_axle_2};

    // Carla Motors
    ufil_msgs::msg::Axle hgv_axle_1;
    hgv_axle_1.track_width = 2.8910882472991943;
    hgv_axle_1.track_width_variance = 1e-9;
    hgv_axle_1.center_to_axle = 7.935710430145264 / 2.0;
    hgv_axle_1.center_to_axle_variance = 1e-9;
    hgv_axle_1.wheel_detected = {true, true};

    ufil_msgs::msg::Axle hgv_axle_2;
    hgv_axle_2.track_width = 2.8910882472991943;
    hgv_axle_2.track_width_variance = 1e-9;
    hgv_axle_2.center_to_axle = -7.935710430145264 / 2.0;
    hgv_axle_2.center_to_axle_variance = 1e-9;
    hgv_axle_2.wheel_detected = {true, true, true, true};

    ufil_msgs::msg::Axle hgv_axle_3;
    hgv_axle_3.track_width = 2.8910882472991943;
    hgv_axle_3.track_width_variance = 1e-9;
    hgv_axle_3.center_to_axle = -7.935710430145264 / 2.0 * 0.7;
    hgv_axle_3.center_to_axle_variance = 1e-9;
    hgv_axle_3.wheel_detected = {true, true, true, true};

    axle_geometries_["vehicle.carlamotors.european_hgv"] = {hgv_axle_1, hgv_axle_2, hgv_axle_3};
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CarlaAdapterNode>();

  RCLCPP_INFO(node->get_logger(), "Started CARLA adapter.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Stopped CARLA adapter.");

  return 0;
}
