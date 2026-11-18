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

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <cpm_lab_lab_msgs/msg/vehicle_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include "ufil_examples_cpm_lab_adapter/vehicle_object.hpp"
#include "CpmLabAdapterParameterRos.hpp"
#include "ufil_examples_cpm_lab_adapter_ros.hpp"

namespace ufil_examples_cpm_lab_adapter
{

/**
 * @brief ROS 2 component publishing the states of CPM Lab vehicles as UFIL object lists.
 *
 * Subscribes:  vehicle_state_topic with {id} replaced, per vehicle in vehicle_ids
 *              (cpm_lab_lab_msgs/VehicleState, best effort)
 * Publishes:   vehicle_<id>/object_list (ufil_msgs/ObjectList), the vehicle alone, on every
 *              received state; e.g. the input of its on-board unit.
 *              object_list (ufil_msgs/ObjectList), all vehicles with a state within
 *              state_timeout, at publish_rate; one list for all keeps the visualization steady.
 *              /diagnostics
 *
 * The vehicle states are scaled to real-world dimensions and transformed from source_frame into
 * target_frame.
 */
class CpmLabAdapterNode : public rclcpp::Node
{
public:
  explicit CpmLabAdapterNode(const rclcpp::NodeOptions & options)
  : Node("cpm_lab_adapter", options),
    parameters_(loadParameters(*this)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, this, false),
    diagnostics_(this)
  {
    publisher_ = create_publisher<ufil_msgs::msg::ObjectList>("object_list", 10);
    std::string ids;
    for (const auto id : parameters_.vehicleIds()) {
      auto & vehicle = vehicles_[id];
      vehicle.publisher = create_publisher<ufil_msgs::msg::ObjectList>(
        "vehicle_" + std::to_string(id) + "/object_list", 10);
      vehicle.subscription = create_subscription<cpm_lab_lab_msgs::msg::VehicleState>(
        parameters_.vehicleStateTopic(id), rclcpp::QoS(10).best_effort(),
        [this, id](const cpm_lab_lab_msgs::msg::VehicleState::ConstSharedPtr & msg) {
          onVehicleState(id, *msg);
        });
      ids += (ids.empty() ? "" : " ") + std::to_string(id);
    }
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / parameters_.publishRate()),
      [this]() {publishAll();});

    RCLCPP_INFO(
      get_logger(), "Publishing vehicles %s from '%s' as objects in '%s' on '%s'.", ids.c_str(),
      parameters_.vehicleStateTopic(parameters_.vehicleIds().front()).c_str(),
      parameters_.targetFrame().c_str(), publisher_->get_topic_name());

    diagnostics_.setHardwareID("cpm_lab");
    diagnostics_.add("CPM Lab adapter", this, &CpmLabAdapterNode::produceDiagnostics);
  }

private:
  struct Vehicle
  {
    rclcpp::Subscription<cpm_lab_lab_msgs::msg::VehicleState>::SharedPtr subscription;
    rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher;
    /// The vehicle as object in target_frame, from the latest state that could be transformed.
    std::optional<ufil_msgs::msg::ObjectList> objects;
    rclcpp::Time received;
    std::uint64_t states_received = 0;
  };

  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static CpmLabAdapterParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return CpmLabAdapterParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onVehicleState(std::uint8_t id, const cpm_lab_lab_msgs::msg::VehicleState & msg)
  {
    auto & vehicle = vehicles_.at(id);
    vehicle.states_received++;
    const VehicleObject object = toVehicleObject(conversions::vehicleStateFromMsg(msg), id);

    ufil_msgs::msg::ObjectList objects;
    objects.header.stamp = msg.header.stamp;
    objects.header.frame_id = parameters_.sourceFrame();
    objects.objects.push_back(conversions::objectToMsg(object));

    try {
      vehicle.objects = tf_buffer_.transform(objects, parameters_.targetFrame());
      vehicle.received = now();
      vehicle.publisher->publish(*vehicle.objects);
      objects_published_++;
      last_error_.clear();
    } catch (const tf2::TransformException & error) {
      transform_failures_++;
      last_error_ = error.what();
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 5000, "Dropping state of vehicle %u, cannot transform '%s' "
        "to '%s': %s", id, parameters_.sourceFrame().c_str(), parameters_.targetFrame().c_str(),
        error.what());
    }
  }

  bool isRecent(const Vehicle & vehicle, const rclcpp::Time & time) const
  {
    return vehicle.objects && (time - vehicle.received).seconds() <= parameters_.stateTimeout();
  }

  /// Publishes the recent vehicles in one list, stamped with the newest of their states.
  void publishAll()
  {
    const auto time = now();
    rclcpp::Time newest(0, 0, time.get_clock_type());
    ufil_msgs::msg::ObjectList objects;
    objects.header.frame_id = parameters_.targetFrame();
    for (const auto & [id, vehicle] : vehicles_) {
      if (!isRecent(vehicle, time)) {
        continue;
      }
      objects.objects.push_back(vehicle.objects->objects.front());
      newest = std::max(newest, rclcpp::Time(vehicle.objects->header.stamp, time.get_clock_type()));
    }
    objects.header.stamp = objects.objects.empty() ? time : newest;
    publisher_->publish(objects);
    lists_published_++;
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const auto time = now();
    std::uint64_t states_received = 0;
    std::string missing;
    for (const auto & [id, vehicle] : vehicles_) {
      states_received += vehicle.states_received;
      if (!isRecent(vehicle, time)) {
        missing += (missing.empty() ? "" : " ") + std::to_string(id);
      }
      diagnostic.add(
        "Vehicle " + std::to_string(id) + " states received", vehicle.states_received);
    }
    if (states_received == 0) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for vehicle states");
    } else if (!last_error_.empty()) {
      diagnostic.summaryf(
        DiagnosticStatus::WARN, "Cannot transform the vehicle state: %s", last_error_.c_str());
    } else if (!missing.empty()) {
      diagnostic.summaryf(
        DiagnosticStatus::WARN, "No recent state of vehicles %s", missing.c_str());
    } else {
      diagnostic.summary(DiagnosticStatus::OK, "Publishing");
    }
    diagnostic.add("Vehicles without recent state", missing);
    diagnostic.add("Target frame", parameters_.targetFrame());
    diagnostic.add("States received", states_received);
    diagnostic.add("Vehicle object lists published", objects_published_);
    diagnostic.add("Object lists with all vehicles published", lists_published_);
    diagnostic.add("Transform failures", transform_failures_);
    diagnostic.add("Last error", last_error_);
  }

  CpmLabAdapterParameterRos parameters_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_;
  std::map<std::uint8_t, Vehicle> vehicles_;
  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater diagnostics_;

  std::uint64_t objects_published_ = 0;
  std::uint64_t lists_published_ = 0;
  std::uint64_t transform_failures_ = 0;
  std::string last_error_;
};

}  // namespace ufil_examples_cpm_lab_adapter

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_examples_cpm_lab_adapter::CpmLabAdapterNode)
