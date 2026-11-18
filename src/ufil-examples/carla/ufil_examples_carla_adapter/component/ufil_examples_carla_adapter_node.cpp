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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <carla_msgs/msg/carla_actor_list.hpp>
#include <derived_object_msgs/msg/object_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_examples_carla_adapter/carla_adapter.hpp"
#include "UfilExamplesCarlaAdapterParameterRos.hpp"
#include "ufil_examples_carla_adapter_ros.hpp"

namespace ufil_examples_carla_adapter
{

/**
 * @brief ROS 2 component converting the ground-truth objects of the CARLA ROS bridge into UFIL
 *        object lists.
 *
 * Subscribes:  objects (derived_object_msgs/ObjectArray), e.g. /carla/objects
 *              actor_list (carla_msgs/CarlaActorList), e.g. /carla/actor_list; the latest list
 *              provides classification, axles and role names of the objects.
 * Publishes:   object_list (ufil_msgs/ObjectList) in target_frame, one per received object
 *              array once an actor list was received.
 *              /diagnostics
 */
class CarlaAdapterNode : public rclcpp::Node
{
public:
  explicit CarlaAdapterNode(const rclcpp::NodeOptions & options)
  : Node("carla_adapter", options),
    parameters_(loadParameters(*this)),
    adapter_(parameters_.config()),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, this),
    diagnostics_(this)
  {
    // The CARLA ROS bridge publishes best effort, keep only the latest data.
    const auto input_qos = rclcpp::SensorDataQoS().keep_last(1);
    publisher_ = create_publisher<ufil_msgs::msg::ObjectList>("object_list", 10);
    actor_list_subscription_ = create_subscription<carla_msgs::msg::CarlaActorList>(
      "actor_list", input_qos,
      [this](const carla_msgs::msg::CarlaActorList::ConstSharedPtr & msg) {onActorList(*msg);});
    objects_subscription_ = create_subscription<derived_object_msgs::msg::ObjectArray>(
      "objects", input_qos,
      [this](const derived_object_msgs::msg::ObjectArray::ConstSharedPtr & msg) {
        onObjects(*msg);
      });

    std::string vehicles = "all objects";
    if (adapter_.config().filter_cooperative_vehicles) {
      vehicles = "cooperative vehicles";
      for (const auto id : adapter_.config().cooperative_vehicle_ids) {
        vehicles += " " + std::to_string(id);
      }
    }
    RCLCPP_INFO(
      get_logger(), "Converting %s from '%s' (actors from '%s') to '%s' in frame '%s'.",
      vehicles.c_str(), objects_subscription_->get_topic_name(),
      actor_list_subscription_->get_topic_name(), publisher_->get_topic_name(),
      parameters_.targetFrame().c_str());

    diagnostics_.setHardwareID("carla");
    diagnostics_.add("CARLA adapter", this, &CarlaAdapterNode::produceDiagnostics);
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static UfilExamplesCarlaAdapterParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return UfilExamplesCarlaAdapterParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onActorList(const carla_msgs::msg::CarlaActorList & msg)
  {
    if (!adapter_.hasActors()) {
      RCLCPP_INFO(
        get_logger(), "Received the first actor list with %zu actors.", msg.actors.size());
    }
    adapter_.updateActors(conversions::actorsFromMsg(msg));
  }

  void onObjects(const derived_object_msgs::msg::ObjectArray & msg)
  {
    if (!adapter_.hasActors()) {
      ++objects_without_actors_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping objects, waiting for an actor list on '%s'.",
        actor_list_subscription_->get_topic_name());
      return;
    }

    const auto objects = adapter_.convert(conversions::objectsFromMsg(msg));

    Pose2D pose;
    try {
      pose = conversions::poseFromMsg(
        tf_buffer_.lookupTransform(
          parameters_.targetFrame(), msg.header.frame_id, tf2::TimePointZero));
    } catch (const tf2::TransformException & error) {
      ++transform_failures_;
      last_transform_error_ = error.what();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping objects, no transform from '%s' to '%s': %s",
        msg.header.frame_id.c_str(), parameters_.targetFrame().c_str(), error.what());
      return;
    }
    last_transform_error_.clear();

    auto header = msg.header;
    header.frame_id = parameters_.targetFrame();
    publisher_->publish(conversions::toMsg(transformObjects(objects, pose), header));
    ++published_lists_;
    RCLCPP_DEBUG(
      get_logger(), "Published %zu of %zu objects.", objects.size(), msg.objects.size());
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const auto & statistics = adapter_.statistics();

    if (!adapter_.hasActors()) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for an actor list");
    } else if (!last_transform_error_.empty()) {
      diagnostic.summary(
        DiagnosticStatus::ERROR, "No transform to " + parameters_.targetFrame());
    } else if (statistics.object_arrays == 0) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for objects");
    } else {
      diagnostic.summary(DiagnosticStatus::OK, "Converting objects");
    }

    diagnostic.add("Target frame", parameters_.targetFrame());
    diagnostic.add("Received actor lists", statistics.actor_lists);
    diagnostic.add("Received object arrays", statistics.object_arrays);
    diagnostic.add("Object arrays dropped without actor list", objects_without_actors_);
    diagnostic.add("Object arrays dropped without transform", transform_failures_);
    diagnostic.add("Published object lists", published_lists_);
    diagnostic.add("Received objects", statistics.objects);
    diagnostic.add("Converted objects", statistics.converted_objects);
    diagnostic.add("Skipped non-cooperative objects", statistics.skipped_objects);
    diagnostic.add("Objects missing in the actor list", statistics.unknown_actors);
    if (!last_transform_error_.empty()) {
      diagnostic.add("Transform error", last_transform_error_);
    }
  }

  UfilExamplesCarlaAdapterParameterRos parameters_;
  CarlaAdapter adapter_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<carla_msgs::msg::CarlaActorList>::SharedPtr actor_list_subscription_;
  rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr objects_subscription_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_;

  std::uint64_t objects_without_actors_ = 0;
  std::uint64_t transform_failures_ = 0;
  std::uint64_t published_lists_ = 0;
  std::string last_transform_error_;

  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_examples_carla_adapter

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_examples_carla_adapter::CarlaAdapterNode)
