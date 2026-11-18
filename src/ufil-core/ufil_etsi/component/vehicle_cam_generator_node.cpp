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

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include "ufil_etsi/types.hpp"
#include "ufil_etsi/vehicle_cam_generator.hpp"
#include "ufil_etsi_ros.hpp"
#include "VehicleCamGeneratorParameterRos.hpp"

namespace ufil_etsi
{

/**
 * @brief ROS 2 component generating the CAMs of a vehicle (ETSI EN 302 637-2).
 *
 * Subscribes:  object (ufil_msgs/ObjectList), the own vehicle at index object_array_index
 * Publishes:   cam (etsi_its_cam_msgs/CAM), /diagnostics
 */
class VehicleCamGeneratorNode : public rclcpp::Node
{
public:
  explicit VehicleCamGeneratorNode(const rclcpp::NodeOptions & options)
  : Node("vehicle_cam_generator_node", options),
    parameters_(loadParameters(*this)),
    generator_(parameters_.config()),
    diagnostics_(this)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    subscription_ = create_subscription<ufil_msgs::msg::ObjectList>(
      "object", 10, [this](const ufil_msgs::msg::ObjectList::ConstSharedPtr & msg) {
        onObjectList(*msg);
      });
    publisher_ = create_publisher<etsi_its_cam_msgs::msg::CAM>("cam", 10);
    // T_CheckCamGen, in ROS time so that the generation follows a simulated clock.
    timer_ = rclcpp::create_timer(
      this, get_clock(), VehicleCamGenerator::kCheckInterval, [this] {onTimer();});

    diagnostics_.setHardwareID("etsi");
    diagnostics_.add("Vehicle CAM generation", this, &VehicleCamGeneratorNode::produceDiagnostics);

    RCLCPP_INFO(
      get_logger(), "Generating CAMs for object %zu of '%s' in frame %s (vehicle role %u).",
      parameters_.settings().object_array_index, subscription_->get_topic_name(),
      parameters_.settings().target_frame.c_str(),
      static_cast<unsigned>(parameters_.config().vehicle_role));
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static VehicleCamGeneratorParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return VehicleCamGeneratorParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onObjectList(const ufil_msgs::msg::ObjectList & msg)
  {
    const auto index = parameters_.settings().object_array_index;
    if (index >= msg.objects.size()) {
      ++invalid_inputs_;
      generator_.clearObject();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring object list with %zu objects, object_array_index is %zu.", msg.objects.size(),
        index);
      return;
    }

    ufil_msgs::msg::ObjectList transformed;
    try {
      transformed = tf_buffer_->transform(msg, parameters_.settings().target_frame);
    } catch (const tf2::TransformException & ex) {
      ++transform_failures_;
      generator_.clearObject();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring objects: %s", ex.what());
      return;
    }
    transform_failures_ = 0;
    invalid_inputs_ = 0;
    generator_.setObject(
      conversions::objectFromMsg(transformed.objects[index]),
      conversions::timeFromMsg(msg.header.stamp), conversions::timeFromRos(now()));
  }

  void onTimer()
  {
    const auto result = generator_.check(conversions::timeFromRos(now()));
    switch (result.event) {
      case GenerationEvent::kGenerated:
        publisher_->publish(conversions::camToMsg(*result.cam));
        if (generator_.status().cams_generated == 1) {
          RCLCPP_INFO(get_logger(), "Sending CAMs of station %u.", result.cam->station_id);
        }
        RCLCPP_DEBUG(
          get_logger(), "Generated CAM (trigger: %s).",
          result.trigger == GenerationTrigger::kDynamics ? "dynamics" :
          result.trigger == GenerationTrigger::kTimeout ? "timeout" : "first");
        break;
      case GenerationEvent::kInvalidTime:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping object: its stamp plus time_shift_nanosec is before 2004, the generation "
          "delta time is undefined. Increase time_shift_nanosec.");
        break;
      case GenerationEvent::kInvalidPosition:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping object: its position is not valid in %s.",
          parameters_.settings().target_frame.c_str());
        break;
      case GenerationEvent::kNoObject:
      case GenerationEvent::kNotDue:
        break;
    }
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const auto status = generator_.status();
    const auto now = conversions::timeFromRos(this->now());
    // Without new objects or CAMs for this long, something is wrong.
    const Duration max_age = 2 * VehicleCamGenerator::kGenCamMax;

    const auto age = [&now](const std::optional<Time> & time) {
        return std::chrono::duration<double>(now - *time).count();
      };
    if (transform_failures_ > 0) {
      diagnostic.summaryf(
        DiagnosticStatus::WARN, "Transform into %s unavailable",
        parameters_.settings().target_frame.c_str());
    } else if (invalid_inputs_ > 0) {
      diagnostic.summary(DiagnosticStatus::WARN, "Own vehicle missing in the object lists");
    } else if (!status.last_object_time) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for objects");
    } else if (now - *status.last_object_time > max_age) {
      diagnostic.summary(DiagnosticStatus::WARN, "No recent objects");
    } else if (!status.last_generation_time || now - *status.last_generation_time > max_age) {
      diagnostic.summary(DiagnosticStatus::ERROR, "Objects cannot be encoded as CAM");
    } else {
      diagnostic.summary(DiagnosticStatus::OK, "Generating CAMs");
    }

    if (status.last_object_time) {
      diagnostic.add("Object age [s]", age(status.last_object_time));
    }
    if (status.last_generation_time) {
      diagnostic.add("Last CAM age [s]", age(status.last_generation_time));
    }
    diagnostic.add("Objects received", status.objects_received);
    diagnostic.add("CAMs generated", status.cams_generated);
    diagnostic.add("Triggered by dynamics", status.triggered_by_dynamics);
    diagnostic.add("Triggered by timeout", status.triggered_by_timeout);
    diagnostic.add("Low frequency containers", status.low_frequency_containers);
    diagnostic.add("Path history points", status.path_history_size);
    diagnostic.add(
      "T_GenCam [s]", std::chrono::duration<double>(status.generation_interval).count());
    diagnostic.add("Dropped (time before 2004)", status.invalid_time);
    diagnostic.add("Dropped (invalid position)", status.invalid_position);
    diagnostic.add("Consecutive transform failures", transform_failures_);
  }

  VehicleCamGeneratorParameterRos parameters_;
  VehicleCamGenerator generator_;
  // Transformations and object selection are done by the node, so their state is tracked here.
  std::uint64_t transform_failures_ = 0;
  std::uint64_t invalid_inputs_ = 0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr subscription_;
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_etsi

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_etsi::VehicleCamGeneratorNode)
