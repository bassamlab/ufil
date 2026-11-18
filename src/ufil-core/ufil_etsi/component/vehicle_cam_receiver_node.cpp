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
#include "ufil_etsi/vehicle_cam_receiver.hpp"
#include "ufil_etsi_ros.hpp"
#include "VehicleCamReceiverParameterRos.hpp"

namespace ufil_etsi
{

/**
 * @brief ROS 2 component converting received vehicle CAMs into UFIL objects.
 *
 * Every vehicle CAM is published as an object list with one object, transformed into
 * target_frame. CAMs of other stations (pedestrians, road side units, ...) are ignored.
 *
 * Subscribes:  cam (etsi_its_cam_msgs/CAM)
 * Publishes:   object_list (ufil_msgs/ObjectList), /diagnostics
 */
class VehicleCamReceiverNode : public rclcpp::Node
{
public:
  explicit VehicleCamReceiverNode(const rclcpp::NodeOptions & options)
  : Node("vehicle_cam_receiver_node", options),
    parameters_(loadParameters(*this)),
    receiver_(parameters_.config()),
    diagnostics_(this)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    subscription_ = create_subscription<etsi_its_cam_msgs::msg::CAM>(
      "cam", 10, [this](const etsi_its_cam_msgs::msg::CAM::ConstSharedPtr & msg) {onCam(*msg);});
    publisher_ = create_publisher<ufil_msgs::msg::ObjectList>("object_list", 10);

    diagnostics_.setHardwareID("etsi");
    diagnostics_.add("Vehicle CAM reception", this, &VehicleCamReceiverNode::produceDiagnostics);

    RCLCPP_INFO(
      get_logger(), "Publishing the vehicles of the CAMs on '%s' in frame %s.",
      subscription_->get_topic_name(), parameters_.settings().target_frame.c_str());
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static VehicleCamReceiverParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return VehicleCamReceiverParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onCam(const etsi_its_cam_msgs::msg::CAM & msg)
  {
    const auto result =
      receiver_.receive(conversions::camFromMsg(msg), conversions::timeFromRos(now()));
    switch (result.event) {
      case ReceptionEvent::kNotAVehicle:
        RCLCPP_DEBUG(
          get_logger(), "Ignoring CAM of station %u with station type %u.",
          msg.header.station_id.value,
          static_cast<unsigned>(msg.cam.cam_parameters.basic_container.station_type.value));
        return;
      case ReceptionEvent::kPositionUnavailable:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping CAM of station %u: reference position unavailable or invalid.",
          msg.header.station_id.value);
        return;
      case ReceptionEvent::kInvalidTime:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping CAM of station %u: the ROS time plus time_shift_nanosec is before 2004, "
          "the generation delta time cannot be resolved. Increase time_shift_nanosec.",
          msg.header.station_id.value);
        return;
      case ReceptionEvent::kConverted:
        break;
    }

    ufil_msgs::msg::ObjectList object_list;
    object_list.header.stamp = conversions::timeToMsg(result.stamp);
    object_list.header.frame_id = frameFromUtmZone(result.zone);
    object_list.objects.push_back(conversions::objectToMsg(result.object));

    ufil_msgs::msg::ObjectList transformed;
    try {
      transformed = tf_buffer_->transform(object_list, parameters_.settings().target_frame);
    } catch (const tf2::TransformException & ex) {
      ++transform_failures_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Dropping CAM of station %u: %s",
        msg.header.station_id.value, ex.what());
      return;
    }
    transform_failures_ = 0;
    publisher_->publish(transformed);
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const auto & status = receiver_.status();

    if (!status.last_reception_time) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for CAMs");
    } else if (transform_failures_ > 0) {
      diagnostic.summaryf(
        DiagnosticStatus::WARN, "Transform into %s unavailable",
        parameters_.settings().target_frame.c_str());
    } else {
      diagnostic.summary(DiagnosticStatus::OK, "Receiving CAMs");
    }

    if (status.last_reception_time) {
      diagnostic.add(
        "Last CAM age [s]", std::chrono::duration<double>(
          conversions::timeFromRos(now()) - *status.last_reception_time).count());
    }
    diagnostic.add("CAMs received", status.received);
    diagnostic.add("Vehicles converted", status.converted);
    diagnostic.add("Ignored (not a vehicle)", status.not_a_vehicle);
    diagnostic.add("Dropped (position unavailable)", status.position_unavailable);
    diagnostic.add("Dropped (time before 2004)", status.invalid_time);
    diagnostic.add("Consecutive transform failures", transform_failures_);
  }

  VehicleCamReceiverParameterRos parameters_;
  VehicleCamReceiver receiver_;
  // Transformations are done by the node, so their state is tracked here.
  std::uint64_t transform_failures_ = 0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<etsi_its_cam_msgs::msg::CAM>::SharedPtr subscription_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_;

  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_etsi

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_etsi::VehicleCamReceiverNode)
