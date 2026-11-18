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

#include <memory>
#include <stdexcept>
#include <string>

#include <cpm_lab_map_msgs/msg/lane_control_light.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ufil_osc/osc_sender.hpp>

#include "ufil_examples_cpm_lab_adapter/lane_control_light.hpp"
#include "OscLaneControlLightParameterRos.hpp"
#include "ufil_examples_cpm_lab_adapter_ros.hpp"

namespace ufil_examples_cpm_lab_adapter
{

/**
 * @brief ROS 2 component forwarding lane control light states as JSON in OSC messages over UDP.
 *
 * Subscribes:  lane_control_lights (cpm_lab_map_msgs/LaneControlLight, reliable)
 * Publishes:   /diagnostics
 *
 * Every received message is sent as one OSC message to osc_address with the JSON encoded state
 * (see toJson()) as its single string argument.
 */
class OscLaneControlLightNode : public rclcpp::Node
{
public:
  explicit OscLaneControlLightNode(const rclcpp::NodeOptions & options)
  : Node("osc_lane_control_light", options),
    parameters_(loadParameters(*this)),
    sender_(createSender(*this, parameters_.config())),
    diagnostics_(this)
  {
    // Lane control light states are rare events: subscribe reliably.
    subscription_ = create_subscription<cpm_lab_map_msgs::msg::LaneControlLight>(
      "lane_control_lights", rclcpp::QoS(10),
      [this](const cpm_lab_map_msgs::msg::LaneControlLight::ConstSharedPtr & msg) {
        forward(toJson(conversions::laneControlLightFromMsg(*msg)));
      });

    RCLCPP_INFO(
      get_logger(), "Forwarding lane control lights from '%s' to OSC address '%s' at %s.",
      subscription_->get_topic_name(), sender_->config().address.c_str(),
      sender_->endpoint().c_str());

    diagnostics_.setHardwareID(sender_->endpoint());
    diagnostics_.add("OSC sender", this, &OscLaneControlLightNode::produceDiagnostics);
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static OscLaneControlLightParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return OscLaneControlLightParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  /// Logs a receiver that cannot be resolved before construction fails.
  static std::unique_ptr<ufil_osc::OscSender> createSender(
    rclcpp::Node & node, const ufil_osc::OscConfig & config)
  {
    try {
      return std::make_unique<ufil_osc::OscSender>(config);
    } catch (const std::exception & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void forward(const std::string & payload)
  {
    switch (sender_->send(payload)) {
      case ufil_osc::SendResult::kSent:
        RCLCPP_DEBUG(
          get_logger(), "Sent %zu bytes to %s.", sender_->status().last_packet_size,
          sender_->endpoint().c_str());
        break;
      case ufil_osc::SendResult::kTooLarge:
        RCLCPP_WARN_THROTTLE(
          get_logger(), steady_clock_, 5000, "Dropping lane control lights: %s.",
          sender_->status().last_error.c_str());
        break;
      case ufil_osc::SendResult::kFailed:
        RCLCPP_ERROR_THROTTLE(
          get_logger(), steady_clock_, 5000, "Sending to %s failed: %s.",
          sender_->endpoint().c_str(), sender_->status().last_error.c_str());
        break;
    }
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    using ufil_osc::SendResult;
    const ufil_osc::SenderStatus status = sender_->status();
    if (!status.last_result) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for lane control lights");
    } else if (*status.last_result == SendResult::kSent) {
      diagnostic.summary(DiagnosticStatus::OK, "Sending");
    } else if (*status.last_result == SendResult::kTooLarge) {
      diagnostic.summaryf(
        DiagnosticStatus::WARN, "Dropping messages: %s", status.last_error.c_str());
    } else {
      diagnostic.summaryf(DiagnosticStatus::ERROR, "Sending failed: %s", status.last_error.c_str());
    }
    diagnostic.add("Receiver", sender_->endpoint());
    diagnostic.add("OSC address", sender_->config().address);
    diagnostic.add("Input topic", subscription_->get_topic_name());
    diagnostic.add("Messages sent", status.sent);
    diagnostic.add("Messages too large", status.too_large);
    diagnostic.add("Messages failed", status.failed);
    diagnostic.add("Last packet size [bytes]", status.last_packet_size);
    diagnostic.add("Last error", status.last_error);
  }

  OscLaneControlLightParameterRos parameters_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::unique_ptr<ufil_osc::OscSender> sender_;
  rclcpp::Subscription<cpm_lab_map_msgs::msg::LaneControlLight>::SharedPtr subscription_;
  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_examples_cpm_lab_adapter

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_examples_cpm_lab_adapter::OscLaneControlLightNode)
