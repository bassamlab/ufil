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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_osc/osc_sender.hpp"
#include "UfilOscParameterRos.hpp"
#include "ufil_osc_ros.hpp"

namespace ufil_osc
{

/**
 * @brief ROS 2 component forwarding ROS 2 messages as JSON in OSC messages over UDP.
 *
 * Subscribes:  input (ufil_msgs/ObjectList or nav_msgs/OccupancyGrid, see message_type)
 * Publishes:   /diagnostics
 *
 * Every received message is sent as one OSC message to osc_address with the JSON encoded message
 * as its single string argument. Messages that do not fit into one UDP datagram are dropped.
 */
class OscMessageConverterNode : public rclcpp::Node
{
public:
  explicit OscMessageConverterNode(const rclcpp::NodeOptions & options)
  : Node("osc_message_converter", options),
    parameters_(loadParameters(*this)),
    sender_(createSender(*this, parameters_.config())),
    diagnostics_(this)
  {
    switch (parameters_.messageType()) {
      case MessageType::kObjectList:
        subscription_ = subscribe<ufil_msgs::msg::ObjectList>();
        break;
      case MessageType::kOccupancyGrid:
        subscription_ = subscribe<nav_msgs::msg::OccupancyGrid>();
        break;
    }

    RCLCPP_INFO(
      get_logger(), "Forwarding %s messages from '%s' to OSC address '%s' at %s.",
      toString(parameters_.messageType()), subscription_->get_topic_name(),
      sender_->config().address.c_str(), sender_->endpoint().c_str());

    diagnostics_.setHardwareID(sender_->endpoint());
    diagnostics_.add("OSC sender", this, &OscMessageConverterNode::produceDiagnostics);
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static UfilOscParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return UfilOscParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  /// Logs a receiver that cannot be resolved before construction fails.
  static std::unique_ptr<OscSender> createSender(rclcpp::Node & node, const OscConfig & config)
  {
    try {
      return std::make_unique<OscSender>(config);
    } catch (const std::exception & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  template<typename MsgT>
  rclcpp::SubscriptionBase::SharedPtr subscribe()
  {
    // Best effort: compatible with reliable and best effort publishers. UDP is lossy anyway.
    return create_subscription<MsgT>(
      "input", rclcpp::SensorDataQoS(),
      [this](const typename MsgT::ConstSharedPtr & msg) {
        forward(conversions::payloadFromMsg(*msg));
      });
  }

  void forward(const std::string & payload)
  {
    const SendResult result = sender_->send(payload);
    switch (result) {
      case SendResult::kSent:
        RCLCPP_DEBUG(
          get_logger(), "Sent %zu bytes to %s.", sender_->status().last_packet_size,
          sender_->endpoint().c_str());
        break;
      case SendResult::kTooLarge:
        RCLCPP_WARN_THROTTLE(
          get_logger(), steady_clock_, 5000, "Dropping message: %s.",
          sender_->status().last_error.c_str());
        break;
      case SendResult::kFailed:
        RCLCPP_ERROR_THROTTLE(
          get_logger(), steady_clock_, 5000, "Sending to %s failed: %s.",
          sender_->endpoint().c_str(), sender_->status().last_error.c_str());
        break;
    }
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const SenderStatus status = sender_->status();
    if (!status.last_result) {
      diagnostic.summary(DiagnosticStatus::STALE, "Waiting for messages");
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
    diagnostic.add("Message type", toString(parameters_.messageType()));
    diagnostic.add("Messages sent", status.sent);
    diagnostic.add("Messages too large", status.too_large);
    diagnostic.add("Messages failed", status.failed);
    diagnostic.add("Last packet size [bytes]", status.last_packet_size);
    diagnostic.add("Last error", status.last_error);
  }

  UfilOscParameterRos parameters_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::unique_ptr<OscSender> sender_;
  rclcpp::SubscriptionBase::SharedPtr subscription_;
  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_osc

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_osc::OscMessageConverterNode)
