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

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

#include "ufil_udp/udp_sender.hpp"
#include "UfilUdpSenderParameterRos.hpp"
#include "ufil_udp_ros.hpp"

namespace ufil_udp
{

/**
 * @brief ROS 2 component sending the payload of every received message as UDP datagram.
 *
 * Subscribes:  udp_input (udp_msgs/UdpPacket); address and src_port of the message are ignored,
 *              the packets are sent to target_address:target_port
 * Publishes:   /diagnostics
 */
class UdpSenderNode : public rclcpp::Node
{
public:
  explicit UdpSenderNode(const rclcpp::NodeOptions & options)
  : Node("udp_sender", options),
    parameters_(loadParameters(*this)),
    sender_(createSender(*this, parameters_.config())),
    target_(parameters_.config().target_address + ":" +
      std::to_string(parameters_.config().target_port)),
    diagnostics_(this)
  {
    subscription_ = create_subscription<udp_msgs::msg::UdpPacket>(
      "udp_input", rclcpp::QoS(10),
      [this](const udp_msgs::msg::UdpPacket::ConstSharedPtr & msg) {onMessage(*msg);});

    diagnostics_.setHardwareID("udp:" + target_);
    diagnostics_.add("UDP sender", this, &UdpSenderNode::produceDiagnostics);

    RCLCPP_INFO(
      get_logger(), "Sending the packets of '%s' to %s.", subscription_->get_topic_name(),
      target_.c_str());
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static UfilUdpSenderParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return UfilUdpSenderParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  static std::unique_ptr<UdpSender> createSender(rclcpp::Node & node, const SenderConfig & config)
  {
    try {
      return std::make_unique<UdpSender>(config);
    } catch (const std::runtime_error & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onMessage(const udp_msgs::msg::UdpPacket & msg)
  {
    if (const auto error = sender_->send(conversions::payloadFromMsg(msg))) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), steady_clock_, 5000, "Cannot send UDP packet of %zu bytes to %s: %s",
        msg.data.size(), target_.c_str(), error->c_str());
    }
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const SenderStatus status = sender_->status();

    diagnostic.add("Target", target_);
    diagnostic.add("Packets sent", status.packets);
    diagnostic.add("Bytes sent", status.bytes);
    diagnostic.add("Send errors", status.errors);
    if (status.last_packet_time) {
      const std::chrono::duration<double> age =
        std::chrono::steady_clock::now() - *status.last_packet_time;
      diagnostic.add("Last packet age [s]", age.count());
    } else {
      diagnostic.add("Last packet age [s]", "no packet");
    }
    if (!status.last_error.empty()) {
      diagnostic.add("Last error", status.last_error);
    }

    const bool new_errors = status.errors > reported_errors_;
    reported_errors_ = status.errors;
    if (new_errors) {
      diagnostic.summaryf(DiagnosticStatus::WARN, "Send errors: %s", status.last_error.c_str());
    } else if (!status.last_packet_time) {
      diagnostic.summary(DiagnosticStatus::STALE, "No packet sent yet");
    } else {
      diagnostic.summary(DiagnosticStatus::OK, "Sending");
    }
  }

  UfilUdpSenderParameterRos parameters_;
  std::unique_ptr<UdpSender> sender_;
  /// "<address>:<port>" for log messages and diagnostics.
  std::string target_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr subscription_;
  diagnostic_updater::Updater diagnostics_;
  /// Send errors at the last diagnostic update.
  std::uint64_t reported_errors_ = 0;
};

}  // namespace ufil_udp

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_udp::UdpSenderNode)
