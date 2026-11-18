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
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

#include "ufil_udp/udp_receiver.hpp"
#include "UfilUdpReceiverParameterRos.hpp"
#include "ufil_udp_ros.hpp"

namespace ufil_udp
{

/**
 * @brief ROS 2 component publishing the UDP datagrams received on a port.
 *
 * Publishes:  udp_output (udp_msgs/UdpPacket), stamped with the reception time, /diagnostics
 */
class UdpReceiverNode : public rclcpp::Node
{
public:
  explicit UdpReceiverNode(const rclcpp::NodeOptions & options)
  : Node("udp_receiver", options),
    parameters_(loadParameters(*this)),
    diagnostics_(this)
  {
    publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("udp_output", rclcpp::QoS(10));

    try {
      receiver_ = std::make_unique<UdpReceiver>(
        parameters_.config(),
        [this](Packet && packet) {onPacket(std::move(packet));},
        [this](const std::string & error) {onError(error);});
    } catch (const std::runtime_error & error) {
      RCLCPP_FATAL(get_logger(), "%s", error.what());
      throw;
    }

    const auto port = std::to_string(receiver_->localPort());
    diagnostics_.setHardwareID("udp:" + port);
    diagnostics_.add("UDP receiver", this, &UdpReceiverNode::produceDiagnostics);

    RCLCPP_INFO(
      get_logger(), "Receiving UDP packets on port %s, publishing them on '%s'.", port.c_str(),
      publisher_->get_topic_name());
  }

  ~UdpReceiverNode() override
  {
    // The receive thread publishes through this node: stop it before any member is destroyed.
    receiver_.reset();
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static UfilUdpReceiverParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return UfilUdpReceiverParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  /// Called from the receive thread.
  void onPacket(Packet && packet)
  {
    publisher_->publish(conversions::packetToMsg(std::move(packet), now()));
  }

  /// Called from the receive thread.
  void onError(const std::string & error)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), steady_clock_, 5000, "UDP receive error on port %u: %s",
      static_cast<unsigned>(parameters_.config().listen_port), error.c_str());
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const ReceiverStatus status = receiver_->status();

    diagnostic.add("Port", status.local_port);
    diagnostic.add("Packets received", status.packets);
    diagnostic.add("Bytes received", status.bytes);
    diagnostic.add("Receive errors", status.errors);
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
    if (!status.receiving) {
      diagnostic.summary(DiagnosticStatus::ERROR, "Not receiving");
    } else if (new_errors) {
      diagnostic.summaryf(DiagnosticStatus::WARN, "Receive errors: %s", status.last_error.c_str());
    } else if (!status.last_packet_time) {
      diagnostic.summary(DiagnosticStatus::STALE, "No packet received yet");
    } else {
      diagnostic.summary(DiagnosticStatus::OK, "Receiving");
    }
  }

  UfilUdpReceiverParameterRos parameters_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr publisher_;
  diagnostic_updater::Updater diagnostics_;
  /// Receive errors at the last diagnostic update.
  std::uint64_t reported_errors_ = 0;
  std::unique_ptr<UdpReceiver> receiver_;
};

}  // namespace ufil_udp

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_udp::UdpReceiverNode)
