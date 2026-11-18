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
#include <exception>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_mqtt/mqtt_client.hpp"
#include "UfilMqttParameterRos.hpp"
#include "ufil_mqtt_ros.hpp"

namespace ufil_mqtt
{

/**
 * @brief ROS 2 component forwarding ROS 2 messages as JSON to an MQTT broker.
 *
 * Subscribes:  input (ufil_msgs/ObjectList or nav_msgs/OccupancyGrid, see message_type)
 * Publishes:   /diagnostics
 *
 * Messages received while the broker is not connected are dropped. The connection is retried every
 * broker.reconnect_interval seconds.
 */
class MqttMessageConverterNode : public rclcpp::Node
{
public:
  explicit MqttMessageConverterNode(const rclcpp::NodeOptions & options)
  : Node("mqtt_message_converter", options),
    parameters_(loadParameters(*this)),
    client_(parameters_.config(), [this](const MqttEvent & event) {onEvent(event);}),
    diagnostics_(this)
  {
    const auto & settings = parameters_.settings();
    switch (settings.message_type) {
      case MessageType::kObjectList:
        subscription_ = subscribe<ufil_msgs::msg::ObjectList>();
        break;
      case MessageType::kOccupancyGrid:
        subscription_ = subscribe<nav_msgs::msg::OccupancyGrid>();
        break;
    }

    RCLCPP_INFO(
      get_logger(), "Forwarding %s messages from '%s' to MQTT topic '%s' on %s as client '%s'.",
      toString(settings.message_type), subscription_->get_topic_name(),
      client_.config().topic.c_str(), serverUri(client_.config()).c_str(),
      client_.config().client_id.c_str());

    // The connection is retried in wall time, independent of a simulated clock.
    reconnect_timer_ = create_wall_timer(
      std::chrono::duration<double>(settings.reconnect_interval), [this] {client_.connect();});
    client_.connect();

    diagnostics_.setHardwareID(serverUri(client_.config()));
    diagnostics_.add("MQTT connection", this, &MqttMessageConverterNode::produceDiagnostics);
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static UfilMqttParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return UfilMqttParameterRos(
        node.get_node_parameters_interface(), node.get_fully_qualified_name());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  template<typename MsgT>
  rclcpp::SubscriptionBase::SharedPtr subscribe()
  {
    // Best effort: compatible with reliable and best effort publishers. Forwarding over the
    // network is lossy anyway.
    return create_subscription<MsgT>(
      "input", rclcpp::SensorDataQoS(),
      [this](const typename MsgT::ConstSharedPtr & msg) {
        forward(conversions::payloadFromMsg(*msg));
      });
  }

  void forward(const std::string & payload)
  {
    switch (client_.publish(payload)) {
      case PublishResult::kQueued:
        RCLCPP_DEBUG(get_logger(), "Published %zu bytes.", payload.size());
        break;
      case PublishResult::kNotConnected:
        RCLCPP_DEBUG(get_logger(), "Dropped a message, not connected to the broker.");
        break;
      case PublishResult::kFailed:
        // Logged by onEvent.
        break;
    }
  }

  /// Called from the thread of the MQTT client.
  void onEvent(const MqttEvent & event)
  {
    const std::string uri = serverUri(parameters_.config());
    switch (event.type) {
      case MqttEvent::Type::kConnected:
        RCLCPP_INFO(get_logger(), "Connected to MQTT broker %s.", uri.c_str());
        break;
      case MqttEvent::Type::kConnectFailed:
        if (event.attempt == 1) {
          RCLCPP_WARN(
            get_logger(), "Cannot connect to MQTT broker %s: %s. Retrying every %.1f s.",
            uri.c_str(), event.reason.c_str(), parameters_.settings().reconnect_interval);
        } else {
          RCLCPP_DEBUG(
            get_logger(), "Connection attempt %s to MQTT broker %s failed: %s.",
            std::to_string(event.attempt).c_str(), uri.c_str(), event.reason.c_str());
        }
        break;
      case MqttEvent::Type::kConnectionLost:
        RCLCPP_WARN(
          get_logger(), "Lost connection to MQTT broker %s: %s. Reconnecting.", uri.c_str(),
          event.reason.c_str());
        break;
      case MqttEvent::Type::kPublishFailed:
        RCLCPP_WARN_THROTTLE(
          get_logger(), steady_clock_, 5000, "Publishing on MQTT topic '%s' failed: %s.",
          parameters_.config().topic.c_str(), event.reason.c_str());
        break;
    }
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const MqttStatus status = client_.status();
    if (status.state == ConnectionState::kConnected) {
      diagnostic.summary(DiagnosticStatus::OK, "Connected");
    } else if (status.consecutive_connect_failures == 0 && status.connection_losses == 0) {
      diagnostic.summary(DiagnosticStatus::WARN, "Connecting");
    } else {
      diagnostic.summaryf(DiagnosticStatus::ERROR, "Not connected: %s", status.last_error.c_str());
    }

    diagnostic.add("Broker", serverUri(client_.config()));
    diagnostic.add("Client ID", client_.config().client_id);
    diagnostic.add("MQTT topic", client_.config().topic);
    diagnostic.add("Input topic", subscription_->get_topic_name());
    diagnostic.add("Message type", toString(parameters_.settings().message_type));
    diagnostic.add("Connection state", toString(status.state));
    diagnostic.add("Connection attempts", status.connect_attempts);
    diagnostic.add("Connection losses", status.connection_losses);
    diagnostic.add("Messages sent", status.sent);
    diagnostic.add("Messages dropped (not connected)", status.dropped);
    diagnostic.add("Messages failed", status.failed);
    diagnostic.add("Last error", status.last_error);
  }

  UfilMqttParameterRos parameters_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  MqttClient client_;
  rclcpp::SubscriptionBase::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_mqtt

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_mqtt::MqttMessageConverterNode)
