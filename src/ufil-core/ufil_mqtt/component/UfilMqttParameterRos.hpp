// Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#ifndef UFIL_MQTT__COMPONENT__UFILMQTTPARAMETERROS_HPP_
#define UFIL_MQTT__COMPONENT__UFILMQTTPARAMETERROS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ufil_mqtt/config.hpp"

namespace ufil_mqtt
{

/// ROS 2 message type received on the input topic.
enum class MessageType
{
  kObjectList,
  kOccupancyGrid,
};

/// Settings only used by the ROS 2 node.
struct NodeSettings
{
  MessageType message_type = MessageType::kObjectList;
  /// Period of the connection attempts while disconnected [s].
  double reconnect_interval = 2.0;
};

/// "object_list" or "occupancy_grid".
inline const char * toString(MessageType type)
{
  return type == MessageType::kObjectList ? "object_list" : "occupancy_grid";
}

/**
 * @brief Declares, validates and reads the parameters of the MQTT message converter node.
 *
 * All parameters are read-only: changing the broker or the message type requires restarting the
 * node. An empty broker.client_id is replaced by the fully qualified node name without the leading
 * slash, so several converters can connect to the same broker.
 */
class UfilMqttParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  UfilMqttParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
    const std::string & fully_qualified_node_name)
  : parameters_(std::move(parameters))
  {
    const std::string message_type = declare<std::string>(
      "message_type", toString(settings_.message_type),
      "ROS 2 message type on the input topic: 'object_list' (ufil_msgs/msg/ObjectList) or "
      "'occupancy_grid' (nav_msgs/msg/OccupancyGrid).");
    if (message_type == "object_list") {
      settings_.message_type = MessageType::kObjectList;
    } else if (message_type == "occupancy_grid") {
      settings_.message_type = MessageType::kOccupancyGrid;
    } else {
      throw std::invalid_argument(
              "invalid parameters: message_type must be 'object_list' or 'occupancy_grid', got '" +
              message_type + "'");
    }

    const MqttConfig defaults;
    config_.protocol = declare(
      "broker.protocol", defaults.protocol,
      "Transport to the broker: 'tcp' or 'ssl' (TLS, broker certificate checked against the "
      "system CAs).");
    config_.host = declare(
      "broker.host", defaults.host, "Host name or IPv4 address of the broker.");
    const auto port = declare<std::int64_t>("broker.port", defaults.port, "Port of the broker.");
    if (port < 1 || port > 65535) {
      throw std::invalid_argument("invalid parameters: broker.port must be in [1, 65535]");
    }
    config_.port = static_cast<int>(port);
    config_.client_id = declare(
      "broker.client_id", std::string(),
      "MQTT client identifier, unique per broker. Empty: the fully qualified node name.");
    if (config_.client_id.empty()) {
      config_.client_id = fully_qualified_node_name.substr(
        fully_qualified_node_name.rfind('/', 0) == 0 ? 1 : 0);
    }
    settings_.reconnect_interval = declare(
      "broker.reconnect_interval", settings_.reconnect_interval,
      "Period of the connection attempts while disconnected [s].");
    config_.topic = declare(
      "mqtt_topic", defaults.topic, "MQTT topic the JSON encoded messages are published on.");

    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
    if (!(settings_.reconnect_interval > 0.0)) {
      throw std::invalid_argument("invalid parameters: broker.reconnect_interval must be > 0");
    }
  }

  const NodeSettings & settings() const {return settings_;}
  const MqttConfig & config() const {return config_;}

private:
  template<typename T>
  T declare(const std::string & name, const T & default_value, const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return parameters_->declare_parameter(
      name, rclcpp::ParameterValue(default_value), descriptor).get<T>();
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  NodeSettings settings_;
  MqttConfig config_;
};

}  // namespace ufil_mqtt

#endif  // UFIL_MQTT__COMPONENT__UFILMQTTPARAMETERROS_HPP_
