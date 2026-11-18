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

#ifndef UFIL_OSC__COMPONENT__UFILOSCPARAMETERROS_HPP_
#define UFIL_OSC__COMPONENT__UFILOSCPARAMETERROS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ufil_osc/config.hpp"

namespace ufil_osc
{

/// ROS 2 message type received on the input topic.
enum class MessageType
{
  kObjectList,
  kOccupancyGrid,
};

/// "object_list" or "occupancy_grid".
inline const char * toString(MessageType type)
{
  return type == MessageType::kObjectList ? "object_list" : "occupancy_grid";
}

/**
 * @brief Declares, validates and reads the parameters of the OSC message converter node.
 *
 * All parameters are read-only: changing the receiver or the message type requires restarting the
 * node.
 */
class UfilOscParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit UfilOscParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    const std::string message_type = declare<std::string>(
      "message_type", toString(message_type_),
      "ROS 2 message type on the input topic: 'object_list' (ufil_msgs/msg/ObjectList) or "
      "'occupancy_grid' (nav_msgs/msg/OccupancyGrid).");
    if (message_type == "object_list") {
      message_type_ = MessageType::kObjectList;
    } else if (message_type == "occupancy_grid") {
      message_type_ = MessageType::kOccupancyGrid;
    } else {
      throw std::invalid_argument(
              "invalid parameters: message_type must be 'object_list' or 'occupancy_grid', got '" +
              message_type + "'");
    }

    const OscConfig defaults;
    config_.host = declare(
      "receiver.host", defaults.host, "Host name or IP address of the OSC receiver.");
    const auto port = declare<std::int64_t>(
      "receiver.port", defaults.port, "UDP port of the OSC receiver.");
    if (port < 1 || port > 65535) {
      throw std::invalid_argument("invalid parameters: receiver.port must be in [1, 65535]");
    }
    config_.port = static_cast<int>(port);
    config_.address = declare(
      "osc_address", defaults.address,
      "OSC address of the messages; their single string argument is the JSON encoded message.");

    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
  }

  MessageType messageType() const {return message_type_;}
  const OscConfig & config() const {return config_;}

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
  MessageType message_type_ = MessageType::kObjectList;
  OscConfig config_;
};

}  // namespace ufil_osc

#endif  // UFIL_OSC__COMPONENT__UFILOSCPARAMETERROS_HPP_
