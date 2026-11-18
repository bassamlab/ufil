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

#ifndef UFIL_UDP__COMPONENT__UFILUDPSENDERPARAMETERROS_HPP_
#define UFIL_UDP__COMPONENT__UFILUDPSENDERPARAMETERROS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ufil_udp/config.hpp"

namespace ufil_udp
{

/**
 * @brief Declares, validates and reads the parameters of the UDP sender node.
 *
 * All parameters are read-only: changing the target requires restarting the node.
 */
class UfilUdpSenderParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit UfilUdpSenderParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    const SenderConfig defaults;
    config_.target_address = declare(
      "target_address", defaults.target_address,
      "IPv4 or IPv6 address the packets are sent to (unicast or broadcast, no host name).");
    const auto target_port = declare<std::int64_t>(
      "target_port", defaults.target_port, "UDP port the packets are sent to, 1 to 65535.");

    if (target_port < 1 || target_port > 65535) {
      throw std::invalid_argument("invalid parameters: target_port must be in [1, 65535]");
    }
    config_.target_port = static_cast<std::uint16_t>(target_port);
    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
  }

  const SenderConfig & config() const {return config_;}

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
  SenderConfig config_;
};

}  // namespace ufil_udp

#endif  // UFIL_UDP__COMPONENT__UFILUDPSENDERPARAMETERROS_HPP_
