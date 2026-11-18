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

#ifndef UFIL_ETSI__COMPONENT__UFILETSIPARAMETERROS_HPP_
#define UFIL_ETSI__COMPONENT__UFILETSIPARAMETERROS_HPP_

// Shared helper of the parameter classes of the ufil_etsi components.

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ufil_etsi/types.hpp"

namespace ufil_etsi
{

/// Declares read-only parameters: changing them requires restarting the node.
class ParameterDeclarer
{
public:
  explicit ParameterDeclarer(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters)) {}

  template<typename T>
  T declare(const std::string & name, const T & default_value, const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return parameters_->declare_parameter(
      name, rclcpp::ParameterValue(default_value), descriptor).get<T>();
  }

  /// Declares time_shift_nanosec, see VehicleCamGeneratorConfig::time_shift.
  Duration declareTimeShift()
  {
    return Duration(
      declare<std::int64_t>(
        "time_shift_nanosec", 0,
        "Added to the ROS time for the ETSI timestamps (generation delta time) [ns]. ITS time "
        "starts in 2004, clocks before 2004 (e.g. simulations starting at 0) need a shift. "
        "Sender and receiver must use the same value."));
  }

private:
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
};

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__COMPONENT__UFILETSIPARAMETERROS_HPP_
