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

#ifndef CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__PARAMETER_HELPERS_HPP_
#define CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__PARAMETER_HELPERS_HPP_

// Helpers shared by the parameter classes of the components.

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

namespace ufil_examples_cpm_lab_adapter::parameters
{

/// Declares a read-only parameter and returns its value.
template<typename T>
T declare(
  rclcpp::node_interfaces::NodeParametersInterface & parameters, const std::string & name,
  const T & default_value, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return parameters.declare_parameter(
    name, rclcpp::ParameterValue(default_value), descriptor).get<T>();
}

/// Declares the read-only id of a CPM Lab vehicle, which is in [0, 255].
inline std::uint8_t declareVehicleId(
  rclcpp::node_interfaces::NodeParametersInterface & parameters, std::int64_t default_value)
{
  const auto id = declare<std::int64_t>(
    parameters, "vehicle_id", default_value, "Id of the CPM Lab vehicle, in [0, 255].");
  if (id < 0 || id > 255) {
    throw std::invalid_argument("invalid parameters: vehicle_id must be in [0, 255]");
  }
  return static_cast<std::uint8_t>(id);
}

/// Declares the read-only, non-empty list of distinct CPM Lab vehicle ids, each in [0, 255].
inline std::vector<std::uint8_t> declareVehicleIds(
  rclcpp::node_interfaces::NodeParametersInterface & parameters,
  const std::vector<std::int64_t> & default_value)
{
  const auto ids = declare(
    parameters, "vehicle_ids", default_value, "Ids of the CPM Lab vehicles, each in [0, 255].");
  if (ids.empty()) {
    throw std::invalid_argument("invalid parameters: vehicle_ids must not be empty");
  }
  std::vector<std::uint8_t> result;
  for (const auto id : ids) {
    if (id < 0 || id > 255) {
      throw std::invalid_argument("invalid parameters: vehicle_ids must be in [0, 255]");
    }
    if (std::find(result.begin(), result.end(), id) != result.end()) {
      throw std::invalid_argument(
              "invalid parameters: vehicle_ids contains " + std::to_string(id) + " twice");
    }
    result.push_back(static_cast<std::uint8_t>(id));
  }
  return result;
}

/// Declares a read-only frame id that must not be empty.
inline std::string declareFrame(
  rclcpp::node_interfaces::NodeParametersInterface & parameters, const std::string & name,
  const std::string & default_value, const std::string & description)
{
  auto frame = declare(parameters, name, default_value, description);
  if (frame.empty()) {
    throw std::invalid_argument("invalid parameters: " + name + " must not be empty");
  }
  return frame;
}

/// Declares a read-only rate [Hz] that must be positive.
inline double declareFrequency(
  rclcpp::node_interfaces::NodeParametersInterface & parameters, const std::string & name,
  double default_value, const std::string & description)
{
  const auto frequency = declare(parameters, name, default_value, description);
  if (!(frequency > 0.0) || frequency > 1000.0) {
    throw std::invalid_argument("invalid parameters: " + name + " must be in (0, 1000] Hz");
  }
  return frequency;
}

}  // namespace ufil_examples_cpm_lab_adapter::parameters

#endif  // CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__PARAMETER_HELPERS_HPP_
