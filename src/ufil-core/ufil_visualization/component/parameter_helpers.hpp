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

#ifndef UFIL_VISUALIZATION__COMPONENT__PARAMETER_HELPERS_HPP_
#define UFIL_VISUALIZATION__COMPONENT__PARAMETER_HELPERS_HPP_

// Helpers shared by the parameter classes of the ufil_visualization components.

#include <stdexcept>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ufil_visualization/marker.hpp"

namespace ufil_visualization::parameters
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

/**
 * @brief Declares a read-only colour parameter given as [r, g, b, a].
 * @throws std::invalid_argument if the value does not have four entries. The value range is
 *         checked by the validation of the core configuration.
 */
inline Color declareColor(
  rclcpp::node_interfaces::NodeParametersInterface & parameters, const std::string & name,
  const Color & default_value, const std::string & description)
{
  const auto values = declare(
    parameters, name,
    std::vector<double>{default_value.r, default_value.g, default_value.b, default_value.a},
    description + " [r, g, b, a], each in [0, 1].");
  if (values.size() != 4) {
    throw std::invalid_argument(
            "invalid parameters: " + name + " must have four entries [r, g, b, a]");
  }
  return {
    static_cast<float>(values[0]), static_cast<float>(values[1]),
    static_cast<float>(values[2]), static_cast<float>(values[3])};
}

}  // namespace ufil_visualization::parameters

#endif  // UFIL_VISUALIZATION__COMPONENT__PARAMETER_HELPERS_HPP_
