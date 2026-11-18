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

#ifndef UFIL_MESSAGE_FILTER__COMPONENT__UFILMESSAGEFILTERPARAMETERROS_HPP_
#define UFIL_MESSAGE_FILTER__COMPONENT__UFILMESSAGEFILTERPARAMETERROS_HPP_

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ufil_message_filter/position_filter.hpp"

namespace ufil_message_filter
{

/**
 * @brief Declares, validates and reads the parameters of the object list position filter.
 *
 * All parameters are read-only: changing the area of interest requires restarting the node.
 */
class UfilMessageFilterParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit UfilMessageFilterParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    const bool enabled = declare(
      "area_of_interest.enabled", false,
      "Forward only the objects inside area_of_interest.vertices. If false, every object is "
      "forwarded and area_of_interest.vertices is not declared.");
    if (enabled) {
      // A ROS 2 parameter file cannot hold an empty array, so the vertices only exist if enabled.
      config_.area_of_interest = areaFromParameter(
        declare(
          "area_of_interest.vertices", std::vector<double>{},
          "Vertices of the area of interest polygon as [x1, y1, x2, y2, ...] [m] in the frame of "
          "the input object lists, at least 3 vertices. Objects on the boundary are filtered "
          "out."));
      if (config_.area_of_interest.empty()) {
        throw std::invalid_argument(
                "invalid parameters: area_of_interest.vertices must be set if "
                "area_of_interest.enabled is true");
      }
    }

    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: area_of_interest.vertices: " + *error);
    }
  }

  const PositionFilterConfig & config() const {return config_;}

  /// Converts the flat [x1, y1, x2, y2, ...] parameter into vertices.
  static std::vector<Position> areaFromParameter(const std::vector<double> & values)
  {
    if (values.size() % 2 != 0) {
      throw std::invalid_argument(
              "invalid parameters: area_of_interest.vertices must contain an even number of values "
              "(x1, y1, x2, y2, ...), got " + std::to_string(values.size()));
    }
    std::vector<Position> vertices;
    for (std::size_t i = 0; i < values.size(); i += 2) {
      vertices.push_back(Position{values[i], values[i + 1]});
    }
    return vertices;
  }

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
  PositionFilterConfig config_;
};

}  // namespace ufil_message_filter

#endif  // UFIL_MESSAGE_FILTER__COMPONENT__UFILMESSAGEFILTERPARAMETERROS_HPP_
