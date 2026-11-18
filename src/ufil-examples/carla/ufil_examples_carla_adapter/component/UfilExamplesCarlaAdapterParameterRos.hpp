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

#ifndef CARLA__UFIL_EXAMPLES_CARLA_ADAPTER__COMPONENT__UFILEXAMPLESCARLAADAPTERPARAMETERROS_HPP_
#define CARLA__UFIL_EXAMPLES_CARLA_ADAPTER__COMPONENT__UFILEXAMPLESCARLAADAPTERPARAMETERROS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ufil_examples_carla_adapter/carla_adapter.hpp"

namespace ufil_examples_carla_adapter
{

/**
 * @brief Declares, validates and reads the parameters of the CARLA adapter.
 *
 * All parameters are read-only: changing them requires restarting the node.
 */
class UfilExamplesCarlaAdapterParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit UfilExamplesCarlaAdapterParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    target_frame_ = declare<std::string>(
      "target_frame", "map",
      "Frame of the published object lists. The objects are transformed from the frame of the "
      "simulator objects with the latest available transform.");
    if (target_frame_.empty()) {
      throw std::invalid_argument("invalid parameters: target_frame must not be empty");
    }

    config_.filter_cooperative_vehicles = declare(
      "cooperative_vehicles.enabled", false,
      "Publish only the cooperative vehicles listed in cooperative_vehicles.ids. If false, every "
      "simulator object is published and cooperative_vehicles.ids is not declared.");
    if (config_.filter_cooperative_vehicles) {
      // A ROS 2 parameter file cannot hold an empty array, so the IDs only exist if enabled.
      config_.cooperative_vehicle_ids = declare(
        "cooperative_vehicles.ids", std::vector<std::int64_t>{},
        "IDs of the cooperative vehicles, i.e. the CARLA actors with the role name "
        "'vehicle_<id>'. Non-negative and unique.");
    }

    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: cooperative_vehicles: " + *error);
    }
  }

  const std::string & targetFrame() const {return target_frame_;}
  const CarlaAdapterConfig & config() const {return config_;}

private:
  template<typename T>
  T declare(const std::string & name, const T & default_value, const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return parameters_->declare_parameter(
      name, rclcpp::ParameterValue(default_value), descriptor).template get<T>();
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  std::string target_frame_;
  CarlaAdapterConfig config_;
};

}  // namespace ufil_examples_carla_adapter

#endif  // CARLA__UFIL_EXAMPLES_CARLA_ADAPTER__COMPONENT__UFILEXAMPLESCARLAADAPTERPARAMETERROS_HPP_
