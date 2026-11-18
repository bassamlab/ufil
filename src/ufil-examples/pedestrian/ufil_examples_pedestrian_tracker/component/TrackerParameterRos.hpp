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

#ifndef PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_TRACKER__COMPONENT__TRACKERPARAMETERROS_HPP_
#define PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_TRACKER__COMPONENT__TRACKERPARAMETERROS_HPP_

#include <stdexcept>
#include <string>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ufil_examples_pedestrian_tracker/pedestrian_tracker.hpp"

namespace ufil_examples_pedestrian_tracker
{

/**
 * @brief Declares, validates and reads the parameters of the pedestrian tracker node.
 *
 * All parameters are read-only: changing the tracker requires restarting the node.
 */
class TrackerParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit TrackerParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    const TrackerConfig defaults;
    config_.process_noise = declare(
      "process_noise", defaults.process_noise,
      "Process noise of the random walk motion model, per axis.");
    config_.track_timeout = declare(
      "track_timeout", defaults.track_timeout,
      "Tracks without an associated detection for this long are deleted [s].");
    config_.max_time_gap = declare(
      "max_time_gap", defaults.max_time_gap,
      "A larger gap between two detection stamps, or a jump back in time, resets the tracker "
      "[s].");

    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
  }

  const TrackerConfig & config() const {return config_;}

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
  TrackerConfig config_;
};

}  // namespace ufil_examples_pedestrian_tracker

#endif  // PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_TRACKER__COMPONENT__TRACKERPARAMETERROS_HPP_
