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

#ifndef SSL_SIMULATOR__COMPONENT__SSLSIMULATORPARAMETERROS_HPP_
#define SSL_SIMULATOR__COMPONENT__SSLSIMULATORPARAMETERROS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

#include "ssl_simulator/config.hpp"

namespace ssl_simulator
{

/// Settings only used by the ROS 2 node.
struct NodeSettings
{
  /// Index of the mat; names the topic ssl_<node_id>/map and the frame ssl_frame_<node_id>.
  std::int64_t node_id = 1;
  /// Period of the simulated frames [s]; the hardware delivers a frame about every 23 ms.
  double frame_period = 0.023;
};

/**
 * @brief Declares, validates and reads the parameters of the virtual SSL node.
 *
 * All parameters are read-only: changing the mat requires restarting the node.
 */
class SslSimulatorParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit SslSimulatorParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    declareSettings();
    declareMat();
    declareSimulation();

    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
  }

  const NodeSettings & settings() const {return settings_;}
  const SimulatorConfig & config() const {return config_;}

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

  void declareSettings()
  {
    const NodeSettings defaults;
    settings_.node_id = declare(
      "node_id", defaults.node_id,
      "Index of the mat; names the topic ssl_<node_id>/map and the frame ssl_frame_<node_id>.");
    settings_.frame_period = declare(
      "frame_period", defaults.frame_period, "Period of the simulated mat frames [s].");

    if (settings_.node_id < 0) {
      throw std::invalid_argument("invalid parameters: node_id must be >= 0");
    }
    if (!(settings_.frame_period > 0.0)) {
      throw std::invalid_argument("invalid parameters: frame_period must be > 0");
    }
  }

  void declareMat()
  {
    const SimulatorConfig defaults;
    config_.pose.x = declare(
      "x", defaults.pose.x, "x position of the corner of cell (0, 0) in the map frame [m].");
    config_.pose.y = declare(
      "y", defaults.pose.y, "y position of the corner of cell (0, 0) in the map frame [m].");
    config_.pose.z = declare("z", defaults.pose.z, "Height of the mat surface [m].");
    config_.pose.heading = declare(
      "heading", defaults.pose.heading,
      "Direction of the mat x axis (width), counter-clockwise from the map x axis [rad].");
    config_.width = declare(
      "width", defaults.width, "Extent of the mat along its x axis (heading) [m].");
    config_.height = declare("height", defaults.height, "Extent of the mat along its y axis [m].");
    config_.resolution = declare("resolution", defaults.resolution, "Cell size of the mat [m].");
  }

  void declareSimulation()
  {
    const SimulatorConfig defaults;
    const auto threshold = declare(
      "threshold", static_cast<std::int64_t>(defaults.threshold),
      "Cells with a lower pressure are reported as 0 [1, 255]; wheel contacts reach up to 27.");
    if (threshold < 1 || threshold > 255) {
      throw std::invalid_argument("invalid parameters: threshold must be in [1, 255]");
    }
    config_.threshold = static_cast<int>(threshold);
    config_.stale_duration = declare(
      "stale_seconds", defaults.stale_duration,
      "Vehicles are dropped when no object list arrived for this long [s].");
    const bool microscale = declare(
      "microscale", false,
      "Simulate the wheel contacts of 1:18 model vehicles instead of full-scale vehicles.");
    config_.wheel_contact_size =
      microscale ? kWheelContactSizeMicroscale : kWheelContactSizeFullScale;
  }

  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  NodeSettings settings_;
  SimulatorConfig config_;
};

}  // namespace ssl_simulator

#endif  // SSL_SIMULATOR__COMPONENT__SSLSIMULATORPARAMETERROS_HPP_
