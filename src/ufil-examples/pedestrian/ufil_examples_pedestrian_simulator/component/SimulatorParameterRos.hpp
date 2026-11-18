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

#ifndef PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__COMPONENT__SIMULATORPARAMETERROS_HPP_
#define PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__COMPONENT__SIMULATORPARAMETERROS_HPP_

#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter_value.hpp>

namespace ufil_examples_pedestrian_simulator
{

/// Configuration of the simulator node.
struct SimulatorSettings
{
  /// Absolute path, or a file name in the config directory of this package.
  std::string scenario_file = "crossing.yaml";
  /// Simulation step and publication period [s].
  double timestep = 0.1;
  /// Frame of the published objects.
  std::string frame_id = "map";
};

/// The absolute scenario_file, or scenario_file in scenario_directory.
inline std::string resolveScenarioFile(
  const std::string & scenario_file, const std::string & scenario_directory)
{
  const std::filesystem::path path(scenario_file);
  if (path.is_absolute()) {
    return path.string();
  }
  return (std::filesystem::path(scenario_directory) / path).string();
}

/**
 * @brief Declares, validates and reads the parameters of the pedestrian simulator node.
 *
 * All parameters are read-only: a different scenario requires restarting the node.
 */
class SimulatorParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit SimulatorParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  : parameters_(std::move(parameters))
  {
    const SimulatorSettings defaults;
    settings_.scenario_file = declare(
      "scenario_file", defaults.scenario_file,
      "Scenario file: an absolute path or a file name in the config directory of "
      "ufil_examples_pedestrian_simulator, e.g. crossing.yaml.");
    settings_.timestep = declare(
      "timestep", defaults.timestep, "Simulation step and publication period [s].");
    settings_.frame_id = declare(
      "frame_id", defaults.frame_id, "Frame of the published objects.");

    if (settings_.scenario_file.empty()) {
      throw std::invalid_argument("invalid parameters: scenario_file must be set");
    }
    if (!(settings_.timestep > 0.0)) {
      throw std::invalid_argument("invalid parameters: timestep must be > 0");
    }
    if (settings_.frame_id.empty()) {
      throw std::invalid_argument("invalid parameters: frame_id must be set");
    }
  }

  const SimulatorSettings & settings() const {return settings_;}

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
  SimulatorSettings settings_;
};

}  // namespace ufil_examples_pedestrian_simulator

#endif  // PEDESTRIAN__UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__COMPONENT__SIMULATORPARAMETERROS_HPP_
