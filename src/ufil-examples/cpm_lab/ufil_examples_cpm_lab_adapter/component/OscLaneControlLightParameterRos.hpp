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

#ifndef CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__OSCLANECONTROLLIGHTPARAMETERROS_HPP_
#define CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__OSCLANECONTROLLIGHTPARAMETERROS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <ufil_osc/config.hpp>

#include "parameter_helpers.hpp"

namespace ufil_examples_cpm_lab_adapter
{

/**
 * @brief Declares, validates and reads the read-only parameters of the lane control light OSC
 * converter node.
 */
class OscLaneControlLightParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit OscLaneControlLightParameterRos(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters)
  {
    const ufil_osc::OscConfig defaults;
    config_.host = parameters::declare(
      *parameters, "receiver.host", defaults.host, "Host name or IP address of the OSC receiver.");
    const auto port = parameters::declare<std::int64_t>(
      *parameters, "receiver.port", defaults.port, "UDP port of the OSC receiver.");
    if (port < 1 || port > 65535) {
      throw std::invalid_argument("invalid parameters: receiver.port must be in [1, 65535]");
    }
    config_.port = static_cast<int>(port);
    config_.address = parameters::declare<std::string>(
      *parameters, "osc_address", "/lane_control_light",
      "OSC address of the messages; their single string argument is the JSON encoded lane "
      "control light state.");

    if (const auto error = ufil_osc::validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
  }

  const ufil_osc::OscConfig & config() const {return config_;}

private:
  ufil_osc::OscConfig config_;
};

}  // namespace ufil_examples_cpm_lab_adapter

#endif  // CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__OSCLANECONTROLLIGHTPARAMETERROS_HPP_
