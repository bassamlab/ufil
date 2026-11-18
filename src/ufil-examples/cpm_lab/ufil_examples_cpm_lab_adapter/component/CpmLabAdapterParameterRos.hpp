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

#ifndef CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__CPMLABADAPTERPARAMETERROS_HPP_
#define CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__CPMLABADAPTERPARAMETERROS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "parameter_helpers.hpp"

namespace ufil_examples_cpm_lab_adapter
{

/// Declares, validates and reads the read-only parameters of the CPM Lab adapter node.
class CpmLabAdapterParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit CpmLabAdapterParameterRos(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters)
  {
    vehicle_ids_ = parameters::declareVehicleIds(*parameters, {1});
    vehicle_state_topic_ = parameters::declare<std::string>(
      *parameters, "vehicle_state_topic", "/lab/vehicle_{id}/vehicleState",
      "State topic of each vehicle; {id} is replaced by the vehicle id.");
    if (vehicle_state_topic_.find(kIdPlaceholder) == std::string::npos) {
      throw std::invalid_argument("invalid parameters: vehicle_state_topic must contain {id}");
    }
    source_frame_ = parameters::declareFrame(
      *parameters, "source_frame", "lab",
      "Frame of the CPM Lab vehicle states. The frame id of the received states is ignored.");
    target_frame_ = parameters::declareFrame(
      *parameters, "target_frame", "map", "Frame of the published object lists.");
    publish_rate_ = parameters::declareFrequency(
      *parameters, "publish_rate", 20.0, "Rate of the object list with all vehicles [Hz].");
    state_timeout_ = parameters::declare(
      *parameters, "state_timeout", 0.5,
      "Vehicles without a state for this long [s] are left out of the object list with all "
      "vehicles.");
    if (!(state_timeout_ > 0.0)) {
      throw std::invalid_argument("invalid parameters: state_timeout must be positive");
    }
  }

  /// Ids of the vehicles, also used as object ids.
  const std::vector<std::uint8_t> & vehicleIds() const {return vehicle_ids_;}
  /// State topic of the given vehicle.
  std::string vehicleStateTopic(std::uint8_t id) const
  {
    auto topic = vehicle_state_topic_;
    for (auto pos = topic.find(kIdPlaceholder); pos != std::string::npos;
      pos = topic.find(kIdPlaceholder, pos))
    {
      topic.replace(pos, kIdPlaceholder.size(), std::to_string(id));
    }
    return topic;
  }
  const std::string & sourceFrame() const {return source_frame_;}
  const std::string & targetFrame() const {return target_frame_;}
  double publishRate() const {return publish_rate_;}
  double stateTimeout() const {return state_timeout_;}

private:
  static constexpr std::string_view kIdPlaceholder = "{id}";

  std::vector<std::uint8_t> vehicle_ids_;
  std::string vehicle_state_topic_;
  std::string source_frame_;
  std::string target_frame_;
  double publish_rate_ = 20.0;
  double state_timeout_ = 0.5;
};

}  // namespace ufil_examples_cpm_lab_adapter

#endif  // CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__CPMLABADAPTERPARAMETERROS_HPP_
