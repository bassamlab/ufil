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

#ifndef UFIL_ETSI__COMPONENT__VEHICLECAMRECEIVERPARAMETERROS_HPP_
#define UFIL_ETSI__COMPONENT__VEHICLECAMRECEIVERPARAMETERROS_HPP_

#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "ufil_etsi/vehicle_cam_receiver.hpp"
#include "UfilEtsiParameterRos.hpp"

namespace ufil_etsi
{

/// Settings only used by the ROS 2 node.
struct VehicleCamReceiverSettings
{
  /// Frame the received objects are transformed into and published in.
  std::string target_frame = "map";
};

/// Declares, validates and reads the (read-only) parameters of the vehicle CAM receiver.
class VehicleCamReceiverParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit VehicleCamReceiverParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  {
    ParameterDeclarer declarer(std::move(parameters));
    const VehicleCamReceiverSettings defaults;
    settings_.target_frame = declarer.declare(
      "target_frame", defaults.target_frame,
      "Frame the objects of the received CAMs are transformed into and published in.");
    config_.time_shift = declarer.declareTimeShift();

    if (settings_.target_frame.empty()) {
      throw std::invalid_argument("invalid parameters: target_frame must be set");
    }
  }

  const VehicleCamReceiverSettings & settings() const {return settings_;}
  const VehicleCamReceiverConfig & config() const {return config_;}

private:
  VehicleCamReceiverSettings settings_;
  VehicleCamReceiverConfig config_;
};

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__COMPONENT__VEHICLECAMRECEIVERPARAMETERROS_HPP_
