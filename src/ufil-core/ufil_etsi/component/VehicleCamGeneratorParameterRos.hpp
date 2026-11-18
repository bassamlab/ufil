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

#ifndef UFIL_ETSI__COMPONENT__VEHICLECAMGENERATORPARAMETERROS_HPP_
#define UFIL_ETSI__COMPONENT__VEHICLECAMGENERATORPARAMETERROS_HPP_

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "ufil_etsi/types.hpp"
#include "ufil_etsi/vehicle_cam_generator.hpp"
#include "UfilEtsiParameterRos.hpp"

namespace ufil_etsi
{

/// Settings only used by the ROS 2 node.
struct VehicleCamGeneratorSettings
{
  /// UTM frame the input objects are transformed into before the CAM is generated.
  std::string target_frame = "utm_32n";
  /// Index of the own vehicle in the received object lists.
  std::size_t object_array_index = 0;
};

/// Declares, validates and reads the (read-only) parameters of the vehicle CAM generator.
class VehicleCamGeneratorParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit VehicleCamGeneratorParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  {
    ParameterDeclarer declarer(std::move(parameters));
    const VehicleCamGeneratorSettings default_settings;
    const VehicleCamGeneratorConfig default_config;

    settings_.target_frame = declarer.declare(
      "target_frame", default_settings.target_frame,
      "UTM frame (utm_<zone><n|s>, e.g. utm_32n) the objects are transformed into; the CAM "
      "position is computed in this UTM zone.");
    const auto object_array_index = declarer.declare<std::int64_t>(
      "object_array_index", static_cast<std::int64_t>(default_settings.object_array_index),
      "Index of the own vehicle in the received object lists.");
    const auto vehicle_role = declarer.declare<std::int64_t>(
      "vehicle_role", default_config.vehicle_role,
      "Vehicle role, see ETSI TS 102 894-2 DE_VehicleRole: 0 (default) to 15.");
    config_.time_shift = declarer.declareTimeShift();

    const auto zone = utmZoneFromFrame(settings_.target_frame);
    if (!zone) {
      throw std::invalid_argument(
              "invalid parameters: target_frame must be a UTM frame like utm_32n, got '" +
              settings_.target_frame + "'");
    }
    config_.utm_zone = *zone;
    if (object_array_index < 0) {
      throw std::invalid_argument("invalid parameters: object_array_index must be >= 0");
    }
    settings_.object_array_index = static_cast<std::size_t>(object_array_index);
    if (vehicle_role < 0 || vehicle_role > etsi::kVehicleRoleMax) {
      throw std::invalid_argument("invalid parameters: vehicle_role must be in [0, 15]");
    }
    config_.vehicle_role = static_cast<std::uint8_t>(vehicle_role);
    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
  }

  const VehicleCamGeneratorSettings & settings() const {return settings_;}
  const VehicleCamGeneratorConfig & config() const {return config_;}

private:
  VehicleCamGeneratorSettings settings_;
  VehicleCamGeneratorConfig config_;
};

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__COMPONENT__VEHICLECAMGENERATORPARAMETERROS_HPP_
