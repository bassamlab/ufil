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

#ifndef UFIL_ETSI__COMPONENT__RSUCAMGENERATORPARAMETERROS_HPP_
#define UFIL_ETSI__COMPONENT__RSUCAMGENERATORPARAMETERROS_HPP_

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "ufil_etsi/rsu_cam_generator.hpp"
#include "UfilEtsiParameterRos.hpp"

namespace ufil_etsi
{

/// Declares, validates and reads the (read-only) parameters of the RSU CAM generator.
class RsuCamGeneratorParameterRos
{
public:
  /// Default position: the CPM lab (see ufil_examples_cpm_lab).
  static constexpr double kDefaultLatitude = 48.079772;
  static constexpr double kDefaultLongitude = 11.635735;

  /// @throws std::invalid_argument if the parameters are invalid.
  explicit RsuCamGeneratorParameterRos(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
  {
    ParameterDeclarer declarer(std::move(parameters));
    const RsuCamGeneratorConfig defaults;

    const auto station_id = declarer.declare<std::int64_t>(
      "station_id", -1,
      "ITS station id of the road side unit in [0, 4294967295], -1 for a random id.");
    config_.position.latitude = declarer.declare(
      "position.latitude", kDefaultLatitude, "Latitude of the road side unit (WGS84) [deg].");
    config_.position.longitude = declarer.declare(
      "position.longitude", kDefaultLongitude, "Longitude of the road side unit (WGS84) [deg].");
    config_.altitude = declarer.declare(
      "position.altitude", defaults.altitude,
      "Altitude of the road side unit above the WGS84 ellipsoid [m].");
    config_.time_shift = declarer.declareTimeShift();

    if (station_id < -1 || station_id > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument(
              "invalid parameters: station_id must be -1 or in [0, 4294967295]");
    }
    random_station_id_ = station_id == -1;
    config_.station_id =
      random_station_id_ ? randomStationId() : static_cast<std::uint32_t>(station_id);
    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
  }

  const RsuCamGeneratorConfig & config() const {return config_;}
  /// True if the station id was chosen randomly.
  bool randomStationIdUsed() const {return random_station_id_;}

private:
  RsuCamGeneratorConfig config_;
  bool random_station_id_ = false;
};

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__COMPONENT__RSUCAMGENERATORPARAMETERROS_HPP_
