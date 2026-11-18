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

#ifndef CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__CPMLABSENSORPARAMETERROS_HPP_
#define CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__CPMLABSENSORPARAMETERROS_HPP_

#include <cstdint>
#include <stdexcept>
#include <string>

#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "ufil_examples_cpm_lab_adapter/sensor_simulator.hpp"
#include "parameter_helpers.hpp"

namespace ufil_examples_cpm_lab_adapter
{

/// Declares, validates and reads the read-only parameters of the CPM Lab sensor node.
class CpmLabSensorParameterRos
{
public:
  /// @throws std::invalid_argument if the parameters are invalid.
  explicit CpmLabSensorParameterRos(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters)
  {
    vehicle_id_ = parameters::declareVehicleId(*parameters, 1);
    target_frame_ = parameters::declareFrame(
      *parameters, "target_frame", "map",
      "Frame whose origin is at the GNSS origin. The vehicle states are transformed into it.");

    gnss_frequency_ = parameters::declareFrequency(
      *parameters, "gnss.frequency", 5.0, "Rate of the GNSS fixes [Hz].");
    config_.artificial_gnss_errors = parameters::declare(
      *parameters, "gnss.artificial_errors", config_.artificial_gnss_errors,
      "Disturb the GNSS fixes with simulated clock, ionospheric and ephemeris errors.");
    config_.origin_latitude = parameters::declare(
      *parameters, "gnss.origin.latitude", config_.origin_latitude,
      "WGS84 latitude of the origin of target_frame [deg].");
    config_.origin_longitude = parameters::declare(
      *parameters, "gnss.origin.longitude", config_.origin_longitude,
      "WGS84 longitude of the origin of target_frame [deg].");

    odometry_frequency_ = parameters::declareFrequency(
      *parameters, "odometry.frequency", 50.0, "Rate of the IMU and wheel odometry messages [Hz].");

    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid parameters: " + *error);
    }
  }

  std::uint8_t vehicleId() const {return vehicle_id_;}
  const std::string & targetFrame() const {return target_frame_;}
  double gnssFrequency() const {return gnss_frequency_;}
  double odometryFrequency() const {return odometry_frequency_;}
  const SensorSimulatorConfig & config() const {return config_;}

private:
  std::uint8_t vehicle_id_ = 1;
  std::string target_frame_;
  double gnss_frequency_ = 5.0;
  double odometry_frequency_ = 50.0;
  SensorSimulatorConfig config_;
};

}  // namespace ufil_examples_cpm_lab_adapter

#endif  // CPM_LAB__UFIL_EXAMPLES_CPM_LAB_ADAPTER__COMPONENT__CPMLABSENSORPARAMETERROS_HPP_
