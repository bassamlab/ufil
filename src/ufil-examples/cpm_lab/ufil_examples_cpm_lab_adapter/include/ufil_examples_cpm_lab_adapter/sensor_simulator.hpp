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

#ifndef UFIL_EXAMPLES_CPM_LAB_ADAPTER__SENSOR_SIMULATOR_HPP_
#define UFIL_EXAMPLES_CPM_LAB_ADAPTER__SENSOR_SIMULATOR_HPP_

#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <string>

#include "ufil_examples_cpm_lab_adapter/artificial_gnss.hpp"
#include "ufil_examples_cpm_lab_adapter/vehicle_state.hpp"
#include "ufil_examples_cpm_lab_adapter/visibility_control.hpp"

namespace ufil_examples_cpm_lab_adapter
{

struct SensorSimulatorConfig
{
  /// WGS84 position of the origin of the target frame [deg]. Default: the CPM Lab.
  double origin_latitude = 48.079772;
  double origin_longitude = 11.635734;
  /// Disturb the GNSS fixes with ArtificialGnss. Otherwise the fixes are exact.
  bool artificial_gnss_errors = true;
  ArtificialGnssConfig gnss_errors;
  /// Vehicle states older than this are discarded [s].
  double max_state_age = 1.0;
};

/// @return a description of the first invalid value, or std::nullopt if the configuration is valid.
UFIL_EXAMPLES_CPM_LAB_ADAPTER_PUBLIC std::optional<std::string> validate(
  const SensorSimulatorConfig & config);

struct GnssFix
{
  /// WGS84 position [deg].
  double latitude = 0.0;
  double longitude = 0.0;
  /// Variance of the position in each axis [m^2].
  double position_variance = 0.0;
};

struct ImuMeasurement
{
  /// Heading in the target frame [rad].
  double yaw = 0.0;
  /// Angular velocity [rad/s] and linear acceleration [m/s^2] of the vehicle state, not scaled.
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
};

struct WheelOdometry
{
  /// Linear velocity in the vehicle frame, scaled to real-world dimensions [m/s].
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
};

enum class StateCheck
{
  /// No vehicle state received since the last one was discarded.
  kNoState,
  /// The latest vehicle state is recent enough to generate measurements.
  kValid,
  /// The latest vehicle state was older than max_state_age and has just been discarded.
  kExpired,
};

struct SensorSimulatorStatus
{
  std::uint64_t states_received = 0;
  std::uint64_t states_expired = 0;
  std::uint64_t gnss_fixes = 0;
  std::uint64_t imu_measurements = 0;
  std::uint64_t odometry_measurements = 0;
  /// Stamp of the latest received state [ns], std::nullopt if none was received yet.
  std::optional<std::int64_t> last_state_stamp_nanoseconds;
  /// Whether a state is available for measurements.
  bool has_state = false;
};

/**
 * @brief Generates GNSS, IMU and wheel odometry measurements from CPM Lab vehicle states.
 *
 * Positions and velocities are scaled to real-world dimensions by kLabScale. The GNSS position is
 * the (optionally disturbed) vehicle position transformed into the target frame, scaled and
 * offset by the UTM coordinates of the configured origin.
 */
class UFIL_EXAMPLES_CPM_LAB_ADAPTER_PUBLIC SensorSimulator
{
public:
  /**
   * @param seed seed of the artificial GNSS errors; std::nullopt draws one from std::random_device.
   * @throws std::invalid_argument if the configuration is invalid.
   */
  explicit SensorSimulator(
    const SensorSimulatorConfig & config, std::optional<unsigned int> seed = std::nullopt);

  void setState(const VehicleState & state);

  /// Discards the state if it is older than max_state_age at now_nanoseconds.
  StateCheck checkState(std::int64_t now_nanoseconds);

  /// Latest vehicle state, std::nullopt if there is none or it was discarded.
  const std::optional<VehicleState> & state() const {return state_;}

  /**
   * The measurements require a state (see state()); they throw std::logic_error otherwise.
   * @param state_to_target transformation from the frame of the state into the target frame in
   *        lab dimensions.
   */
  GnssFix gnssFix(const Eigen::Isometry3d & state_to_target);
  ImuMeasurement imu(const Eigen::Isometry3d & state_to_target);
  WheelOdometry wheelOdometry();

  SensorSimulatorStatus status() const {return status_;}
  const SensorSimulatorConfig & config() const {return config_;}

private:
  const VehicleState & requireState() const;

  SensorSimulatorConfig config_;
  ArtificialGnss artificial_gnss_;
  /// UTM coordinates of the origin.
  int zone_ = 0;
  bool northp_ = true;
  double origin_easting_ = 0.0;
  double origin_northing_ = 0.0;
  std::optional<VehicleState> state_;
  SensorSimulatorStatus status_;
};

}  // namespace ufil_examples_cpm_lab_adapter

#endif  // UFIL_EXAMPLES_CPM_LAB_ADAPTER__SENSOR_SIMULATOR_HPP_
