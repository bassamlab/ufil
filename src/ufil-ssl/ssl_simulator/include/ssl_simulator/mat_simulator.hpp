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

#ifndef SSL_SIMULATOR__MAT_SIMULATOR_HPP_
#define SSL_SIMULATOR__MAT_SIMULATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <vector>

#include "ssl_simulator/config.hpp"
#include "ssl_simulator/types.hpp"
#include "ssl_simulator/visibility_control.hpp"

namespace ssl_simulator
{

/// Event of a simulated frame, reported to the caller for logging.
enum class FrameEvent
{
  kNone,
  /// The last object list is older than the stale duration, all vehicles were dropped.
  kVehiclesExpired,
  /// The time jumped backwards; vehicles received before the jump were dropped.
  kTimeJump,
};

/// Result of MatSimulator::step().
struct Frame
{
  PressureMap pressure;
  /// Vehicles with at least one wheel on the mat.
  std::size_t vehicles_on_mat = 0;
  FrameEvent event = FrameEvent::kNone;
};

/// Counters and state of the simulator for diagnostics.
struct SimulatorStatus
{
  /// Vehicles of the last object list that are still simulated.
  std::size_t vehicles = 0;
  /// Vehicles with at least one wheel on the mat in the last frame.
  std::size_t vehicles_on_mat = 0;
  /// Cells at or above the threshold in the last frame.
  std::size_t active_cells = 0;
  std::uint64_t frames = 0;
  std::uint64_t object_lists = 0;
  /// Vehicles dropped because their state is not finite.
  std::uint64_t rejected_vehicles = 0;
  std::uint64_t expirations = 0;
  std::uint64_t time_jumps = 0;
  std::optional<Timestamp> last_object_list;
  std::optional<Timestamp> last_frame;
};

/**
 * @brief Simulates the pressure map of the sensitive surface layer (SSL) for vehicles driving
 *        over it.
 *
 * Every object list replaces all simulated vehicles. step() extrapolates each vehicle with a
 * constant velocity and yaw rate model to the frame time, rasterizes the square contact patches
 * of its wheels onto the mat grid and adds line crosstalk, sensor noise and the threshold of the
 * mat electronics.
 *
 * The class is not thread-safe.
 */
class SSL_SIMULATOR_PUBLIC MatSimulator
{
public:
  /// @throws std::invalid_argument if the configuration is invalid.
  explicit MatSimulator(const SimulatorConfig & config);

  /**
   * @brief Replaces all simulated vehicles.
   * @param vehicles vehicles in the map frame; the state timestamps are the measurement times.
   * @param received time the object list was received, used for the stale check.
   * @return number of vehicles dropped because their state is not finite.
   */
  std::size_t setVehicles(std::vector<Vehicle> vehicles, const Timestamp & received);

  /// Simulates the frame of the mat at the given time.
  Frame step(const Timestamp & now);

  const SimulatorConfig & config() const {return config_;}
  SimulatorStatus status() const;

private:
  void addVehicle(
    const Vehicle & vehicle, const Timestamp & now, PressureMap & contact,
    std::size_t & vehicles_on_mat);
  void addNoise(PressureMap & pressure);
  /// True if the last object list is older than the stale duration; requires received_.
  bool isStale(const Timestamp & now) const;

  SimulatorConfig config_;
  std::size_t columns_;
  std::size_t rows_;
  std::mt19937 generator_;

  std::vector<Vehicle> vehicles_;
  std::optional<Timestamp> received_;
  SimulatorStatus status_;
};

/// Extrapolates the state to the given time with a constant velocity and yaw rate model.
SSL_SIMULATOR_PUBLIC VehicleState predict(
  const VehicleState & state,
  const Timestamp & time);

/// Transforms a point from the map frame into the mat frame.
SSL_SIMULATOR_PUBLIC Point2D mapToMat(const MatPose & pose, const Point2D & point);

/**
 * @brief Wheel centers of a vehicle.
 * @param center vehicle center
 * @param yaw vehicle yaw in the same frame as center [rad]
 */
SSL_SIMULATOR_PUBLIC std::vector<Point2D> wheelPositions(
  const Point2D & center, double yaw, const std::vector<Axle> & axles);

/**
 * @brief Cells whose center lies inside the square contact patch of a wheel.
 * @param wheel wheel center in the mat frame
 * @param yaw orientation of the patch in the mat frame [rad]
 */
SSL_SIMULATOR_PUBLIC std::vector<Cell> contactCells(
  const Point2D & wheel, double yaw, double size, double resolution, std::size_t columns,
  std::size_t rows);

/// Adds the crosstalk of pressed cells along their row: each neighbour receives 70 % of the
/// pressure of the previous one (rounded down). Cells keep the maximum of all contributions.
SSL_SIMULATOR_PUBLIC PressureMap addLineCrosstalk(const PressureMap & pressure);

/// Sets cells below the threshold to 0.
SSL_SIMULATOR_PUBLIC void applyThreshold(PressureMap & pressure, int threshold);

}  // namespace ssl_simulator

#endif  // SSL_SIMULATOR__MAT_SIMULATOR_HPP_
