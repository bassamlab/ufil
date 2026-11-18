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

#ifndef SSL_SIMULATOR__CONFIG_HPP_
#define SSL_SIMULATOR__CONFIG_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "ssl_simulator/visibility_control.hpp"

namespace ssl_simulator
{

/// Pose of the mat in the map frame. The origin is the corner of cell (0, 0); the x axis of the
/// mat (its width) points in the direction of heading.
struct MatPose
{
  double x = 0.0;        ///< [m]
  double y = 0.0;        ///< [m]
  double z = 0.011;      ///< [m], only used for the published grid and transform
  double heading = 0.0;  ///< [rad], counter-clockwise from the map x axis
};

/// Side length of the square wheel contact patch, including crosstalk, of a full-scale vehicle
/// [m].
inline constexpr double kWheelContactSizeFullScale = 0.405;
/// Side length of the wheel contact patch of a 1:18 model vehicle [m].
inline constexpr double kWheelContactSizeMicroscale = 0.0225;

/// Configuration of the mat simulator.
struct SimulatorConfig
{
  MatPose pose;
  double width = 10.0;      ///< Extent along the mat x axis [m].
  double height = 5.0;      ///< Extent along the mat y axis [m].
  double resolution = 0.2;  ///< Cell size [m].
  /// Cells with a lower pressure are reported as 0. Simulated pressures are in [1, 30].
  int threshold = 10;
  /// Vehicles are dropped when no object list arrived for this long [s].
  double stale_duration = 0.5;
  /// Side length of the square contact patch of a wheel [m].
  double wheel_contact_size = kWheelContactSizeFullScale;
  /// Seed of the random generator (noise, pressure distribution); random if unset.
  std::optional<std::uint32_t> seed;

  /// Number of cells along the x axis.
  SSL_SIMULATOR_PUBLIC std::size_t columns() const;
  /// Number of cells along the y axis.
  SSL_SIMULATOR_PUBLIC std::size_t rows() const;
};

/// @return a description of the first invalid value, or std::nullopt if the configuration is
///         valid.
SSL_SIMULATOR_PUBLIC std::optional<std::string> validate(const SimulatorConfig & config);

}  // namespace ssl_simulator

#endif  // SSL_SIMULATOR__CONFIG_HPP_
