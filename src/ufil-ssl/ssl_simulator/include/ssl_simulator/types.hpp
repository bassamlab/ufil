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

#ifndef SSL_SIMULATOR__TYPES_HPP_
#define SSL_SIMULATOR__TYPES_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <ufil_object_tracking/types/state.hpp>
#include <ufil_object_tracking/types/time.hpp>

namespace ssl_simulator
{

using Timestamp = ufil::type::Timestamp;
using Duration = ufil::type::Duration;

/// Kinematic state of a vehicle in the map frame: x, y [m], absolute velocity v_x, v_y [m/s],
/// yaw [rad] and yaw rate [rad/s]. The timestamp is the time the state refers to.
using VehicleState = ufil::type::state::PoseVelocity2D;

/// Two-dimensional point [m].
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

/// Axle of a vehicle.
struct Axle
{
  /// True if the axle has a single wheel on the longitudinal axis of the vehicle.
  bool single_track = false;
  /// Distance between the wheels of the axle [m], ignored for single track axles.
  double track_width = 0.0;
  /// Longitudinal distance from the vehicle center to the axle, positive to the front [m].
  double center_to_axle = 0.0;
};

/// Vehicle driving over the mat.
struct Vehicle
{
  std::uint32_t id = 0;
  VehicleState state;
  /// Only the wheels of the axles touch the mat; a vehicle without axles is invisible.
  std::vector<Axle> axles;
};

/// Cell of the mat grid. Columns run along the x axis (width) of the mat, rows along its y
/// axis (height).
struct Cell
{
  std::size_t column = 0;
  std::size_t row = 0;

  bool operator==(const Cell & other) const = default;
};

/// Pressure of every cell of the mat, row-major (index = row * columns + column), 0 = no
/// pressure. The layout matches nav_msgs/OccupancyGrid.
struct PressureMap
{
  std::size_t columns = 0;
  std::size_t rows = 0;
  std::vector<std::uint8_t> values;

  PressureMap() = default;
  PressureMap(std::size_t columns_, std::size_t rows_)
  : columns(columns_), rows(rows_), values(columns_ * rows_, 0U)
  {
  }

  std::uint8_t & at(std::size_t column, std::size_t row) {return values[row * columns + column];}
  std::uint8_t at(std::size_t column, std::size_t row) const
  {
    return values[row * columns + column];
  }
  std::uint8_t at(const Cell & cell) const {return at(cell.column, cell.row);}

  /// Number of cells with a pressure > 0.
  std::size_t activeCells() const
  {
    return static_cast<std::size_t>(
      std::count_if(values.begin(), values.end(), [](std::uint8_t v) {return v > 0U;}));
  }
};

}  // namespace ssl_simulator

#endif  // SSL_SIMULATOR__TYPES_HPP_
