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

#ifndef SSL_WHEEL_TRACKER__TYPES_HPP_
#define SSL_WHEEL_TRACKER__TYPES_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <vector>

#include <ufil_object_tracking/types/time.hpp>

namespace ssl_wheel_tracker
{

using Timestamp = ufil::type::Timestamp;
using Duration = ufil::type::Duration;

/// Two-dimensional point [m].
struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

/// Pose of the corner of cell (0, 0) of a grid; the column axis points in the direction of yaw.
struct GridOrigin
{
  double x = 0.0;    ///< [m]
  double y = 0.0;    ///< [m]
  double yaw = 0.0;  ///< [rad], counter-clockwise
};

/// Cell of a grid. Columns run along the x axis of the grid, rows along its y axis.
struct Cell
{
  std::size_t column = 0;
  std::size_t row = 0;

  bool operator==(const Cell & other) const = default;
};

/// Pressure of every cell of a mat, row-major (index = row * columns + column), 0 = no
/// pressure. The layout matches nav_msgs/OccupancyGrid.
struct PressureGrid
{
  std::size_t columns = 0;
  std::size_t rows = 0;
  double resolution = 0.0;  ///< Cell size [m].
  GridOrigin origin;
  std::vector<std::uint8_t> values;

  std::uint8_t & at(std::size_t column, std::size_t row) {return values[row * columns + column];}
  std::uint8_t at(std::size_t column, std::size_t row) const
  {
    return values[row * columns + column];
  }
  std::uint8_t at(const Cell & cell) const {return at(cell.column, cell.row);}

  /// True if the cell count matches the size and the resolution is positive.
  bool valid() const {return values.size() == columns * rows && resolution > 0.0;}
};

/// Wheel contact detected in a pressure grid.
struct WheelDetection
{
  /// Pressure-weighted center of the contact, in the frame of the grid origin [m].
  Point2D position;
  /// Sum of the pressure of the contact cells above the mean background pressure.
  double pressure = 0.0;
  /// Number of cells assigned to the contact.
  std::size_t cells = 0;
};

/// Constant velocity state, ordered [x, vx, y, vy] ([m], [m/s]).
using StateVector = Eigen::Vector4d;
using StateCovariance = Eigen::Matrix4d;

/// Estimate of a track at one point in time.
struct TrackState
{
  Timestamp stamp;
  StateVector mean = StateVector::Zero();
  StateCovariance covariance = StateCovariance::Identity();
  /// True if a detection was associated at this time, false for a prediction.
  bool matched = false;

  double x() const {return mean(0);}
  double vx() const {return mean(1);}
  double y() const {return mean(2);}
  double vy() const {return mean(3);}
};

/// Wheel track.
struct Track
{
  std::uint64_t id = 0;
  /// The last history_length matched states, followed by the predictions since the last match.
  std::vector<TrackState> history;
  /// Number of associated detections since the track was started.
  std::size_t hits = 0;
  /// Number of consecutive frames without an associated detection.
  std::size_t misses = 0;
  Timestamp last_update;

  const TrackState & state() const {return history.back();}
};

}  // namespace ssl_wheel_tracker

#endif  // SSL_WHEEL_TRACKER__TYPES_HPP_
