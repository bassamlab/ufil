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

#ifndef UFIL_EXAMPLES_CPM_LAB_ADAPTER__VEHICLE_OBJECT_HPP_
#define UFIL_EXAMPLES_CPM_LAB_ADAPTER__VEHICLE_OBJECT_HPP_

#include <array>
#include <cstdint>

#include "ufil_examples_cpm_lab_adapter/vehicle_state.hpp"
#include "ufil_examples_cpm_lab_adapter/visibility_control.hpp"

namespace ufil_examples_cpm_lab_adapter
{

/**
 * @brief A CPM Lab vehicle as a UFIL object, scaled to real-world dimensions.
 *
 * Position and velocity are given in the frame of the vehicle state.
 */
struct VehicleObject
{
  std::uint32_t id = 0;
  double existence_probability = 0.0;
  /// Position [m].
  double x = 0.0;
  double y = 0.0;
  /// Velocity [m/s].
  double vx = 0.0;
  double vy = 0.0;
  /// Heading [rad].
  double yaw = 0.0;
  /// Variances of x, y, vx, vy, yaw and yaw rate. A negative variance marks a value that is not
  /// estimated (the yaw rate).
  std::array<double, 6> state_variances{};
  /// Dimensions [m].
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  /// Variances of length, width and height.
  std::array<double, 3> dimension_variances{};
};

/**
 * @brief Converts the state of a CPM Lab vehicle into a car object with real-world dimensions.
 *
 * Position, velocity and the vehicle dimensions are scaled by kLabScale; the velocity is rotated
 * from the vehicle frame into the frame of the state.
 */
UFIL_EXAMPLES_CPM_LAB_ADAPTER_PUBLIC VehicleObject toVehicleObject(
  const VehicleState & state, std::uint32_t id);

}  // namespace ufil_examples_cpm_lab_adapter

#endif  // UFIL_EXAMPLES_CPM_LAB_ADAPTER__VEHICLE_OBJECT_HPP_
