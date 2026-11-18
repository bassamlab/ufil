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

#include "ufil_examples_cpm_lab_adapter/vehicle_object.hpp"

#include <Eigen/Geometry>

namespace ufil_examples_cpm_lab_adapter
{

namespace
{

/// Dimensions of a CPM Lab vehicle [m].
constexpr double kVehicleLength = 0.22;
constexpr double kVehicleWidth = 0.105;
constexpr double kVehicleHeight = 0.06;

}  // namespace

VehicleObject toVehicleObject(const VehicleState & state, std::uint32_t id)
{
  VehicleObject object;
  object.id = id;
  object.existence_probability = 0.99;

  object.x = state.position.x() * kLabScale;
  object.y = state.position.y() * kLabScale;
  object.yaw = state.yaw;
  const Eigen::Vector2d velocity = Eigen::Rotation2Dd(state.yaw) *
    (state.linear_velocity.head<2>() * kLabScale);
  object.vx = velocity.x();
  object.vy = velocity.y();
  // The positioning system of the lab is accurate, the yaw rate is not estimated.
  object.state_variances = {0.9, 0.9, 1.4, 1.4, 0.05, -1.0};

  object.length = kVehicleLength * kLabScale;
  object.width = kVehicleWidth * kLabScale;
  object.height = kVehicleHeight * kLabScale;
  object.dimension_variances = {0.081 * kLabScale, 0.001 * kLabScale, 0.001 * kLabScale};
  return object;
}

}  // namespace ufil_examples_cpm_lab_adapter
