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

#ifndef UFIL_EXAMPLES_CPM_LAB_ADAPTER__VEHICLE_STATE_HPP_
#define UFIL_EXAMPLES_CPM_LAB_ADAPTER__VEHICLE_STATE_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <string>

namespace ufil_examples_cpm_lab_adapter
{

/// Scale of the CPM Lab: its vehicles are 1:18 models of passenger cars.
inline constexpr double kLabScale = 18.0;

/// State of a CPM Lab vehicle in lab dimensions (not scaled to real-world dimensions).
struct VehicleState
{
  /// Time the state refers to [ns].
  std::int64_t stamp_nanoseconds = 0;
  /// Frame the pose is given in.
  std::string frame_id;
  /// Position [m].
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  /// Heading around the z axis [rad].
  double yaw = 0.0;
  /// Linear velocity in the vehicle frame [m/s].
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  /// Angular velocity in the vehicle frame [rad/s].
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  /// Linear acceleration in the vehicle frame [m/s^2].
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
};

}  // namespace ufil_examples_cpm_lab_adapter

#endif  // UFIL_EXAMPLES_CPM_LAB_ADAPTER__VEHICLE_STATE_HPP_
