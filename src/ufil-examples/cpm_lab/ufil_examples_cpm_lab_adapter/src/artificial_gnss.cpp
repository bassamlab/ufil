// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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


#include "ufil_examples_cpm_lab_adapter/artificial_gnss.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Convert from ROS quaternion to yaw angle
inline double convertQuaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  tf2::Quaternion tf_quat;
  tf2::fromMsg(quaternion, tf_quat);
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  tf2::Matrix3x3(tf_quat).getEulerYPR(yaw, pitch, roll);
  return yaw;
}

namespace ufil_examples_cpm_lab_adapter
{
ArtificialGnss::ArtificialGnss(
  const int number_of_frequency_bands, const float clock_error,
  const float ionospheric_error, const float ephemeris_error)
: random_engine_(random_device_())
  , number_of_frequency_bands_(number_of_frequency_bands)
  , clock_error_(clock_error)
  , ephemeris_error_(ephemeris_error)
  , ionospheric_error_(ionospheric_error)
  , satellite_positions_{{-1000000, -1000000, 1111111},
    {1732050, 2, 1111111},
    {-1000000, 1000000, 1111111},
    {2.25, 2, 1277778}}
  , back_ranges_(4, 0.0f)
  , center_ranges_(4, 0.0f)
{
}

void ArtificialGnss::updateReceiverPositions(
  const cpm_lab_lab_msgs::msg::VehicleState::SharedPtr & vehicle_state_msg)
{
  // Generate center receiver pose
  this->center_receiver_position_.x() = vehicle_state_msg->pose.position.x;
  this->center_receiver_position_.y() = vehicle_state_msg->pose.position.y;

  // Generate back receive pose
  auto back_receiver_vec = Eigen::Vector3f(-0.075f, 0.0f, 0.0f);
  double yaw = convertQuaternionToYaw(vehicle_state_msg->pose.orientation);
  Eigen::AngleAxisf rotation(static_cast<float>(yaw),
    Eigen::Vector3f::UnitZ());
  back_receiver_vec = rotation * back_receiver_vec;

  this->back_receiver_position_.x() = vehicle_state_msg->pose.position.x + back_receiver_vec.x();
  this->back_receiver_position_.y() = vehicle_state_msg->pose.position.y + back_receiver_vec.y();
}

float ArtificialGnss::scaledRandomError(float real_world_error_impact)
{
  // 95% Rule
  auto std_deviation = real_world_error_impact / 18.0f / 3.0f;
  std::normal_distribution<float> distribution(0, std_deviation);
  return distribution(this->random_engine_);
}

void ArtificialGnss::updatePseudoRanges()
{
  for (size_t i = 0; i < this->satellite_positions_.size(); i++) {
    this->center_ranges_.at(i) = 0;
    this->back_ranges_.at(i) = 0;
    for (auto k = 0; k < this->number_of_frequency_bands_; k++) {
      const auto err_ionospheric = this->scaledRandomError(this->ionospheric_error_);
      const auto err_clock = this->scaledRandomError(this->clock_error_);
      const auto err_ephemeris = this->scaledRandomError(this->ephemeris_error_);
      const auto err_total = err_ionospheric + err_clock + err_ephemeris;

      const auto & satellite_position = this->satellite_positions_.at(i);
      const auto center_distance = (this->center_receiver_position_ - satellite_position).norm() +
        err_total;
      const auto back_distance = (this->back_receiver_position_ - satellite_position).norm() +
        err_total;

      this->center_ranges_.at(i) += center_distance;
      this->back_ranges_.at(i) += back_distance;
    }
    this->center_ranges_.at(i) /= this->number_of_frequency_bands_;
    this->back_ranges_.at(i) /= this->number_of_frequency_bands_;
  }
}

Eigen::Vector3d ArtificialGnss::updateGnssLocks()
{
  Eigen::Matrix4d A;
  Eigen::Vector4d b_back;

  A << 1.0, -2.0 * this->satellite_positions_[0].x(), -2.0 * this->satellite_positions_[0].y(),
    -2.0 * this->satellite_positions_[0].z(), 1.0, -2.0 * this->satellite_positions_[1].x(),
    -2.0 * this->satellite_positions_[1].y(), -2.0 * this->satellite_positions_[1].z(), 1.0,
    -2.0 * this->satellite_positions_[2].x(), -2.0 * this->satellite_positions_[2].y(),
    -2.0 * this->satellite_positions_[2].z(), 1.0, -2.0 * this->satellite_positions_[3].x(),
    -2.0 * this->satellite_positions_[3].y(), -2.0 * this->satellite_positions_[3].z();

  b_back << std::pow(this->back_ranges_[0], 2) - std::pow(this->satellite_positions_[0].norm(), 2),
    std::pow(this->back_ranges_[1], 2) - std::pow(this->satellite_positions_[1].norm(), 2),
    std::pow(this->back_ranges_[2], 2) - std::pow(this->satellite_positions_[2].norm(), 2),
    std::pow(this->back_ranges_[3], 2) - std::pow(this->satellite_positions_[3].norm(), 2);

  auto qr_decomposition = A.colPivHouseholderQr();
  auto back_triangulated = qr_decomposition.solve(b_back);

  auto x = back_triangulated[1];
  auto y = back_triangulated[2];
  return {x, y, 0.0};
}

Eigen::Vector3d
ArtificialGnss::generateGnssLock(
  const cpm_lab_lab_msgs::msg::VehicleState::SharedPtr & vehicle_state_msg)
{
  this->updateReceiverPositions(vehicle_state_msg);
  this->updatePseudoRanges();
  auto position = this->updateGnssLocks();

  return position;
}
}  // namespace ufil_examples_cpm_lab_adapter
