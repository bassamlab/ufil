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


#ifndef UFIL_EXAMPLES_CPM_LAB_ADAPTER__ARTIFICIAL_GNSS_HPP_
#define UFIL_EXAMPLES_CPM_LAB_ADAPTER__ARTIFICIAL_GNSS_HPP_

#include <eigen3/Eigen/Dense>
#include <memory>
#include <random>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <cpm_lab_lab_msgs/msg/vehicle_state.hpp>

namespace ufil_examples_cpm_lab_adapter
{
class ArtificialGnss
{
private:
  std::random_device random_device_;
  std::default_random_engine random_engine_;

  Eigen::Vector3d back_receiver_position_;
  Eigen::Vector3d center_receiver_position_;

  const int number_of_frequency_bands_;

  // Real world error values
  const float clock_error_;
  const float ephemeris_error_;
  const float ionospheric_error_;

  const std::vector<Eigen::Vector3d> satellite_positions_;

  std::vector<double> back_ranges_;
  std::vector<double> center_ranges_;

  void updateReceiverPositions(
    const cpm_lab_lab_msgs::msg::VehicleState::SharedPtr & vehicle_state_msg);

  float scaledRandomError(float real_world_error_impact);

  void updatePseudoRanges();

  Eigen::Vector3d updateGnssLocks();

public:
  RCLCPP_SMART_PTR_DEFINITIONS(ArtificialGnss)

  explicit ArtificialGnss(
    int number_of_frequency_bands = 3, float clock_error = 2.0f, float ionospheric_error = 5.0f,
    float ephemeris_error = 2.5f);

  Eigen::Vector3d generateGnssLock(
    const cpm_lab_lab_msgs::msg::VehicleState::SharedPtr & vehicle_state_msg);
};
}  // namespace ufil_examples_cpm_lab_adapter

#endif  // UFIL_EXAMPLES_CPM_LAB_ADAPTER__ARTIFICIAL_GNSS_HPP_
