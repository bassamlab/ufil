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


#ifndef UFIL_ETSI__HIGHFREQUENCYCONTAINERBUILDER_HPP_
#define UFIL_ETSI__HIGHFREQUENCYCONTAINERBUILDER_HPP_

#include <memory>
#include <etsi_its_cam_msgs/msg/high_frequency_container.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include "ufil_msgs/msg/object_stamped.hpp"

// Use using-declarations instead of using-directive
using etsi_its_cam_msgs::msg::HighFrequencyContainer;
using etsi_its_cam_msgs::msg::Curvature;
using etsi_its_cam_msgs::msg::CurvatureCalculationMode;

namespace etsi_message_converter
{

class HighFrequencyContainerBuilder
{
private:
  // Container
  std::unique_ptr<HighFrequencyContainer> high_frequency_container_ =
    std::make_unique<HighFrequencyContainer>();

  // Constant fields
  Curvature curvature_;
  CurvatureCalculationMode curvature_calculation_mode_;

public:
  HighFrequencyContainerBuilder();

  void reset();

  HighFrequencyContainerBuilder & dimension(
    const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped);

  HighFrequencyContainerBuilder & heading(
    const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped);

  HighFrequencyContainerBuilder & speed(
    const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped);

  HighFrequencyContainerBuilder & driveDirection(
    const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped);

  HighFrequencyContainerBuilder & acceleration(
    const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped);

  HighFrequencyContainerBuilder & yawRate(
    const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped);

  std::unique_ptr<HighFrequencyContainer> get();
};

}  // namespace etsi_message_converter

#endif  // UFIL_ETSI__HIGHFREQUENCYCONTAINERBUILDER_HPP_
