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


#ifndef UFIL_ETSI__CAMBUILDER_HPP_
#define UFIL_ETSI__CAMBUILDER_HPP_

#include <memory>
#include "BasicContainerBuilder.hpp"
#include "HighFrequencyContainerBuilder.hpp"
#include <etsi_its_cam_msgs/msg/cam.hpp>
#include "LowFrequencyContainerBuilder.hpp"
#include <rclcpp/time.hpp>
#include <ufil_msgs/msg/object_stamped.hpp>

// using namespace etsi_its_cam_msgs::msg;
using etsi_its_cam_msgs::msg::CAM;
using etsi_its_cam_msgs::msg::ItsPduHeader;
using etsi_its_cam_msgs::msg::StationID;
using etsi_its_cam_msgs::msg::VehicleRole;
using etsi_its_cam_msgs::msg::PathHistory;

namespace etsi_message_converter
{

class CamBuilder {
private:
        // CAM
  std::unique_ptr<CAM> cam_ = nullptr;

        // ITS PDU Header
  ItsPduHeader its_pdu_header_;

  BasicContainerBuilder basic_container_builder_;
  HighFrequencyContainerBuilder high_frequency_container_builder_;
  LowFrequencyContainerBuilder low_frequency_container_builder_;

public:
  explicit CamBuilder(
    StationID::_value_type station_id,
    VehicleRole::_value_type vehicle_role);

  void reset();

  CamBuilder & generationDeltaTime(const rclcpp::Time & measurement_time);

  CamBuilder & basicContainer(const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped);

  CamBuilder & highFrequencyContainer(
    const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped);

  CamBuilder & lowFrequencyContainer(const PathHistory::_array_type & path_points);

  std::unique_ptr<CAM> get();
};

}  // namespace etsi_message_converter

#endif  // UFIL_ETSI__CAMBUILDER_HPP_
