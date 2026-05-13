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


#include "ufil_etsi/CamBuilder.hpp"

#include <utility>

#include "etsi_its_msgs_utils/cam_access.hpp"

namespace etsi_message_converter
{

CamBuilder::CamBuilder(StationID::_value_type station_id, VehicleRole::_value_type vehicle_role)
: its_pdu_header_(),
  basic_container_builder_(),
  high_frequency_container_builder_(),
  low_frequency_container_builder_(vehicle_role)
{
  // Create ITS PDU Header
  this->its_pdu_header_.protocol_version = 2;                       // Document V1.4.1
  this->its_pdu_header_.message_id = ItsPduHeader::MESSAGE_ID_CAM;  // CAM
  assert(StationID::MIN <= station_id && station_id <= StationID::MAX);
  this->its_pdu_header_.station_id.value = station_id;

  // Prepare new CAM
  this->reset();
}

void CamBuilder::reset()
{
  // Reset CAM
  this->cam_ = std::make_unique<CAM>();

  // Prepare next message
  this->cam_->header = this->its_pdu_header_;
  this->cam_->cam.cam_parameters.low_frequency_container_is_present =
    false;  // Default to not present, will be updated accordingly
  this->cam_->cam.cam_parameters.special_vehicle_container_is_present = false;
}

CamBuilder & CamBuilder::generationDeltaTime(const rclcpp::Time & measurement_time)
{
  // std::cout << "cam: " <<  *this->cam_.cam.generation_delta_time << std::endl;
  // Create Generation Delta Time
  etsi_its_cam_msgs::access::setGenerationDeltaTime(*this->cam_, measurement_time.nanoseconds());
  return *this;
}

CamBuilder & CamBuilder::basicContainer(
  const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped)
{
  this->cam_->cam.cam_parameters.basic_container =
    *this->basic_container_builder_.stationType(object_stamped)
    .referencePosition(object_stamped)
    .get();
  return *this;
}

CamBuilder & CamBuilder::highFrequencyContainer(
  const ufil_msgs::msg::ObjectStamped::UniquePtr & object_stamped)
{
  // Create container
  this->cam_->cam.cam_parameters.high_frequency_container =
    *this->high_frequency_container_builder_.heading(object_stamped)
    .dimension(object_stamped)
    .speed(object_stamped)
    .driveDirection(object_stamped)
    .acceleration(object_stamped)
    .yawRate(object_stamped)
    .get();
  return *this;
}

CamBuilder & CamBuilder::lowFrequencyContainer(const PathHistory::_array_type & path_points)
{
  // Set presence of container
  this->cam_->cam.cam_parameters.low_frequency_container_is_present = true;

  // Create container
  this->cam_->cam.cam_parameters.low_frequency_container =
    *this->low_frequency_container_builder_.pathHistory(path_points).get();
  return *this;
}

std::unique_ptr<CAM> CamBuilder::get()
{
  auto result = std::move(this->cam_);
  this->reset();
  return result;
}

}  // namespace etsi_message_converter
