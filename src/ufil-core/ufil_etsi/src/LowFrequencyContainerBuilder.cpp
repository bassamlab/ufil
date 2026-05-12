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


#include <cassert>
#include "ufil_etsi/LowFrequencyContainerBuilder.hpp"

namespace etsi_message_converter
{

etsi_message_converter::LowFrequencyContainerBuilder::LowFrequencyContainerBuilder(
  VehicleRole::_value_type vehicle_role)
{
        // Vehicle Role (we don't have MIN and MAX defined unfortunately)
  assert(0 <= vehicle_role && vehicle_role <= 15);
  this->vehicle_role_.value = vehicle_role;

  // Exterior Lights (not supported)
  // If a vehicle is not equipped with a certain light
  // or if the light switch status information is not available,
  // the corresponding bit shall be set to 0.
  this->exterior_lights_.value.resize(1);
  this->exterior_lights_.value[0] = 0x00;
  this->exterior_lights_.bits_unused = 0;
}

void etsi_message_converter::LowFrequencyContainerBuilder::reset()
{
        // Reset container
  this->low_frequency_container_ = std::make_unique<LowFrequencyContainer>();

        // Prepare next container
  this->low_frequency_container_->choice =
    LowFrequencyContainer::CHOICE_BASIC_VEHICLE_CONTAINER_LOW_FREQUENCY;
  this->low_frequency_container_->basic_vehicle_container_low_frequency.vehicle_role =
    this->vehicle_role_;
  this->low_frequency_container_->basic_vehicle_container_low_frequency.exterior_lights =
    this->exterior_lights_;
}

LowFrequencyContainerBuilder & LowFrequencyContainerBuilder::pathHistory(
  const PathHistory::_array_type & path_points)
{
        // The list of path points may consist of up to 23 elements.
  assert(path_points.size() <= 23);
  this->low_frequency_container_->basic_vehicle_container_low_frequency.path_history.array =
    path_points;

  return *this;
}

std::unique_ptr<LowFrequencyContainer> etsi_message_converter::LowFrequencyContainerBuilder::get()
{
  auto result = std::move(this->low_frequency_container_);
  this->reset();
  return result;
}

}  // namespace etsi_message_converter
