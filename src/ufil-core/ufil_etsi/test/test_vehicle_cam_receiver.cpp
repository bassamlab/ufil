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

#include <gtest/gtest.h>

#include <chrono>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/cam_encoding.hpp"
#include "ufil_etsi/its_time.hpp"
#include "ufil_etsi/vehicle_cam_receiver.hpp"

// Core library tests, no ROS 2 required.

using std::chrono::milliseconds;
using std::chrono::seconds;
using ufil_etsi::Cam;
using ufil_etsi::ReceptionEvent;
using ufil_etsi::Time;
using ufil_etsi::VehicleCamReceiver;

namespace
{

Cam carCam(Time generation_time)
{
  ufil_etsi::Object object;
  object.id = 3;
  object.x = 691000.0;
  object.y = 5335000.0;
  object.classification[0] = 1.0;
  Cam cam;
  cam.station_id = object.id;
  cam.generation_delta_time = ufil_etsi::generationDeltaTime(generation_time).value_or(0);
  cam.station_type = ufil_etsi::encodeStationType(object);
  cam.reference_position =
    *ufil_etsi::encodeReferencePosition(object, ufil_etsi::UtmZone{32, true});
  cam.vehicle_high_frequency = ufil_etsi::encodeVehicleHighFrequency(object);
  return cam;
}

}  // namespace

TEST(VehicleCamReceiver, ConvertsVehicleCams)
{
  VehicleCamReceiver receiver;
  const Time generation = seconds(1704067200) + milliseconds(250);
  const auto result = receiver.receive(carCam(generation), generation + milliseconds(30));
  ASSERT_EQ(result.event, ReceptionEvent::kConverted);
  EXPECT_EQ(result.stamp, generation);
  EXPECT_EQ(result.zone, (ufil_etsi::UtmZone{32, true}));
  EXPECT_NEAR(result.object.x, 691000.0, 0.02);
  EXPECT_EQ(result.object.id, 3u);
  EXPECT_EQ(receiver.status().received, 1u);
  EXPECT_EQ(receiver.status().converted, 1u);
  EXPECT_EQ(receiver.status().last_reception_time, generation + milliseconds(30));
}

TEST(VehicleCamReceiver, TimeShiftIsAppliedAndRemoved)
{
  // Sender and receiver run on a simulated clock starting at 0, shifted into 2024.
  ufil_etsi::VehicleCamReceiverConfig config;
  config.time_shift = seconds(1704067200);
  VehicleCamReceiver receiver(config);
  const Time generation = seconds(100) + milliseconds(10);
  const auto result =
    receiver.receive(carCam(generation + config.time_shift), generation + milliseconds(20));
  ASSERT_EQ(result.event, ReceptionEvent::kConverted);
  EXPECT_EQ(result.stamp, generation);

  // Without the shift the time cannot be resolved.
  VehicleCamReceiver unshifted;
  EXPECT_EQ(
    unshifted.receive(carCam(generation + config.time_shift), generation).event,
    ReceptionEvent::kInvalidTime);
  EXPECT_EQ(unshifted.status().invalid_time, 1u);
}

TEST(VehicleCamReceiver, CountsIgnoredAndDroppedCams)
{
  VehicleCamReceiver receiver;
  const Time now = seconds(1704067200);
  Cam rsu = carCam(now);
  rsu.station_type = ufil_etsi::etsi::station_type::kRoadSideUnit;
  EXPECT_EQ(receiver.receive(rsu, now).event, ReceptionEvent::kNotAVehicle);
  Cam no_position = carCam(now);
  no_position.reference_position.longitude = ufil_etsi::etsi::kLongitudeUnavailable;
  EXPECT_EQ(receiver.receive(no_position, now).event, ReceptionEvent::kPositionUnavailable);
  EXPECT_EQ(receiver.status().received, 2u);
  EXPECT_EQ(receiver.status().not_a_vehicle, 1u);
  EXPECT_EQ(receiver.status().position_unavailable, 1u);
  EXPECT_EQ(receiver.status().converted, 0u);
}
