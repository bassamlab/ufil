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

#include "ufil_etsi/vehicle_cam_receiver.hpp"

#include <utility>

#include "ufil_etsi/cam_decoding.hpp"
#include "ufil_etsi/its_time.hpp"

namespace ufil_etsi
{

VehicleCamReceiver::VehicleCamReceiver(VehicleCamReceiverConfig config)
: config_(std::move(config))
{
}

ReceptionResult VehicleCamReceiver::receive(const Cam & cam, Time now)
{
  ++status_.received;
  status_.last_reception_time = now;

  ReceptionResult result;
  auto decoded = decodeVehicleCam(cam);
  switch (decoded.status) {
    case DecodeStatus::kNotAVehicle:
      ++status_.not_a_vehicle;
      result.event = ReceptionEvent::kNotAVehicle;
      return result;
    case DecodeStatus::kPositionUnavailable:
      ++status_.position_unavailable;
      result.event = ReceptionEvent::kPositionUnavailable;
      return result;
    case DecodeStatus::kOk:
      break;
  }

  // The generation delta time is resolved in the shifted time of the sender.
  const auto generation_time =
    unixTimeFromGenerationDeltaTime(cam.generation_delta_time, now + config_.time_shift);
  if (!generation_time) {
    ++status_.invalid_time;
    result.event = ReceptionEvent::kInvalidTime;
    return result;
  }

  ++status_.converted;
  result.event = ReceptionEvent::kConverted;
  result.object = std::move(decoded.object);
  result.zone = decoded.zone;
  result.stamp = *generation_time - config_.time_shift;
  return result;
}

}  // namespace ufil_etsi
