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

#ifndef UFIL_ETSI__VEHICLE_CAM_RECEIVER_HPP_
#define UFIL_ETSI__VEHICLE_CAM_RECEIVER_HPP_

#include <cstdint>
#include <optional>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/types.hpp"
#include "ufil_etsi/visibility_control.hpp"

namespace ufil_etsi
{

/// Configuration of the VehicleCamReceiver.
struct VehicleCamReceiverConfig
{
  /// Time shift of the sender (see VehicleCamGeneratorConfig::time_shift). The reception time
  /// is shifted by it before the generation delta time is resolved, the result is shifted back.
  Duration time_shift{0};
};

/// Outcome of VehicleCamReceiver::receive().
enum class ReceptionEvent
{
  kConverted,
  kNotAVehicle,  ///< Not a vehicle CAM (e.g. pedestrian or road side unit), ignored.
  kPositionUnavailable,  ///< Reference position unavailable or invalid, dropped.
  kInvalidTime,  ///< The shifted reception time is before 2004, dropped.
};

struct ReceptionResult
{
  ReceptionEvent event = ReceptionEvent::kConverted;
  /// The following members are only valid if event is kConverted.
  Object object;  ///< In UTM coordinates of zone.
  UtmZone zone;
  Time stamp{0};  ///< Generation time of the CAM.
};

/// Counters of the receiver.
struct VehicleCamReceiverStatus
{
  std::uint64_t received = 0;
  std::uint64_t converted = 0;
  std::uint64_t not_a_vehicle = 0;
  std::uint64_t position_unavailable = 0;
  std::uint64_t invalid_time = 0;
  /// Clock time of the last received CAM, if any.
  std::optional<Time> last_reception_time;
};

/// Converts received vehicle CAMs into objects (see decodeVehicleCam) and counts the results.
class UFIL_ETSI_PUBLIC VehicleCamReceiver
{
public:
  explicit VehicleCamReceiver(VehicleCamReceiverConfig config = {});

  /// Converts a CAM received at the clock time now.
  ReceptionResult receive(const Cam & cam, Time now);

  const VehicleCamReceiverConfig & config() const {return config_;}
  const VehicleCamReceiverStatus & status() const {return status_;}

private:
  VehicleCamReceiverConfig config_;
  VehicleCamReceiverStatus status_;
};

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__VEHICLE_CAM_RECEIVER_HPP_
