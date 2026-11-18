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

#ifndef UFIL_CENTRAL_FUSION__CENTRAL_FUSION_HPP_
#define UFIL_CENTRAL_FUSION__CENTRAL_FUSION_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <ufil_object_tracking/types/id.hpp>
#include <ufil_object_tracking/types/sensor_fov.hpp>
#include <ufil_object_tracking/types/time.hpp>

#include "ufil_central_fusion/config.hpp"
#include "ufil_central_fusion/definitions.hpp"
#include "ufil_central_fusion/visibility_control.hpp"

namespace ufil_central_fusion
{

enum class MeasurementEvent
{
  /// The measurements were buffered and will be fused with the next update().
  kBuffered,
  /// The stamp jumped by more than time_jump_threshold; the fusion was reset and the
  /// measurements were buffered.
  kResetAfterTimeJump,
  /// The stamp of a CAM sensor jumped by more than time_jump_threshold (e.g. a broken
  /// packet); the measurements were dropped.
  kRejectedTimeJump,
  /// The measurements are older than the buffer; they were dropped.
  kRejectedTooOld,
};

struct SensorStatus
{
  std::string name;
  std::uint64_t received = 0;
  std::uint64_t rejected_too_old = 0;
  std::uint64_t rejected_time_jump = 0;
  /// Stamp of the latest accepted measurements.
  std::optional<ufil::type::Timestamp> last_measurement_time;
  bool has_fov = false;
  bool occlusion_available = false;
};

/// Snapshot of the fusion state, intended for monitoring.
struct FusionStatus
{
  std::vector<SensorStatus> sensors;
  std::size_t active_tracks = 0;
  std::size_t buffered_measurement_sets = 0;
  std::uint64_t updates = 0;
  /// Updates that replayed buffered measurements because data arrived out of order.
  std::uint64_t replays = 0;
  std::uint64_t time_jump_resets = 0;
};

/**
 * @brief Fuses the object lists of several sensors into one set of tracks.
 *
 * Measurements are buffered for buffer_duration. When measurements arrive that are older than
 * already fused ones, the tracks are rolled back and the buffer is replayed in stamp order, so
 * the result does not depend on the arrival order.
 */
class UFIL_CENTRAL_FUSION_PUBLIC CentralFusion
{
public:
  /// @throws std::invalid_argument if the configuration is invalid.
  CentralFusion(const FusionConfig & config, const std::vector<SensorConfig> & sensors);
  ~CentralFusion();

  CentralFusion(const CentralFusion &) = delete;
  CentralFusion & operator=(const CentralFusion &) = delete;

  const FusionConfig & config() const;
  const std::vector<SensorConfig> & sensors() const;

  /// Buffer the measurements of a sensor, given in the fusion frame.
  MeasurementEvent addMeasurements(
    std::size_t sensor, std::set<Measurement> measurements, ufil::type::Timestamp stamp);

  /// Field of view of a sensor, given in the fusion frame.
  void setSensorFov(std::size_t sensor, const ufil::type::SensorFOV & fov);
  /// The sensor provides an occlusion grid, so missing detections may be due to occlusion.
  void setOcclusionAvailable(std::size_t sensor);

  /// Fuse buffered measurements and predict the tracks to the given time.
  void update(ufil::type::Timestamp now);

  /// Drop all tracks and buffered measurements.
  void reset();

  const std::map<ufil::type::Id, Track> & tracks() const;

  FusionStatus status() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ufil_central_fusion

#endif  // UFIL_CENTRAL_FUSION__CENTRAL_FUSION_HPP_
