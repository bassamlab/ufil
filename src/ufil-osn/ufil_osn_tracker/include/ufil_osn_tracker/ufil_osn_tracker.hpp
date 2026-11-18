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


#ifndef UFIL_OSN_TRACKER__UFIL_OSN_TRACKER_HPP_
#define UFIL_OSN_TRACKER__UFIL_OSN_TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>

#include <ufil_object_tracking/types/id.hpp>
#include <ufil_object_tracking/types/occupancy_grid.hpp>
#include <ufil_object_tracking/types/sensor_fov.hpp>
#include <ufil_object_tracking/types/time.hpp>

#include "ufil_osn_tracker/config.hpp"
#include "ufil_osn_tracker/definitions.hpp"
#include "ufil_osn_tracker/visibility_control.hpp"

namespace ufil_osn_tracker
{

enum class UpdateResult
{
  /// The detections were processed.
  kUpdated,
  /// The time since the previous update exceeded time_jump_threshold. All tracks were dropped
  /// before the detections were processed.
  kResetAfterTimeJump,
  /// The timestamp is older than the previous update; the detections were ignored.
  kRejectedOutOfOrder,
};

/// Snapshot of the tracker state, intended for monitoring.
struct TrackerStatus
{
  std::uint64_t processed_updates = 0;
  std::uint64_t time_jump_resets = 0;
  std::uint64_t rejected_out_of_order = 0;
  /// Time between the two most recent update() calls [s].
  double last_time_delta = 0.0;

  std::size_t active_tracks = 0;
  /// Of the most recent update.
  std::size_t associations = 0;
  std::size_t unassociated_measurements = 0;
  /// Since construction or the last reset.
  std::size_t total_detector_associations = 0;
  std::size_t total_associator_associations = 0;

  bool occlusion_grid_available = false;
  bool fov_available = false;
};

/**
 * @brief Multi-object tracker for the dynamic point cloud of an outdoor sensor node.
 *
 * Clusters the point cloud into detections, associates them with the existing tracks and
 * estimates position, velocity, dimension, classification and existence of every object.
 * Tracks located in occupied cells of the occlusion grid are flagged as occluded.
 */
class UFIL_OSN_TRACKER_PUBLIC OsnObjectTracker
{
public:
  /// @throws std::invalid_argument if the configuration is invalid.
  explicit OsnObjectTracker(const TrackerConfig & config = TrackerConfig{});
  ~OsnObjectTracker();

  OsnObjectTracker(const OsnObjectTracker &) = delete;
  OsnObjectTracker & operator=(const OsnObjectTracker &) = delete;

  /// @throws std::invalid_argument if the configuration is invalid; the previous one is kept.
  void configure(const TrackerConfig & config);
  const TrackerConfig & config() const;

  UpdateResult update(std::set<Detection> && detections, ufil::type::Timestamp timestamp);

  /// Drop all tracks. Configuration, occlusion grid and field of view are kept.
  void reset();

  void setCurrentFOV(const ufil::type::SensorFOV & fov);
  void setOcclusionGrid(std::shared_ptr<const ufil::type::OccupancyGrid> grid);

  const std::map<ufil::type::Id, Track> & tracks() const;
  const std::set<Measurement> & unassociatedMeasurements() const;
  const std::map<ufil::type::Id, Measurement> & associations() const;

  TrackerStatus status() const;

private:
  struct Pipeline;

  TrackerConfig config_;
  std::unique_ptr<Pipeline> pipeline_;
  std::shared_ptr<const ufil::type::OccupancyGrid> occlusion_grid_;
  std::optional<ufil::type::SensorFOV> fov_;
  std::optional<ufil::type::Timestamp> last_update_time_;

  std::uint64_t processed_updates_ = 0;
  std::uint64_t time_jump_resets_ = 0;
  std::uint64_t rejected_out_of_order_ = 0;
  double last_time_delta_ = 0.0;
};

}  // namespace ufil_osn_tracker

#endif  // UFIL_OSN_TRACKER__UFIL_OSN_TRACKER_HPP_
