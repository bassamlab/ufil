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

#ifndef UFIL_EXAMPLES_PEDESTRIAN_TRACKER__PEDESTRIAN_TRACKER_HPP_
#define UFIL_EXAMPLES_PEDESTRIAN_TRACKER__PEDESTRIAN_TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>

#include <ufil_object_tracking/tracking/multi_target_tracker.hpp>
#include <ufil_object_tracking/types/classification.hpp>
#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/existence_probability.hpp>
#include <ufil_object_tracking/types/id.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_object_tracking/types/time.hpp>
#include <ufil_object_tracking/types/track.hpp>

#include "ufil_examples_pedestrian_tracker/visibility_control.hpp"

namespace ufil_examples_pedestrian_tracker
{

using State = ufil::type::state::PoseVelocity2D;
using Dimension = ufil::type::dimension::Dimension3D;
using Classification = ufil::type::classification::ObjectClassification;
using Control = ufil::type::control::None;
using Measurement = ufil::type::measurement::Pose2DWithDimension3D;
using ExistenceProbability = ufil::type::ExistenceProbability;
using Detection = Measurement;
using Track =
  ufil::type::Track<State, Dimension, Classification, ExistenceProbability, Control, Measurement>;

/// Configuration of the pedestrian tracker.
struct TrackerConfig
{
  /// Process noise of the random walk motion model, per axis.
  double process_noise = 10.0;
  /// Tracks without an associated detection for this long are deleted [s].
  double track_timeout = 1.0;
  /// A gap between two updates larger than this, or a jump back in time, resets the tracker [s].
  double max_time_gap = 1.0;
};

/// The reason why a configuration is invalid, or nothing if it is valid.
UFIL_EXAMPLES_PEDESTRIAN_TRACKER_PUBLIC
std::optional<std::string> validate(const TrackerConfig & config);

/// What happened during an update.
enum class UpdateEvent
{
  kUpdated,
  /// The timestamp jumped back or by more than max_time_gap; all tracks were dropped first.
  kResetAfterTimeJump,
};

/// Health information of the tracker, e.g. for diagnostics.
struct TrackerStatus
{
  std::size_t active_tracks = 0;
  std::uint64_t updates = 0;
  std::uint64_t time_jump_resets = 0;
  /// Timestamp of the last update, empty before the first one.
  std::optional<ufil::type::Timestamp> last_update;
};

/**
 * @brief Multi-target tracker for pedestrians with a random walk motion model.
 *
 * Detections are associated with the Wasserstein distance and the Hungarian algorithm;
 * unassociated detections start new tracks.
 */
class UFIL_EXAMPLES_PEDESTRIAN_TRACKER_PUBLIC PedestrianTracker
{
public:
  /// @throws std::invalid_argument if the configuration is invalid.
  explicit PedestrianTracker(const TrackerConfig & config = TrackerConfig());

  /// Updates the tracks with the detections of the given time.
  UpdateEvent update(std::set<Detection> && detections, ufil::type::Timestamp timestamp);

  /// The current tracks by id.
  const std::map<ufil::type::Id, Track> & tracks() const;

  const TrackerConfig & config() const;
  TrackerStatus status() const;

private:
  void reset();

  TrackerConfig config_;
  std::unique_ptr<ufil::tracking::MultiTargetTracker<Track, Detection>> tracker_;
  std::uint64_t updates_ = 0;
  std::uint64_t time_jump_resets_ = 0;
  std::optional<ufil::type::Timestamp> last_update_;
};

}  // namespace ufil_examples_pedestrian_tracker

#endif  // UFIL_EXAMPLES_PEDESTRIAN_TRACKER__PEDESTRIAN_TRACKER_HPP_
