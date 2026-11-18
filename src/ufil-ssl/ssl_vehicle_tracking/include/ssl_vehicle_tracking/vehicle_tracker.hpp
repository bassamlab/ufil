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

#ifndef SSL_VEHICLE_TRACKING__VEHICLE_TRACKER_HPP_
#define SSL_VEHICLE_TRACKING__VEHICLE_TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ssl_vehicle_tracking/types.hpp"
#include "ssl_vehicle_tracking/vehicle_template.hpp"
#include "ssl_vehicle_tracking/visibility_control.hpp"

namespace ssl_vehicle_tracking
{

/// Configuration of the VehicleTracker.
struct TrackerConfig
{
  /// Maximum deviation of a wheel track from its expected position [m].
  double search_width = 0.9;
  /// Number of template wheel pairs (longest first) used to estimate the vehicle rotation.
  std::size_t considered_rotation_pairs = 2;
  /// Wheels and vehicles whose probability falls below this value are dropped [0, 1].
  double dropping_threshold = 0.05;
};

/// @return a description of the first invalid value, or std::nullopt if the configuration is
///         valid.
SSL_VEHICLE_TRACKING_PUBLIC std::optional<std::string> validate(const TrackerConfig & config);

/// Result of VehicleTracker::update().
struct UpdateResult
{
  /// Ids of the vehicles detected in this frame.
  std::vector<std::uint32_t> created;
  /// Ids of the vehicles dropped in this frame.
  std::vector<std::uint32_t> dropped;
  /// Ids of the vehicles whose missing wheels were reassigned in this frame.
  std::vector<std::uint32_t> reassembled;
  /// Number of wheel tracks unassigned from vehicles in this frame.
  std::size_t dropped_wheels = 0;
};

/// Counters and state of the tracker for diagnostics.
struct TrackerStatus
{
  std::size_t vehicles = 0;
  /// Wheel tracks of the last frame.
  std::size_t wheel_tracks = 0;
  /// Wheel tracks of the last frame that are not assigned to a vehicle.
  std::size_t unassigned_tracks = 0;
  std::uint64_t frames = 0;
  std::uint64_t vehicles_created = 0;
  std::uint64_t vehicles_dropped = 0;
  std::uint64_t reassemblies = 0;
  std::uint64_t wheels_dropped = 0;
};

/**
 * @brief Assembles vehicles from wheel tracks of the sensitive surface layer (SSL) and tracks
 *        them.
 *
 * Every frame (the latest state of all wheel tracks)
 * 1. evaluates the existing vehicles: every wheel gets the product of a track existence
 *    probability (from its frames without detection) and a position probability (deviation
 *    from the fitted template); the vehicle gets the product of a probability from the number of
 *    missing wheel tracks and the mean wheel probability,
 * 2. unassigns wheels below the dropping threshold,
 * 3. reassembles vehicles with at least two but not all wheel tracks by searching the missing
 *    wheels at the positions of the fitted template,
 * 4. drops vehicles below the dropping threshold unless they were just reassembled,
 * 5. detects new vehicles among the unassigned, currently detected wheel tracks: every track
 *    pair whose distance matches the longest wheel pair of a template is a candidate, the other
 *    wheels are searched at the template positions; complete candidates above the dropping
 *    threshold are accepted, the most probable first, without sharing tracks,
 * 6. updates position, velocity and yaw of every vehicle with at least two current wheel tracks.
 *
 * The class is not thread-safe.
 */
class SSL_VEHICLE_TRACKING_PUBLIC VehicleTracker
{
public:
  /// @throws std::invalid_argument if the configuration is invalid or no template is given.
  VehicleTracker(const TrackerConfig & config, std::vector<VehicleTemplate> templates);

  /// Processes one frame with the latest state of every wheel track.
  UpdateResult update(const std::vector<WheelTrack> & tracks);

  const std::vector<Vehicle> & vehicles() const {return vehicles_;}
  const std::vector<VehicleTemplate> & templates() const {return templates_;}
  const TrackerConfig & config() const {return config_;}
  TrackerStatus status() const;

private:
  void evaluate(Vehicle & vehicle, const std::vector<WheelTrack> & tracks) const;
  std::vector<Vehicle> detect(const std::vector<WheelTrack> & tracks);
  void updateMotion(Vehicle & vehicle, const std::vector<WheelTrack> & tracks) const;
  Vehicle makeVehicle(
    std::size_t template_index, std::vector<std::optional<std::string>> wheel_tracks) const;

  TrackerConfig config_;
  std::vector<VehicleTemplate> templates_;
  std::vector<Vehicle> vehicles_;
  std::uint32_t next_id_ = 1;
  TrackerStatus status_;
};

/**
 * @brief Searches the template wheels that are not yet known among the available tracks.
 *
 * The wheels are searched in ascending order. For each, the template is fitted to the tracks
 * found so far and the nearest available track within the search width of the expected
 * position is taken.
 *
 * @param known_wheels template wheel of each known track (at least two)
 * @param known_tracks ids of the known tracks; they must be available
 * @return the number of found tracks and the track of every template wheel
 */
SSL_VEHICLE_TRACKING_PUBLIC std::pair<std::size_t, std::vector<std::optional<std::string>>>
findRemainingTracks(
  const VehicleTemplate & vehicle_template, const std::vector<std::size_t> & known_wheels,
  const std::vector<std::string> & known_tracks, const std::vector<WheelTrack> & available,
  double search_width, std::size_t max_pairs);

}  // namespace ssl_vehicle_tracking

#endif  // SSL_VEHICLE_TRACKING__VEHICLE_TRACKER_HPP_
