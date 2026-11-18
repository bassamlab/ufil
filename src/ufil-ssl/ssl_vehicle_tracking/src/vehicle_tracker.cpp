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

#include "ssl_vehicle_tracking/vehicle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ssl_vehicle_tracking
{

namespace
{

// Probability models of the evaluation. Inputs: wheel age (0 = detected now, -n = n frames
// without detection), wheel deviation in search widths, number of missing wheel tracks.
double wheelExistenceProbability(double age) {return 0.5 * std::tanh((age + 4.0) * 0.8) + 0.5;}
double wheelPositionProbability(double deviation)
{
  return -0.5 * std::tanh((deviation - 1.0) * 3.0) + 0.5;
}
double tracksMissingProbability(double missing)
{
  return -0.5 * std::tanh((missing - 1.6) * 2.0) + 0.5;
}

/// Index of the first wheel assigned to the track, if any.
std::optional<std::size_t> wheelOf(const Vehicle & vehicle, const std::string & track)
{
  for (std::size_t i = 0; i < vehicle.wheel_tracks.size(); ++i) {
    if (vehicle.wheel_tracks[i] == track) {
      return i;
    }
  }
  return std::nullopt;
}

/// Tracks of the vehicle in the order of the frame.
std::vector<const WheelTrack *> vehicleTracks(
  const Vehicle & vehicle, const std::vector<WheelTrack> & tracks)
{
  std::vector<const WheelTrack *> result;
  for (const auto & track : tracks) {
    if (wheelOf(vehicle, track.id)) {
      result.push_back(&track);
    }
  }
  return result;
}

double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace

std::optional<std::string> validate(const TrackerConfig & config)
{
  if (!(std::isfinite(config.search_width) && config.search_width > 0.0)) {
    return "search_width must be > 0";
  }
  if (config.considered_rotation_pairs < 1) {
    return "considered_rotation_pairs must be >= 1";
  }
  if (!(config.dropping_threshold >= 0.0 && config.dropping_threshold <= 1.0)) {
    return "dropping_threshold must be in [0, 1]";
  }
  return std::nullopt;
}

std::pair<std::size_t, std::vector<std::optional<std::string>>> findRemainingTracks(
  const VehicleTemplate & vehicle_template, const std::vector<std::size_t> & known_wheels,
  const std::vector<std::string> & known_tracks, const std::vector<WheelTrack> & available,
  double search_width, std::size_t max_pairs)
{
  std::vector<std::size_t> found_wheels = known_wheels;
  std::vector<std::optional<std::string>> found_tracks(known_tracks.begin(), known_tracks.end());
  const auto is_found = [&found_tracks](const std::string & id) {
      return std::find(found_tracks.begin(), found_tracks.end(), id) != found_tracks.end();
    };

  for (std::size_t next = 0; next < vehicle_template.wheelCount(); ++next) {
    if (std::find(known_wheels.begin(), known_wheels.end(), next) != known_wheels.end()) {
      continue;
    }

    // Fit the template to the tracks found so far.
    std::vector<Point2D> points;
    std::vector<std::size_t> wheels;
    for (std::size_t i = 0; i < found_tracks.size(); ++i) {
      if (!found_tracks[i]) {
        continue;
      }
      for (const auto & track : available) {
        if (track.id == *found_tracks[i]) {
          points.push_back(track.position);
        }
      }
      wheels.push_back(found_wheels[i]);
    }
    const Pose2D pose = fitTemplate(vehicle_template, points, wheels, max_pairs);
    const Point2D expected = transformPoint(pose, vehicle_template.wheels()[next]);

    // Nearest track that is not yet used.
    const WheelTrack * nearest = nullptr;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const auto & track : available) {
      if (is_found(track.id)) {
        continue;
      }
      const double d = distance(track.position, expected);
      if (d < nearest_distance) {
        nearest_distance = d;
        nearest = &track;
      }
    }
    if (nearest != nullptr && nearest_distance <= search_width) {
      found_tracks.push_back(nearest->id);
    } else {
      found_tracks.push_back(std::nullopt);
    }
    found_wheels.push_back(next);
  }

  std::size_t found = 0;
  std::vector<std::optional<std::string>> result(found_tracks.size());
  for (std::size_t i = 0; i < found_tracks.size(); ++i) {
    if (found_tracks[i]) {
      ++found;
    }
    result[found_wheels[i]] = found_tracks[i];
  }
  return {found - known_tracks.size(), result};
}

VehicleTracker::VehicleTracker(const TrackerConfig & config, std::vector<VehicleTemplate> templates)
: config_(config), templates_(std::move(templates))
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument("invalid tracker configuration: " + *error);
  }
  if (templates_.empty()) {
    throw std::invalid_argument("invalid tracker configuration: no vehicle template");
  }
}

Vehicle VehicleTracker::makeVehicle(
  std::size_t template_index, std::vector<std::optional<std::string>> wheel_tracks) const
{
  const std::size_t wheels = templates_[template_index].wheelCount();
  Vehicle vehicle;
  vehicle.template_index = template_index;
  vehicle.wheel_tracks = std::move(wheel_tracks);
  vehicle.wheel_last_seen.assign(wheels, 0);
  vehicle.wheel_probabilities.assign(wheels, 1.0);
  return vehicle;
}

UpdateResult VehicleTracker::update(const std::vector<WheelTrack> & tracks)
{
  UpdateResult result;
  ++status_.frames;

  // 1. Evaluate and 2. unassign improbable wheels.
  for (auto & vehicle : vehicles_) {
    evaluate(vehicle, tracks);
    for (std::size_t i = 0; i < vehicle.wheel_tracks.size(); ++i) {
      if (vehicle.wheel_tracks[i] &&
        vehicle.wheel_probabilities[i] < config_.dropping_threshold)
      {
        vehicle.wheel_tracks[i].reset();
        ++result.dropped_wheels;
      }
    }
  }

  // 3. Reassemble vehicles with missing wheels.
  std::set<std::uint32_t> reassembled;
  for (auto & vehicle : vehicles_) {
    std::vector<std::string> keep;
    std::vector<std::size_t> keep_wheels;
    for (const auto * track : vehicleTracks(vehicle, tracks)) {
      keep.push_back(track->id);
      keep_wheels.push_back(*wheelOf(vehicle, track->id));
    }
    if (keep.size() < 2 || keep.size() >= vehicle.wheel_tracks.size()) {
      continue;
    }
    const auto [found, new_tracks] = findRemainingTracks(
      templates_[vehicle.template_index], keep_wheels, keep, tracks, config_.search_width,
      config_.considered_rotation_pairs);
    if (found > 0) {
      for (std::size_t i = 0; i < new_tracks.size(); ++i) {
        if (new_tracks[i]) {
          vehicle.wheel_tracks[i] = new_tracks[i];
        }
      }
      reassembled.insert(vehicle.id);
      result.reassembled.push_back(vehicle.id);
    }
  }

  // 4. Drop improbable vehicles.
  std::erase_if(
    vehicles_, [&](const Vehicle & vehicle) {
      if (vehicle.existence_probability < config_.dropping_threshold &&
      !reassembled.count(vehicle.id))
      {
        result.dropped.push_back(vehicle.id);
        return true;
      }
      return false;
    });

  // 5. Detect new vehicles.
  for (auto & vehicle : detect(tracks)) {
    vehicle.id = next_id_++;
    result.created.push_back(vehicle.id);
    vehicles_.push_back(std::move(vehicle));
  }

  // 6. Update the motion.
  for (auto & vehicle : vehicles_) {
    updateMotion(vehicle, tracks);
  }

  status_.vehicles_created += result.created.size();
  status_.vehicles_dropped += result.dropped.size();
  status_.reassemblies += result.reassembled.size();
  status_.wheels_dropped += result.dropped_wheels;
  status_.wheel_tracks = tracks.size();
  std::set<std::string> assigned;
  for (const auto & vehicle : vehicles_) {
    for (const auto & track : vehicleTracks(vehicle, tracks)) {
      assigned.insert(track->id);
    }
  }
  status_.unassigned_tracks = tracks.size() - assigned.size();
  return result;
}

void VehicleTracker::evaluate(Vehicle & vehicle, const std::vector<WheelTrack> & tracks) const
{
  const auto & vehicle_template = templates_[vehicle.template_index];
  const auto current = vehicleTracks(vehicle, tracks);
  const std::size_t wheels = vehicle.wheel_tracks.size();

  // Position probability of every wheel with a current track; NaN = not evaluated.
  std::vector<double> position(wheels, std::numeric_limits<double>::quiet_NaN());
  if (current.size() < 2) {
    position.assign(wheels, 0.0);
  } else {
    std::vector<Point2D> points;
    std::vector<std::size_t> wheel_map;
    for (const auto * track : current) {
      points.push_back(track->position);
      wheel_map.push_back(*wheelOf(vehicle, track->id));
    }
    const Pose2D pose = fitTemplate(
      vehicle_template, points, wheel_map, config_.considered_rotation_pairs);
    for (std::size_t i = 0; i < wheels; ++i) {
      const auto it = std::find(wheel_map.begin(), wheel_map.end(), i);
      if (it != wheel_map.end()) {
        const auto k = static_cast<std::size_t>(it - wheel_map.begin());
        const double deviation =
          distance(transformPoint(pose, vehicle_template.wheels()[i]), points[k]);
        position[i] = wheelPositionProbability(deviation / config_.search_width);
      }
    }
  }

  double sum = 0.0;
  for (std::size_t i = 0; i < wheels; ++i) {
    double age = static_cast<double>(vehicle.wheel_last_seen[i]);
    if (vehicle.wheel_tracks[i]) {
      for (const auto * track : current) {
        if (track->id == *vehicle.wheel_tracks[i]) {
          age = -static_cast<double>(track->frames_unseen);
          break;
        }
      }
    }
    vehicle.wheel_probabilities[i] =
      wheelExistenceProbability(age) * (std::isnan(position[i]) ? 1.0 : position[i]);
    sum += vehicle.wheel_probabilities[i];
  }

  const double missing =
    static_cast<double>(vehicle_template.wheelCount()) - static_cast<double>(current.size());
  vehicle.existence_probability =
    tracksMissingProbability(missing) * (sum / static_cast<double>(wheels));
}

std::vector<Vehicle> VehicleTracker::detect(const std::vector<WheelTrack> & tracks)
{
  std::set<std::string> assigned;
  for (const auto & vehicle : vehicles_) {
    for (const auto & track : vehicle.wheel_tracks) {
      if (track) {
        assigned.insert(*track);
      }
    }
  }
  std::vector<WheelTrack> unassigned;
  for (const auto & track : tracks) {
    if (!assigned.count(track.id) && track.frames_unseen <= 0) {
      unassigned.push_back(track);
    }
  }

  // Candidates: track pairs at the distance of the longest template wheel pair.
  std::vector<Vehicle> candidates;
  for (std::size_t t = 0; t < templates_.size(); ++t) {
    const auto & vehicle_template = templates_[t];
    if (unassigned.size() < vehicle_template.wheelCount()) {
      continue;
    }
    const auto [first, second] = vehicle_template.longestPair();
    const double target = vehicle_template.distance(first, second);
    for (const auto & p : unassigned) {
      for (const auto & q : unassigned) {
        const double d = distance(p.position, q.position);
        if (target - config_.search_width < d && d < target + config_.search_width) {
          auto [found, wheel_tracks] = findRemainingTracks(
            vehicle_template, {first, second}, {p.id, q.id}, unassigned, config_.search_width,
            config_.considered_rotation_pairs);
          candidates.push_back(makeVehicle(t, std::move(wheel_tracks)));
        }
      }
    }
  }

  for (auto & candidate : candidates) {
    evaluate(candidate, unassigned);
  }
  std::erase_if(
    candidates, [this](const Vehicle & v) {
      return !(v.existence_probability > config_.dropping_threshold);
    });
  std::stable_sort(
    candidates.begin(), candidates.end(), [](const Vehicle & a, const Vehicle & b) {
      return a.existence_probability > b.existence_probability;
    });

  // The most probable complete candidates whose tracks are still available.
  std::set<std::string> available;
  for (const auto & track : unassigned) {
    available.insert(track.id);
  }
  std::vector<Vehicle> accepted;
  for (auto & candidate : candidates) {
    const bool complete = std::all_of(
      candidate.wheel_tracks.begin(), candidate.wheel_tracks.end(),
      [&available](const auto & track) {return track && available.count(*track);});
    if (!complete) {
      continue;
    }
    for (const auto & track : candidate.wheel_tracks) {
      available.erase(*track);
    }
    accepted.push_back(std::move(candidate));
  }
  return accepted;
}

void VehicleTracker::updateMotion(Vehicle & vehicle, const std::vector<WheelTrack> & tracks) const
{
  const auto current = vehicleTracks(vehicle, tracks);
  for (const auto * track : current) {
    vehicle.wheel_last_seen[*wheelOf(vehicle, track->id)] = track->frames_unseen;
  }
  for (auto & last_seen : vehicle.wheel_last_seen) {
    --last_seen;
  }
  if (current.size() < 2) {
    return;
  }

  Point2D position;
  Point2D velocity;
  for (const auto * track : current) {
    position.x += track->position.x;
    position.y += track->position.y;
    velocity.x += track->velocity.x;
    velocity.y += track->velocity.y;
  }
  const auto n = static_cast<double>(current.size());
  vehicle.position = {position.x / n, position.y / n};
  vehicle.velocity = {velocity.x / n, velocity.y / n};
  // The heading follows the motion; a standing vehicle keeps its last heading.
  if (std::hypot(vehicle.velocity.x, vehicle.velocity.y) >= 0.01) {
    vehicle.yaw = std::atan2(vehicle.velocity.y, vehicle.velocity.x);
  }
}

TrackerStatus VehicleTracker::status() const
{
  TrackerStatus status = status_;
  status.vehicles = vehicles_.size();
  return status;
}

}  // namespace ssl_vehicle_tracking
