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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "ssl_vehicle_tracking/vehicle_tracker.hpp"

using ssl_vehicle_tracking::Point2D;
using ssl_vehicle_tracking::Pose2D;
using ssl_vehicle_tracking::TrackerConfig;
using ssl_vehicle_tracking::Vehicle;
using ssl_vehicle_tracking::VehicleTemplate;
using ssl_vehicle_tracking::VehicleTracker;
using ssl_vehicle_tracking::WheelTrack;

namespace
{

constexpr double kPi = 3.14159265358979323846;

VehicleTemplate loadCar()
{
  return ssl_vehicle_tracking::loadTemplate(
    std::string(TEMPLATE_DIR) + "/muCar_scaled_lab.json", "car");
}

/// Wheel tracks of a vehicle at the pose, moving with the velocity; ids are prefix + index.
std::vector<WheelTrack> wheelsAt(
  const VehicleTemplate & vehicle_template, const Pose2D & pose, const Point2D & velocity,
  const std::string & prefix)
{
  std::vector<WheelTrack> tracks;
  for (std::size_t i = 0; i < vehicle_template.wheelCount(); ++i) {
    WheelTrack track;
    track.id = prefix + std::to_string(i);
    track.position = ssl_vehicle_tracking::transformPoint(pose, vehicle_template.wheels()[i]);
    track.velocity = velocity;
    tracks.push_back(track);
  }
  return tracks;
}

std::set<std::string> assignedTracks(const Vehicle & vehicle)
{
  std::set<std::string> result;
  for (const auto & track : vehicle.wheel_tracks) {
    if (track) {
      result.insert(*track);
    }
  }
  return result;
}

}  // namespace

TEST(VehicleTracker, DetectsAVehicle)
{
  VehicleTracker tracker(TrackerConfig{}, {loadCar()});
  const Pose2D pose{5.0, 2.0, 0.3};
  const Point2D velocity{2.0 * std::cos(0.3), 2.0 * std::sin(0.3)};
  const auto tracks = wheelsAt(tracker.templates()[0], pose, velocity, "w");

  const auto result = tracker.update(tracks);
  ASSERT_EQ(result.created.size(), 1u);
  ASSERT_EQ(tracker.vehicles().size(), 1u);
  const auto & vehicle = tracker.vehicles()[0];
  EXPECT_EQ(vehicle.id, 1u);
  EXPECT_EQ(assignedTracks(vehicle), (std::set<std::string>{"w0", "w1", "w2", "w3"}));
  EXPECT_GT(vehicle.existence_probability, 0.9);

  double x = 0.0;
  double y = 0.0;
  for (const auto & track : tracks) {
    x += track.position.x / 4.0;
    y += track.position.y / 4.0;
  }
  EXPECT_NEAR(vehicle.position.x, x, 1e-9);
  EXPECT_NEAR(vehicle.position.y, y, 1e-9);
  EXPECT_NEAR(vehicle.yaw, 0.3, 1e-9);
  EXPECT_NEAR(vehicle.velocity.x, velocity.x, 1e-9);

  const auto status = tracker.status();
  EXPECT_EQ(status.vehicles, 1u);
  EXPECT_EQ(status.unassigned_tracks, 0u);
  EXPECT_EQ(status.vehicles_created, 1u);
}

TEST(VehicleTracker, KeepsTheHeadingOfAStandingVehicle)
{
  VehicleTracker tracker(TrackerConfig{}, {loadCar()});
  tracker.update(wheelsAt(tracker.templates()[0], {0.0, 0.0, 1.0}, {0.0, 1.0}, "w"));
  EXPECT_NEAR(tracker.vehicles()[0].yaw, kPi / 2.0, 1e-9);
  tracker.update(wheelsAt(tracker.templates()[0], {0.0, 0.0, 1.0}, {0.0, 0.0}, "w"));
  EXPECT_NEAR(tracker.vehicles()[0].yaw, kPi / 2.0, 1e-9);
}

TEST(VehicleTracker, DetectsSeparateVehiclesAndIgnoresStrayTracks)
{
  VehicleTracker tracker(TrackerConfig{}, {loadCar()});
  auto tracks = wheelsAt(tracker.templates()[0], {0.0, 0.0, 0.0}, {1.0, 0.0}, "a");
  const auto other = wheelsAt(tracker.templates()[0], {0.0, 10.0, 0.0}, {1.0, 0.0}, "b");
  tracks.insert(tracks.end(), other.begin(), other.end());
  WheelTrack stray;
  stray.id = "stray";
  stray.position = {20.0, 20.0};
  tracks.push_back(stray);

  tracker.update(tracks);
  ASSERT_EQ(tracker.vehicles().size(), 2u);
  std::set<std::set<std::string>> vehicles;
  for (const auto & vehicle : tracker.vehicles()) {
    vehicles.insert(assignedTracks(vehicle));
  }
  EXPECT_TRUE(vehicles.count({"a0", "a1", "a2", "a3"}));
  EXPECT_TRUE(vehicles.count({"b0", "b1", "b2", "b3"}));
  EXPECT_EQ(tracker.status().unassigned_tracks, 1u);
}

TEST(VehicleTracker, OnlyDetectsCompleteVehiclesFromCurrentTracks)
{
  VehicleTracker tracker(TrackerConfig{}, {loadCar()});
  auto tracks = wheelsAt(tracker.templates()[0], {0.0, 0.0, 0.0}, {1.0, 0.0}, "w");
  tracks.pop_back();
  tracker.update(tracks);
  EXPECT_TRUE(tracker.vehicles().empty());

  tracks = wheelsAt(tracker.templates()[0], {0.0, 0.0, 0.0}, {1.0, 0.0}, "w");
  tracks[2].frames_unseen = 1;
  tracker.update(tracks);
  EXPECT_TRUE(tracker.vehicles().empty());
}

TEST(VehicleTracker, ReassemblesAndDropsVehicles)
{
  VehicleTracker tracker(TrackerConfig{}, {loadCar()});
  const auto & vehicle_template = tracker.templates()[0];
  auto tracks = wheelsAt(vehicle_template, {0.0, 0.0, 0.0}, {1.0, 0.0}, "w");
  tracker.update(tracks);
  ASSERT_EQ(tracker.vehicles().size(), 1u);

  // One wheel track is lost: the vehicle survives with the remaining tracks.
  auto missing = tracks;
  const std::string lost = missing.back().id;
  missing.pop_back();
  auto result = tracker.update(missing);
  ASSERT_EQ(tracker.vehicles().size(), 1u);
  EXPECT_TRUE(result.reassembled.empty());

  // The wheel reappears as a new track at the expected position and is reassigned.
  auto renewed = tracks;
  renewed.back().id = "new";
  result = tracker.update(renewed);
  ASSERT_EQ(result.reassembled.size(), 1u);
  EXPECT_TRUE(assignedTracks(tracker.vehicles()[0]).count("new"));
  EXPECT_FALSE(assignedTracks(tracker.vehicles()[0]).count(lost));

  // All tracks are gone: the vehicle is dropped.
  result = tracker.update({});
  ASSERT_EQ(result.dropped.size(), 1u);
  EXPECT_EQ(result.dropped[0], 1u);
  EXPECT_TRUE(tracker.vehicles().empty());
  EXPECT_EQ(tracker.status().vehicles_dropped, 1u);
}

TEST(VehicleTracker, PrefersTheMatchingTemplate)
{
  auto motorcycle = ssl_vehicle_tracking::loadTemplate(
    std::string(TEMPLATE_DIR) + "/motorcycle_carla.json", "motorcycle");
  VehicleTracker tracker(TrackerConfig{}, {loadCar(), motorcycle});
  tracker.update(wheelsAt(tracker.templates()[1], {0.0, 0.0, 0.5}, {1.0, 0.5}, "m"));
  ASSERT_EQ(tracker.vehicles().size(), 1u);
  EXPECT_EQ(tracker.vehicles()[0].template_index, 1u);
}

TEST(FindRemainingTracks, FindsTheWheelsAtTheTemplatePositions)
{
  const auto vehicle_template = loadCar();
  auto tracks = wheelsAt(vehicle_template, {3.0, 1.0, -0.7}, {0.0, 0.0}, "w");
  tracks[1].position.x += 0.3;  // within the search width
  tracks[2].position.x += 2.0;  // too far
  const auto [found, result] = ssl_vehicle_tracking::findRemainingTracks(
    vehicle_template, {0, 3}, {"w0", "w3"}, tracks, 0.9, 2);
  EXPECT_EQ(found, 1u);
  ASSERT_EQ(result.size(), 4u);
  EXPECT_EQ(result[0], "w0");
  EXPECT_EQ(result[1], "w1");
  EXPECT_FALSE(result[2]);
  EXPECT_EQ(result[3], "w3");
}

TEST(VehicleTracker, InvalidConfigurationThrows)
{
  const auto expect_invalid = [](TrackerConfig config) {
      EXPECT_THROW(VehicleTracker(config, {loadCar()}), std::invalid_argument);
    };
  TrackerConfig config;
  config.search_width = 0.0;
  expect_invalid(config);
  config = TrackerConfig{};
  config.considered_rotation_pairs = 0;
  expect_invalid(config);
  config = TrackerConfig{};
  config.dropping_threshold = 1.5;
  expect_invalid(config);
  EXPECT_THROW(VehicleTracker(TrackerConfig{}, {}), std::invalid_argument);
}
