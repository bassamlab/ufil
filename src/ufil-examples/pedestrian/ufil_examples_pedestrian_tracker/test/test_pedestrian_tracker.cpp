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

#include <set>
#include <stdexcept>

#include "ufil_examples_pedestrian_tracker/pedestrian_tracker.hpp"

using ufil_examples_pedestrian_tracker::Detection;
using ufil_examples_pedestrian_tracker::PedestrianTracker;
using ufil_examples_pedestrian_tracker::TrackerConfig;
using ufil_examples_pedestrian_tracker::UpdateEvent;

namespace
{

Detection detection(float x, float y)
{
  Detection result;
  result.x() = x;
  result.y() = y;
  result.yaw() = 0.0F;
  result.covariance().setIdentity();
  result.dimension().length() = 1.0F;
  result.dimension().width() = 1.0F;
  result.dimension().height() = 1.0F;
  result.dimension().covariance().setIdentity();
  return result;
}

ufil::type::Timestamp at(double seconds)
{
  return ufil::from_seconds<ufil::type::Timestamp>(1000.0 + seconds);
}

}  // namespace

TEST(PedestrianTrackerTest, TracksSeparatedPedestrians)
{
  PedestrianTracker tracker;
  EXPECT_FALSE(tracker.status().last_update);
  for (int step = 0; step < 20; ++step) {
    const float offset = 0.1F * static_cast<float>(step);
    std::set<Detection> detections{detection(offset, 0.0F), detection(offset, 20.0F)};
    EXPECT_EQ(tracker.update(std::move(detections), at(0.1 * step)), UpdateEvent::kUpdated);
  }
  EXPECT_EQ(tracker.tracks().size(), 2u);
  EXPECT_EQ(tracker.status().active_tracks, 2u);
  EXPECT_EQ(tracker.status().updates, 20u);
  EXPECT_EQ(tracker.status().time_jump_resets, 0u);
  ASSERT_TRUE(tracker.status().last_update);

  bool near_second = false;
  for (const auto & [id, track] : tracker.tracks()) {
    near_second |= std::abs(track.currentState().y() - 20.0F) < 1.0F;
  }
  EXPECT_TRUE(near_second);
}

TEST(PedestrianTrackerTest, DeletesTracksAfterTheTimeout)
{
  TrackerConfig config;
  config.track_timeout = 0.5;
  PedestrianTracker tracker(config);
  for (int step = 0; step < 5; ++step) {
    tracker.update({detection(0.0F, 0.0F)}, at(0.1 * step));
  }
  ASSERT_EQ(tracker.tracks().size(), 1u);
  for (int step = 5; step < 12; ++step) {
    tracker.update({}, at(0.1 * step));
  }
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST(PedestrianTrackerTest, ResetsAfterTimeJumps)
{
  PedestrianTracker tracker;
  tracker.update({detection(0.0F, 0.0F)}, at(0.0));
  tracker.update({detection(0.0F, 0.0F)}, at(0.1));
  ASSERT_EQ(tracker.tracks().size(), 1u);

  // Back in time: the old track is dropped, the new detection starts a new track.
  EXPECT_EQ(
    tracker.update({detection(5.0F, 5.0F)}, at(0.05)), UpdateEvent::kResetAfterTimeJump);
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_NEAR(tracker.tracks().begin()->second.currentState().x(), 5.0F, 1e-3F);

  // Too far forward.
  EXPECT_EQ(tracker.update({}, at(2.0)), UpdateEvent::kResetAfterTimeJump);
  EXPECT_EQ(tracker.status().time_jump_resets, 2u);
}

TEST(PedestrianTrackerTest, RejectsInvalidConfiguration)
{
  TrackerConfig config;
  EXPECT_FALSE(ufil_examples_pedestrian_tracker::validate(config));
  config.process_noise = 0.0;
  EXPECT_TRUE(ufil_examples_pedestrian_tracker::validate(config));
  EXPECT_THROW(PedestrianTracker{config}, std::invalid_argument);

  config = TrackerConfig();
  config.track_timeout = -1.0;
  EXPECT_THROW(PedestrianTracker{config}, std::invalid_argument);

  config = TrackerConfig();
  config.max_time_gap = 0.0;
  EXPECT_THROW(PedestrianTracker{config}, std::invalid_argument);
}
