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

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

#include "ssl_wheel_tracker/wheel_tracker.hpp"

using ssl_wheel_tracker::FrameResult;
using ssl_wheel_tracker::Point2D;
using ssl_wheel_tracker::Timestamp;
using ssl_wheel_tracker::TrackerConfig;
using ssl_wheel_tracker::TrackState;
using ssl_wheel_tracker::WheelTracker;

namespace
{

constexpr double kFramePeriod = 0.023;

Timestamp at(double seconds)
{
  return ufil::from_seconds<Timestamp>(seconds);
}

/// Feeds and processes one frame per call.
void track(WheelTracker & tracker, double time, const std::vector<Point2D> & detections)
{
  ASSERT_EQ(tracker.addFrame(at(time), detections), FrameResult::kAccepted);
  tracker.update();
}

}  // namespace

TEST(Assign, PicksTheGlobalOptimum)
{
  Eigen::MatrixXd distances(2, 2);
  // Greedy would give track 0 detection 0 (0.1) and track 1 detection 1 (1.9).
  distances << 0.1, 0.2,
    0.3, 1.9;
  const auto assignment = ssl_wheel_tracker::assign(distances, 2.0);
  ASSERT_TRUE(assignment[0] && assignment[1]);
  EXPECT_EQ(*assignment[0], 1u);
  EXPECT_EQ(*assignment[1], 0u);
}

TEST(Assign, RespectsTheGate)
{
  Eigen::MatrixXd distances(2, 1);
  distances << 2.0, 2.5;
  const auto assignment = ssl_wheel_tracker::assign(distances, 2.0);
  EXPECT_FALSE(assignment[0]);
  EXPECT_FALSE(assignment[1]);
}

TEST(Assign, MissingIsCheaperThanAFarPair)
{
  // Track 0 to detection 1 (1.5) + track 1 to detection 0 (1.5) costs more than
  // track 0 to detection 0 (0.1) + track 1 missed (2.0).
  Eigen::MatrixXd distances(2, 2);
  distances << 0.1, 1.5,
    1.5, std::numeric_limits<double>::infinity();
  const auto assignment = ssl_wheel_tracker::assign(distances, 2.0);
  ASSERT_TRUE(assignment[0]);
  EXPECT_EQ(*assignment[0], 0u);
  EXPECT_FALSE(assignment[1]);
}

TEST(Assign, MoreDetectionsThanTracks)
{
  Eigen::MatrixXd distances(1, 3);
  distances << 0.5, 0.2, 0.9;
  const auto assignment = ssl_wheel_tracker::assign(distances, 2.0);
  ASSERT_TRUE(assignment[0]);
  EXPECT_EQ(*assignment[0], 1u);
  EXPECT_TRUE(ssl_wheel_tracker::assign(Eigen::MatrixXd(0, 3), 2.0).empty());
  EXPECT_FALSE(ssl_wheel_tracker::assign(Eigen::MatrixXd(2, 0), 2.0)[1]);
}

TEST(KalmanFilter, PredictsWithConstantVelocity)
{
  TrackState state;
  state.stamp = at(1.0);
  state.mean << 1.0, 2.0, 3.0, -1.0;
  state.covariance.setZero();

  const auto prediction = ssl_wheel_tracker::predict(state, at(1.5), 0.04);
  EXPECT_NEAR(prediction.x(), 2.0, 1e-9);
  EXPECT_NEAR(prediction.y(), 2.5, 1e-9);
  EXPECT_FALSE(prediction.matched);
  // Q = q [[dt^3/3, dt^2/2], [dt^2/2, dt]] per axis.
  EXPECT_NEAR(prediction.covariance(0, 0), 0.04 * 0.125 / 3.0, 1e-12);
  EXPECT_NEAR(prediction.covariance(0, 1), 0.04 * 0.125, 1e-12);
  EXPECT_NEAR(prediction.covariance(3, 3), 0.04 * 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(prediction.covariance(0, 2), 0.0);
}

TEST(KalmanFilter, CorrectionWeightsPredictionAndMeasurement)
{
  TrackState prediction;
  prediction.mean << 0.0, 0.0, 0.0, 0.0;
  prediction.covariance = Eigen::Matrix4d::Identity() * 0.01;

  const auto state = ssl_wheel_tracker::correct(prediction, {1.0, -1.0}, 0.01);
  EXPECT_TRUE(state.matched);
  EXPECT_NEAR(state.x(), 0.5, 1e-9);
  EXPECT_NEAR(state.y(), -0.5, 1e-9);
  EXPECT_NEAR(state.covariance(0, 0), 0.005, 1e-12);
  EXPECT_NEAR(state.covariance(1, 1), 0.01, 1e-12);
}

TEST(WheelTracker, ConfirmsAfterTheConfiguredHits)
{
  WheelTracker tracker{TrackerConfig{}};
  for (int frame = 0; frame < 3; ++frame) {
    track(tracker, frame * kFramePeriod, {{1.0, 1.0}});
    EXPECT_TRUE(tracker.tracks().empty());
    EXPECT_EQ(tracker.tentativeTracks().size(), 1u);
  }
  ASSERT_EQ(tracker.addFrame(at(3 * kFramePeriod), {{1.0, 1.0}}), FrameResult::kAccepted);
  const auto result = tracker.update();
  EXPECT_EQ(result.frames, 1u);
  ASSERT_EQ(result.confirmed.size(), 1u);
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_TRUE(tracker.tentativeTracks().empty());

  const auto & wheel = tracker.tracks().front();
  EXPECT_EQ(wheel.id, 1u);
  EXPECT_EQ(wheel.hits, 4u);
  EXPECT_EQ(wheel.misses, 0u);
  EXPECT_EQ(wheel.history.size(), 4u);
  // The prior velocity of 0.35 m/s moves the estimate slightly.
  EXPECT_NEAR(wheel.state().x(), 1.0, 0.01);
}

TEST(WheelTracker, FollowsAMovingWheel)
{
  WheelTracker tracker{TrackerConfig{}};
  const Point2D velocity{3.0, -1.0};
  for (int frame = 0; frame < 60; ++frame) {
    const double t = frame * kFramePeriod;
    track(tracker, t, {{velocity.x * t, 5.0 + velocity.y * t}, {20.0, 20.0 + 0.5 * t}});
  }
  ASSERT_EQ(tracker.tracks().size(), 2u);
  const auto & wheel = tracker.tracks()[0];
  EXPECT_NEAR(wheel.state().vx(), velocity.x, 0.05);
  EXPECT_NEAR(wheel.state().vy(), velocity.y, 0.05);
  EXPECT_EQ(wheel.history.size(), TrackerConfig{}.history_length);
  EXPECT_EQ(tracker.status().tracks_confirmed, 2u);
  EXPECT_EQ(tracker.status().detections, 120u);
}

TEST(WheelTracker, KeepsPredictionsUntilDeletion)
{
  WheelTracker tracker{TrackerConfig{}};
  for (int frame = 0; frame < 5; ++frame) {
    track(tracker, frame * kFramePeriod, {{1.0, 1.0}});
  }
  ASSERT_EQ(tracker.tracks().size(), 1u);
  const std::size_t matched = tracker.tracks().front().history.size();

  // 0.25 s without detection: kept until the time since the last match exceeds it.
  int frame = 5;
  for (; (frame - 4) * kFramePeriod <= 0.25; ++frame) {
    track(tracker, frame * kFramePeriod, {});
    ASSERT_EQ(tracker.tracks().size(), 1u);
    const auto & wheel = tracker.tracks().front();
    EXPECT_EQ(wheel.misses, static_cast<std::size_t>(frame - 4));
    EXPECT_EQ(wheel.history.size(), matched + wheel.misses);
    EXPECT_FALSE(wheel.state().matched);
  }
  ASSERT_EQ(tracker.addFrame(at(frame * kFramePeriod), {}), FrameResult::kAccepted);
  const auto result = tracker.update();
  ASSERT_EQ(result.deleted.size(), 1u);
  EXPECT_EQ(result.deleted[0], 1u);
  EXPECT_TRUE(tracker.tracks().empty());

  // A match removes the predictions from the history.
  for (int i = 0; i < 4; ++i) {
    track(tracker, (++frame) * kFramePeriod, {{1.0, 1.0}});
  }
  track(tracker, (++frame) * kFramePeriod, {});
  track(tracker, (++frame) * kFramePeriod, {{1.0, 1.0}});
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_EQ(tracker.tracks().front().id, 2u);
  EXPECT_EQ(tracker.tracks().front().history.size(), 5u);
}

TEST(WheelTracker, TentativeTracksTimeOut)
{
  WheelTracker tracker{TrackerConfig{}};
  track(tracker, 0.0, {{1.0, 1.0}});
  track(tracker, 0.2, {});
  EXPECT_EQ(tracker.tentativeTracks().size(), 1u);
  track(tracker, 0.3, {});
  EXPECT_TRUE(tracker.tentativeTracks().empty());
}

TEST(WheelTracker, ConfirmedTracksTakeDetectionsFirst)
{
  WheelTracker tracker{TrackerConfig{}};
  for (int frame = 0; frame < 4; ++frame) {
    track(tracker, frame * kFramePeriod, {{1.0, 1.0}});
  }
  track(tracker, 4 * kFramePeriod, {{1.0, 1.0}, {1.5, 1.0}});
  ASSERT_EQ(tracker.tracks().size(), 1u);
  EXPECT_NEAR(tracker.tracks().front().state().x(), 1.0, 0.05);
  EXPECT_EQ(tracker.tentativeTracks().size(), 1u);
}

TEST(WheelTracker, MergesFramesWithTheSameStampAndSortsFrames)
{
  WheelTracker tracker{TrackerConfig{}};
  EXPECT_EQ(tracker.addFrame(at(0.1), {{1.0, 1.0}}), FrameResult::kAccepted);
  EXPECT_EQ(tracker.addFrame(at(0.0), {{1.0, 1.0}}), FrameResult::kAccepted);
  EXPECT_EQ(tracker.addFrame(at(0.1), {{5.0, 1.0}}), FrameResult::kAccepted);
  const auto result = tracker.update();
  EXPECT_EQ(result.frames, 2u);
  EXPECT_EQ(tracker.status().frames, 2u);
  EXPECT_EQ(tracker.status().detections, 3u);
  EXPECT_EQ(tracker.tentativeTracks().size(), 2u);
  EXPECT_EQ(*tracker.status().last_frame, at(0.1));
}

TEST(WheelTracker, DropsLateFramesAndResetsOnTimeJumps)
{
  WheelTracker tracker{TrackerConfig{}};
  for (int frame = 0; frame < 4; ++frame) {
    track(tracker, 10.0 + frame * kFramePeriod, {{1.0, 1.0}});
  }
  ASSERT_EQ(tracker.tracks().size(), 1u);

  EXPECT_EQ(tracker.addFrame(at(10.0 + 3 * kFramePeriod), {}), FrameResult::kLate);
  EXPECT_EQ(tracker.addFrame(at(10.0), {}), FrameResult::kLate);
  EXPECT_EQ(tracker.update().frames, 0u);
  EXPECT_EQ(tracker.status().late_frames, 2u);
  EXPECT_EQ(tracker.tracks().size(), 1u);

  EXPECT_EQ(tracker.addFrame(at(1.0), {{1.0, 1.0}}), FrameResult::kReset);
  EXPECT_TRUE(tracker.tracks().empty());
  EXPECT_EQ(tracker.update().frames, 1u);
  EXPECT_EQ(tracker.status().resets, 1u);
  EXPECT_EQ(tracker.tentativeTracks().size(), 1u);
}

TEST(WheelTracker, ConfirmationWithASingleHit)
{
  TrackerConfig config;
  config.confirmation_hits = 1;
  WheelTracker tracker{config};
  track(tracker, 0.0, {{1.0, 1.0}});
  EXPECT_EQ(tracker.tracks().size(), 1u);
}

TEST(WheelTracker, InvalidConfigurationThrows)
{
  const auto expect_invalid = [](auto modify) {
      TrackerConfig config;
      modify(config);
      EXPECT_THROW(WheelTracker{config}, std::invalid_argument);
    };
  expect_invalid([](TrackerConfig & c) {c.process_noise = 0.0;});
  expect_invalid([](TrackerConfig & c) {c.measurement_noise = -1.0;});
  expect_invalid([](TrackerConfig & c) {c.gate_distance = 0.0;});
  expect_invalid([](TrackerConfig & c) {c.confirmation_hits = 0;});
  expect_invalid([](TrackerConfig & c) {c.deletion_timeout = 0.0;});
  expect_invalid([](TrackerConfig & c) {c.history_length = 0;});
  expect_invalid(
    [](TrackerConfig & c) {c.initial_velocity.x = std::numeric_limits<double>::quiet_NaN();});
}
