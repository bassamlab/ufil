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

#include <chrono>
#include <set>
#include <stdexcept>
#include <vector>

#include "ufil_central_fusion/central_fusion.hpp"

using ufil_central_fusion::CentralFusion;
using ufil_central_fusion::FusionConfig;
using ufil_central_fusion::Measurement;
using ufil_central_fusion::MeasurementEvent;
using ufil_central_fusion::SensorConfig;

namespace
{

constexpr std::size_t kOsn = 0;
constexpr std::size_t kCam = 1;

std::vector<SensorConfig> sensors()
{
  SensorConfig osn{"osn", false, {}};
  SensorConfig cam{"cam", true, {}};
  return {osn, cam};
}

ufil::type::Timestamp at(double seconds)
{
  return ufil::from_seconds<ufil::type::Timestamp>(1000.0 + seconds);
}

/// A car at (x, y) driving east at 5 m/s.
std::set<Measurement> carAt(double x, double y)
{
  Measurement measurement;
  measurement.x() = x;
  measurement.y() = y;
  measurement.vx() = 5.0;
  measurement.vy() = 0.0;
  measurement.yaw() = 0.0;
  measurement.covariance() = Measurement::CovarianceMatrixType::Identity() * 0.1;
  measurement.dimension().length() = 4.5;
  measurement.dimension().width() = 1.8;
  measurement.dimension().height() = 1.5;
  measurement.dimension().covariance().setIdentity();
  measurement.existenceProbability() = 0.9;
  measurement.classification().car() = 1.0;
  return {measurement};
}

}  // namespace

TEST(CentralFusion, InvalidConfigurationIsRejected)
{
  FusionConfig config;
  config.buffer_duration = 0.0;
  EXPECT_THROW(CentralFusion(config, sensors()), std::invalid_argument);
  EXPECT_THROW(CentralFusion(FusionConfig{}, {}), std::invalid_argument);
}

TEST(CentralFusion, FusesMeasurementsIntoATrack)
{
  CentralFusion fusion(FusionConfig{}, sensors());
  for (int i = 0; i < 20; ++i) {
    const double t = 0.1 * i;
    ASSERT_EQ(fusion.addMeasurements(kOsn, carAt(5.0 * t, 0.0), at(t)),
      MeasurementEvent::kBuffered);
    fusion.update(at(t + 0.01));
  }
  const auto status = fusion.status();
  EXPECT_EQ(status.active_tracks, 1u);
  EXPECT_EQ(status.sensors[kOsn].received, 20u);
  EXPECT_EQ(status.sensors[kCam].received, 0u);
  EXPECT_EQ(status.updates, 20u);
  ASSERT_EQ(fusion.tracks().size(), 1u);
  EXPECT_NEAR(fusion.tracks().begin()->second.currentState().x(), 9.5, 1.0);
}

TEST(CentralFusion, ReplaysLateMeasurementsInOrder)
{
  CentralFusion fusion(FusionConfig{}, sensors());
  fusion.addMeasurements(kOsn, carAt(0.0, 0.0), at(0.0));
  fusion.addMeasurements(kOsn, carAt(1.0, 0.0), at(0.2));
  fusion.update(at(0.25));
  EXPECT_EQ(fusion.status().replays, 0u);

  // A CAM arrives late with an older stamp.
  EXPECT_EQ(fusion.addMeasurements(kCam, carAt(0.5, 0.0), at(0.1)), MeasurementEvent::kBuffered);
  fusion.update(at(0.3));
  const auto status = fusion.status();
  EXPECT_EQ(status.replays, 1u);
  EXPECT_EQ(status.buffered_measurement_sets, 3u);
  EXPECT_EQ(status.active_tracks, 1u);
}

TEST(CentralFusion, RejectsMeasurementsOlderThanBuffer)
{
  FusionConfig config;
  config.buffer_duration = 1.0;
  CentralFusion fusion(config, sensors());
  fusion.addMeasurements(kOsn, carAt(0.0, 0.0), at(5.0));
  EXPECT_EQ(
    fusion.addMeasurements(kOsn, carAt(0.0, 0.0), at(3.5)), MeasurementEvent::kRejectedTooOld);
  EXPECT_EQ(fusion.status().sensors[kOsn].rejected_too_old, 1u);
}

TEST(CentralFusion, TimeJumpResetsOrRejectsCam)
{
  CentralFusion fusion(FusionConfig{}, sensors());
  fusion.addMeasurements(kOsn, carAt(0.0, 0.0), at(0.0));
  fusion.update(at(0.05));
  ASSERT_EQ(fusion.tracks().size(), 1u);

  // A CAM with a broken stamp is dropped without touching the tracks.
  EXPECT_EQ(
    fusion.addMeasurements(kCam, carAt(0.0, 0.0), at(500.0)),
    MeasurementEvent::kRejectedTimeJump);
  EXPECT_EQ(fusion.tracks().size(), 1u);
  EXPECT_EQ(fusion.status().sensors[kCam].rejected_time_jump, 1u);

  // A jump in the data of a regular sensor (restarted recording) resets the fusion and keeps
  // the new measurements.
  EXPECT_EQ(
    fusion.addMeasurements(kOsn, carAt(0.0, 0.0), at(-100.0)),
    MeasurementEvent::kResetAfterTimeJump);
  EXPECT_TRUE(fusion.tracks().empty());
  EXPECT_EQ(fusion.status().time_jump_resets, 1u);
  EXPECT_EQ(fusion.status().buffered_measurement_sets, 1u);
  EXPECT_EQ(
    fusion.addMeasurements(kOsn, carAt(0.5, 0.0), at(-99.9)), MeasurementEvent::kBuffered);
}

TEST(CentralFusion, TracksSensorCapabilities)
{
  CentralFusion fusion(FusionConfig{}, sensors());
  EXPECT_FALSE(fusion.status().sensors[kOsn].has_fov);
  fusion.setSensorFov(kOsn, ufil::type::SensorFOV());
  fusion.setOcclusionAvailable(kOsn);
  const auto status = fusion.status();
  EXPECT_TRUE(status.sensors[kOsn].has_fov);
  EXPECT_TRUE(status.sensors[kOsn].occlusion_available);
  EXPECT_FALSE(status.sensors[kCam].has_fov);
  EXPECT_EQ(status.sensors[kCam].name, "cam");
}
