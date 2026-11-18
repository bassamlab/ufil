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

#include <numbers>  // cpplint treats this C++20 header as a C system header

#include <chrono>
#include <optional>
#include <stdexcept>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/vehicle_cam_generator.hpp"

// Core library tests, no ROS 2 required.

using std::chrono::milliseconds;
using std::chrono::seconds;
using ufil_etsi::GenerationEvent;
using ufil_etsi::GenerationTrigger;
using ufil_etsi::Object;
using ufil_etsi::Time;
using ufil_etsi::VehicleCamGenerator;
using ufil_etsi::VehicleCamGeneratorConfig;

namespace
{

/// 2024-01-01T00:00:00Z
const Time kStart = seconds(1704067200);

Object car(double x, double yaw = 0.0, double speed = 0.0)
{
  Object object;
  object.id = 7;
  object.x = 691000.0 + x;
  object.y = 5335000.0;
  object.yaw = yaw;
  object.v_x = speed;
  object.classification[0] = 1.0;
  return object;
}

class VehicleCamGeneratorTest : public ::testing::Test
{
protected:
  VehicleCamGeneratorConfig config_;
  VehicleCamGenerator generator_{config_};

  /// Sets the object at time now and checks the generation conditions.
  ufil_etsi::GenerationResult step(const Object & object, Time now)
  {
    generator_.setObject(object, now, now);
    return generator_.check(now);
  }
};

}  // namespace

TEST(VehicleCamGeneratorConfigTest, Validation)
{
  VehicleCamGeneratorConfig config;
  EXPECT_FALSE(ufil_etsi::validate(config).has_value());
  config.vehicle_role = 16;
  EXPECT_TRUE(ufil_etsi::validate(config).has_value());
  EXPECT_THROW(VehicleCamGenerator{config}, std::invalid_argument);
  config.vehicle_role = 0;
  config.utm_zone.zone = 0;
  EXPECT_TRUE(ufil_etsi::validate(config).has_value());
}

TEST_F(VehicleCamGeneratorTest, FirstObjectIsSentImmediately)
{
  EXPECT_EQ(generator_.check(kStart).event, GenerationEvent::kNoObject);

  const auto result = step(car(0.0), kStart);
  ASSERT_EQ(result.event, GenerationEvent::kGenerated);
  EXPECT_EQ(result.trigger, GenerationTrigger::kFirst);
  const auto & cam = *result.cam;
  EXPECT_EQ(cam.station_id, 7u);
  EXPECT_EQ(cam.station_type, ufil_etsi::etsi::station_type::kPassengerCar);
  ASSERT_TRUE(cam.vehicle_high_frequency.has_value());
  // The first CAM has the low-frequency container with an empty path history.
  ASSERT_TRUE(cam.vehicle_low_frequency.has_value());
  EXPECT_EQ(cam.vehicle_low_frequency->vehicle_role, 0);
  EXPECT_TRUE(cam.vehicle_low_frequency->path_history.empty());

  // The object is used only once.
  EXPECT_EQ(generator_.check(kStart + milliseconds(50)).event, GenerationEvent::kNoObject);
}

TEST_F(VehicleCamGeneratorTest, StandingVehicleSendsEverySecond)
{
  step(car(0.0), kStart);
  Time now = kStart;
  int generated = 0;
  for (int i = 1; i <= 60; ++i) {  // 3 s
    now += milliseconds(50);
    const auto result = step(car(0.0), now);
    if (result.event == GenerationEvent::kGenerated) {
      EXPECT_EQ(result.trigger, GenerationTrigger::kTimeout);
      EXPECT_EQ(i % 20, 0) << "CAM after " << i * 50 << " ms";
      ++generated;
    } else {
      EXPECT_EQ(result.event, GenerationEvent::kNotDue);
    }
  }
  EXPECT_EQ(generated, 3);
  EXPECT_EQ(generator_.status().triggered_by_timeout, 3u);
  EXPECT_EQ(generator_.status().generation_interval, VehicleCamGenerator::kGenCamMax);
}

TEST_F(VehicleCamGeneratorTest, DynamicsTriggerAfterAtLeast100Ms)
{
  step(car(0.0), kStart);
  // Moved 5 m, but only 50 ms after the last CAM.
  EXPECT_EQ(step(car(5.0), kStart + milliseconds(50)).event, GenerationEvent::kNotDue);
  const auto result = step(car(5.0), kStart + milliseconds(150));
  ASSERT_EQ(result.event, GenerationEvent::kGenerated);
  EXPECT_EQ(result.trigger, GenerationTrigger::kDynamics);
  // T_GenCam is the interval of the dynamics triggered CAM.
  EXPECT_EQ(generator_.status().generation_interval, milliseconds(150));

  // Heading change of 5 deg and speed change of 1 m/s trigger as well.
  constexpr double kDegree = std::numbers::pi / 180.0;
  EXPECT_EQ(
    step(car(5.0, 5.0 * kDegree), kStart + milliseconds(300)).trigger,
    GenerationTrigger::kDynamics);
  EXPECT_EQ(
    step(car(5.0, 5.0 * kDegree, 1.0), kStart + milliseconds(450)).trigger,
    GenerationTrigger::kDynamics);
}

TEST_F(VehicleCamGeneratorTest, HeadingChangeAcrossZeroIsSmall)
{
  constexpr double kDegree = std::numbers::pi / 180.0;
  step(car(0.0, 359.0 * kDegree), kStart);
  EXPECT_EQ(step(car(0.0, 1.0 * kDegree), kStart + milliseconds(200)).event,
    GenerationEvent::kNotDue);
}

TEST_F(VehicleCamGeneratorTest, GenerationIntervalIsResetAfterNGenCamTimeouts)
{
  step(car(0.0), kStart);
  step(car(5.0), kStart + milliseconds(200));
  ASSERT_EQ(generator_.status().generation_interval, milliseconds(200));

  // The vehicle stops changing: CAMs every 200 ms until N_GenCam = 3 timeouts.
  Time now = kStart + milliseconds(200);
  for (int i = 0; i < 3; ++i) {
    now += milliseconds(200);
    const auto result = step(car(5.0), now);
    EXPECT_EQ(result.trigger, GenerationTrigger::kTimeout);
  }
  EXPECT_EQ(generator_.status().generation_interval, VehicleCamGenerator::kGenCamMax);
}

TEST_F(VehicleCamGeneratorTest, PathHistoryWithLowFrequencyContainer)
{
  // Driving east with 10 m/s: a dynamics triggered CAM every 500 ms.
  Time now = kStart;
  std::optional<ufil_etsi::Cam> last;
  for (int i = 0; i <= 4; ++i) {
    const auto result = step(car(5.0 * i), now);
    ASSERT_EQ(result.event, GenerationEvent::kGenerated);
    EXPECT_TRUE(result.cam->vehicle_low_frequency.has_value());
    last = result.cam;
    now += milliseconds(500);
  }
  const auto & history = last->vehicle_low_frequency->path_history;
  ASSERT_EQ(history.size(), 4u);
  for (const auto & point : history) {
    // Each point is 5 m west of the preceding (younger) one and 500 ms older.
    EXPECT_LT(point.delta_longitude, 0);
    EXPECT_NEAR(point.delta_latitude, 0, 50);
    EXPECT_EQ(point.delta_time, 50);
    EXPECT_EQ(point.delta_altitude, ufil_etsi::etsi::kDeltaAltitudeUnavailable);
  }
  EXPECT_NEAR(history[0].delta_longitude, history[3].delta_longitude, 5);
  EXPECT_EQ(generator_.status().path_history_size, 5u);
}

TEST_F(VehicleCamGeneratorTest, LowFrequencyContainerAtMostEvery500Ms)
{
  EXPECT_TRUE(step(car(0.0), kStart).cam->vehicle_low_frequency.has_value());
  EXPECT_FALSE(step(car(5.0), kStart + milliseconds(200)).cam->vehicle_low_frequency.has_value());
  EXPECT_TRUE(step(car(10.0), kStart + milliseconds(500)).cam->vehicle_low_frequency.has_value());
}

TEST_F(VehicleCamGeneratorTest, TimeBefore2004IsRejected)
{
  const auto result = step(car(0.0), Time(seconds(100)));
  EXPECT_EQ(result.event, GenerationEvent::kInvalidTime);
  EXPECT_FALSE(result.cam.has_value());
  EXPECT_EQ(generator_.status().invalid_time, 1u);
  EXPECT_EQ(generator_.check(Time(seconds(101))).event, GenerationEvent::kNoObject);
}

TEST(VehicleCamGeneratorTimeShift, ShiftMovesTheGenerationDeltaTime)
{
  VehicleCamGeneratorConfig config;
  config.time_shift = seconds(1704067200);
  VehicleCamGenerator generator(config);
  generator.setObject(car(0.0), Time(seconds(100)), Time(seconds(100)));
  const auto result = generator.check(Time(seconds(100)));
  ASSERT_EQ(result.event, GenerationEvent::kGenerated);
}

TEST_F(VehicleCamGeneratorTest, InvalidPositionIsRejected)
{
  auto object = car(0.0);
  object.x = 1e9;
  EXPECT_EQ(step(object, kStart).event, GenerationEvent::kInvalidPosition);
  EXPECT_EQ(generator_.status().invalid_position, 1u);
}

TEST_F(VehicleCamGeneratorTest, ClockJumpBackRestarts)
{
  step(car(0.0), kStart);
  const auto result = step(car(0.0), kStart - seconds(10));
  EXPECT_EQ(result.event, GenerationEvent::kGenerated);
  EXPECT_EQ(result.trigger, GenerationTrigger::kFirst);
}
