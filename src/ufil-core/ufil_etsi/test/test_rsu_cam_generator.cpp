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
#include <stdexcept>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/its_time.hpp"
#include "ufil_etsi/rsu_cam_generator.hpp"

// Core library tests, no ROS 2 required.

using std::chrono::seconds;
using ufil_etsi::RsuCamGenerator;
using ufil_etsi::RsuCamGeneratorConfig;
using ufil_etsi::Time;
namespace etsi = ufil_etsi::etsi;

TEST(RsuCamGenerator, StaticRoadSideUnitCam)
{
  RsuCamGeneratorConfig config;
  config.station_id = 1234;
  config.position = {48.079772, 11.635735};
  config.altitude = 520.25;
  RsuCamGenerator generator(config);

  const Time now = seconds(1704067200);
  const auto cam = generator.generate(now);
  ASSERT_TRUE(cam.has_value());
  EXPECT_EQ(cam->station_id, 1234u);
  EXPECT_EQ(cam->station_type, etsi::station_type::kRoadSideUnit);
  EXPECT_EQ(cam->generation_delta_time, *ufil_etsi::generationDeltaTime(now));
  EXPECT_EQ(cam->reference_position.latitude, 480797720);
  EXPECT_EQ(cam->reference_position.longitude, 116357350);
  EXPECT_EQ(cam->reference_position.altitude, 52025);
  EXPECT_EQ(cam->reference_position.semi_major_confidence, etsi::kSemiAxisLengthMin);
  EXPECT_FALSE(cam->vehicle_high_frequency.has_value());
  EXPECT_FALSE(cam->vehicle_low_frequency.has_value());
  EXPECT_EQ(generator.generated(), 1u);

  EXPECT_FALSE(generator.generate(Time(0)).has_value());
}

TEST(RsuCamGenerator, Validation)
{
  RsuCamGeneratorConfig config;
  EXPECT_FALSE(ufil_etsi::validate(config).has_value());
  config.position.latitude = 95.0;
  EXPECT_THROW(RsuCamGenerator{config}, std::invalid_argument);
  config.position.latitude = 0.0;
  config.position.longitude = -181.0;
  EXPECT_TRUE(ufil_etsi::validate(config).has_value());
  config.position.longitude = 0.0;
  config.altitude = 9000.0;
  EXPECT_TRUE(ufil_etsi::validate(config).has_value());
}
