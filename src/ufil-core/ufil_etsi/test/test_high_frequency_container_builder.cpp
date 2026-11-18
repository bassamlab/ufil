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

#include <numbers>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/cam_encoding.hpp"
#include "ufil_etsi/types.hpp"

// The high-frequency container is computed by ufil_etsi::encodeVehicleHighFrequency (formerly
// HighFrequencyContainerBuilder).

using ufil_etsi::encodeVehicleHighFrequency;
using ufil_etsi::Object;
namespace etsi = ufil_etsi::etsi;

namespace
{

/// Object heading north (yaw = pi/2) with a global velocity and acceleration.
Object objectHeadingNorth()
{
  Object object;
  object.yaw = std::numbers::pi / 2.0;
  // Driving north at 10 m/s.
  object.v_x = 0.0;
  object.v_y = 10.0;
  // Braking (acceleration south) and accelerating to the left (west).
  object.a_x = -1.5;
  object.a_y = -2.0;
  // Velocity: precise east-west, uncertain north-south (= along the heading).
  object.covariance[18] = 1e-6;
  object.covariance[27] = 0.04;
  // Acceleration: Var(a_x) = 0.25 (lateral), Var(a_y) = 1e-4 (longitudinal).
  object.covariance[36] = 0.25;
  object.covariance[45] = 1e-4;
  return object;
}

}  // namespace

TEST(HighFrequencyContainerBuilder, AccelerationIsProjectedIntoVehicleFrame)
{
  const auto hf = encodeVehicleHighFrequency(objectHeadingNorth());

  // Acceleration south while heading north is braking: -2.0 m/s^2 longitudinal.
  EXPECT_EQ(hf.longitudinal_acceleration, -20);
  // Acceleration west while heading north is to the left: +1.5 m/s^2 lateral.
  ASSERT_TRUE(hf.lateral_acceleration.has_value());
  EXPECT_EQ(hf.lateral_acceleration->value, 15);

  // Longitudinal sigma 0.01 m/s^2 -> 95 % within 0.1 m/s^2; lateral sigma 0.5 -> 9.8 dm/s^2.
  EXPECT_EQ(hf.longitudinal_acceleration_confidence, 1);  // pointOneMeterPerSecSquared
  EXPECT_EQ(hf.lateral_acceleration->confidence, 9);
}

TEST(HighFrequencyContainerBuilder, SpeedConfidenceIsAlongHeading)
{
  const auto hf = encodeVehicleHighFrequency(objectHeadingNorth());

  EXPECT_EQ(hf.speed, 1000);
  // Along the heading (north) sigma is 0.2 m/s -> 1.96 * 20 cm/s = 39 cm/s.
  EXPECT_EQ(hf.speed_confidence, 39);
  EXPECT_EQ(hf.drive_direction, etsi::kDriveDirectionForward);
}
