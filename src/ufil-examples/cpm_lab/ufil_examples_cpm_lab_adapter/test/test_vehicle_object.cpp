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

#include <numbers>  // listed with the C headers, cpplint does not know this C++20 header

#include <cmath>

#include "ufil_examples_cpm_lab_adapter/vehicle_object.hpp"

using ufil_examples_cpm_lab_adapter::kLabScale;
using ufil_examples_cpm_lab_adapter::toVehicleObject;
using ufil_examples_cpm_lab_adapter::VehicleState;

TEST(VehicleObjectTest, ScalesPositionAndDimensions)
{
  VehicleState state;
  state.position = {1.0, 2.0, 0.5};
  state.yaw = 0.3;
  const auto object = toVehicleObject(state, 7);
  EXPECT_EQ(object.id, 7u);
  EXPECT_DOUBLE_EQ(object.existence_probability, 0.99);
  EXPECT_DOUBLE_EQ(object.x, 1.0 * kLabScale);
  EXPECT_DOUBLE_EQ(object.y, 2.0 * kLabScale);
  EXPECT_DOUBLE_EQ(object.yaw, 0.3);
  EXPECT_DOUBLE_EQ(object.length, 0.22 * kLabScale);
  EXPECT_DOUBLE_EQ(object.width, 0.105 * kLabScale);
  EXPECT_DOUBLE_EQ(object.height, 0.06 * kLabScale);
  EXPECT_LT(object.state_variances[5], 0.0) << "the yaw rate is not estimated";
  for (int i = 0; i < 5; ++i) {
    EXPECT_GT(object.state_variances[i], 0.0);
  }
}

TEST(VehicleObjectTest, RotatesVelocityIntoTheStateFrame)
{
  VehicleState state;
  state.yaw = std::numbers::pi / 2.0;
  state.linear_velocity = {1.0, 0.0, 0.0};
  const auto object = toVehicleObject(state, 1);
  EXPECT_NEAR(object.vx, 0.0, 1e-12);
  EXPECT_NEAR(object.vy, kLabScale, 1e-12);
}
