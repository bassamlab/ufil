// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#include <optional>
#include <unordered_set>

#include <ufil_object_tracking/models/transition/cv_transition_model.hpp>
#include <ufil_object_tracking/types/classification.hpp>
#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/track.hpp>
#include <ufil_object_tracking/types/state.hpp>

TEST(ConstantVelocityTransitionModel, statePrediction)
{
  using State = ufil::type::state::PositionVelocity2D;
  using Control = ufil::type::control::None;
  using TransitionModel = ufil::model::transition::ConstantVelocityTransitionModel<State, Control>;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);
  auto duration = ufil::from_seconds<ufil::type::Duration>(1.0);

  State state;
  state.x() = 1.0f;
  state.y() = 2.0f;
  state.vx() = 0.5f;
  state.vy() = -0.5f;

  State predicted_state;
  predicted_state.x() = 0.0f;
  predicted_state.y() = 0.0f;
  predicted_state.vx() = 0.0f;
  predicted_state.vy() = 0.0f;

  TransitionModel model;
  model.onModelInitialization();
  model.onEveryTimestep(state, timestamp, duration);
  model.step(state, std::nullopt, timestamp, predicted_state);

  // In a Constant Velocity trasition model, the position should change by velocity * delta_t
  EXPECT_FLOAT_EQ(predicted_state.x(), state.x() + state.vx() * ufil::to_seconds(duration));
  EXPECT_FLOAT_EQ(predicted_state.y(), state.y() + state.vy() * ufil::to_seconds(duration));
  // The velocity should be the same as the input state as the values are copied directly
  EXPECT_FLOAT_EQ(predicted_state.vx(), state.vx());
  EXPECT_FLOAT_EQ(predicted_state.vy(), state.vy());

  // The covariance should have grown due to process noise check by comparing the sum of the
  // variances
  auto state_cov_sum = state.covariance().trace();
  auto predicted_state_cov_sum = predicted_state.covariance().trace();
  EXPECT_GT(predicted_state_cov_sum, state_cov_sum);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
