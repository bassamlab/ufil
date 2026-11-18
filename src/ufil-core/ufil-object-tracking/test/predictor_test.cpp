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
#include <type_traits>

#include <ufil_object_tracking/predict/probability/existence_predictor.hpp>
#include <ufil_object_tracking/predict/state/model_state_predictor.hpp>
#include <ufil_object_tracking/predict/selective_predictor.hpp>
#include <ufil_object_tracking/predict/state/state_predictor.hpp>
#include <ufil_object_tracking/types/classification.hpp>
#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/existence_probability.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_object_tracking/types/track.hpp>

using State = ufil::type::state::Pose2D;
using Dimension = ufil::type::dimension::Dimension2D;
using Classification = ufil::type::classification::ObjectClassification;
using ExistenceProbability = ufil::type::ExistenceProbability;
using Control = ufil::type::control::None;
using Measurement = ufil::type::measurement::Pose2D;
using Track = ufil::type::Track<State, Dimension, Classification, ExistenceProbability, Control,
    Measurement>;

class TestStatePredictor : public ufil::predict::StatePredictor<Track>
{
public:
  using Base = ufil::predict::StatePredictor<Track>;
  using Base::predict;

  void predict(
    const TrackType & track, const std::optional<ControlType> & /*control*/,
    const ufil::type::Timestamp & timestamp, StateType & prediction) override
  {
    prediction = track.previousState();
    prediction.timestamp() = timestamp;
    prediction.x() = 11.0f;
    prediction.y() = 22.0f;
  }
};

class TestExistencePredictor : public ufil::predict::ExistencePredictor<Track>
{
public:
  using Base = ufil::predict::ExistencePredictor<Track>;
  using Base::predict;

  void predict(
    const TrackType & /*track*/, const std::optional<ControlType> & /*control*/,
    const ufil::type::Timestamp & /*timestamp*/,
    ExistenceProbabilityType & prediction) override
  {
    prediction.existence() = 0.6f;
  }
};

static_assert(std::is_base_of_v<ufil::predict::StatePredictor<Track>,
  ufil::predict::ModelStatePredictor<Track>>);
static_assert(std::is_base_of_v<ufil::predict::StatePredictor<Track>,
  ufil::predict::SelectivePredictor<Track>>);

namespace
{

Track makeTrack()
{
  Track track;

  State previous_state(ufil::from_nanoseconds<ufil::type::Timestamp>(1000));
  previous_state.x() = 1.0f;
  previous_state.y() = 2.0f;
  track.insert({previous_state, {}});

  State current_state(ufil::from_nanoseconds<ufil::type::Timestamp>(2000));
  current_state.x() = 3.0f;
  current_state.y() = 4.0f;
  track.insert({current_state, {}});

  return track;
}

}  // namespace

TEST(Predictor, statePredictorWritesCurrentStateAndStoresControl)
{
  Track track = makeTrack();
  TestStatePredictor predictor;
  auto timestamp = ufil::from_nanoseconds<ufil::type::Timestamp>(3000);
  std::optional<Control> control = Control{};

  predictor.predict(track, control, timestamp);

  EXPECT_FLOAT_EQ(track.currentState().x(), 11.0f);
  EXPECT_FLOAT_EQ(track.currentState().y(), 22.0f);
  EXPECT_TRUE(track.currentHistoryEntry().hasControlInput());
  EXPECT_EQ(track.currentHistoryEntry().controlInput().has_value(), control.has_value());
}

TEST(Predictor, existencePredictorWritesCurrentExistenceAndStoresControl)
{
  Track track = makeTrack();
  TestExistencePredictor predictor;
  std::optional<Control> control = Control{};

  predictor.predict(track, control, track.currentState().timestamp());

  EXPECT_FLOAT_EQ(track.currentExistenceProbability().existence(), 0.6f);
  EXPECT_TRUE(track.currentHistoryEntry().hasControlInput());
  EXPECT_EQ(track.currentHistoryEntry().controlInput().has_value(), control.has_value());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
