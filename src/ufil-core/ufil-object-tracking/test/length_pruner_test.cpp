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

#include <map>
#include <stdexcept>

#include <ufil_object_tracking/management/pruner/length_pruner.hpp>
#include <ufil_object_tracking/types/classification.hpp>
#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/existence_probability.hpp>
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

TEST(LengthPruner, keepsNewestEntriesOnly)
{
  Track track;
  State state1(ufil::from_nanoseconds<ufil::type::Timestamp>(1000));
  State state2(ufil::from_nanoseconds<ufil::type::Timestamp>(2000));
  State state3(ufil::from_nanoseconds<ufil::type::Timestamp>(3000));
  track.insert({state1, {}});
  track.insert({state2, {}});
  track.insert({state3, {}});

  std::map<ufil::type::Id, Track> tracks{};
  tracks.emplace(track.uuid(), track);

  ufil::management::LengthPruner<Track> pruner(2);
  pruner.pruneTrackHistory(ufil::from_nanoseconds<ufil::type::Timestamp>(4000), tracks);

  auto & pruned_track = tracks.begin()->second;
  EXPECT_EQ(pruned_track.history().size(), 2);
  EXPECT_FALSE(pruned_track.contains(state1.timestamp()));
  EXPECT_TRUE(pruned_track.contains(state2.timestamp()));
  EXPECT_TRUE(pruned_track.contains(state3.timestamp()));
}

TEST(LengthPruner, doesNotPruneWhenAtOrBelowLimit)
{
  Track track;
  State state1(ufil::from_nanoseconds<ufil::type::Timestamp>(1000));
  State state2(ufil::from_nanoseconds<ufil::type::Timestamp>(2000));
  track.insert({state1, {}});
  track.insert({state2, {}});

  std::map<ufil::type::Id, Track> tracks{};
  tracks.emplace(track.uuid(), track);

  ufil::management::LengthPruner<Track> pruner(2);
  pruner.pruneTrackHistory(ufil::from_nanoseconds<ufil::type::Timestamp>(3000), tracks);

  auto & pruned_track = tracks.begin()->second;
  EXPECT_EQ(pruned_track.history().size(), 2);
  EXPECT_TRUE(pruned_track.contains(state1.timestamp()));
  EXPECT_TRUE(pruned_track.contains(state2.timestamp()));
}

TEST(LengthPruner, constructorRejectsZeroLength)
{
  EXPECT_THROW(
    ufil::management::LengthPruner<Track> pruner(0),
    std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
