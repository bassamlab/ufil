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

#include <boost/uuid/uuid_io.hpp>

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

TEST(Track, defaultConstructor)
{
  Track track;
  EXPECT_TRUE(track.history().empty());
  EXPECT_EQ(track.creationTime(), ufil::type::Timestamp::min());
}

TEST(Track, insertAndRetrieveHistoryEntry)
{
  Track track;
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 1.0f;
  state.y() = 2.0f;

  track.insert({state, {}});

  EXPECT_FALSE(track.history().empty());
  EXPECT_EQ(track.currentState().x(), 1.0f);
  EXPECT_EQ(track.currentState().y(), 2.0f);
}

TEST(Track, creationTimeSetOnFirstInsert)
{
  Track track;
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(1234567890));
  track.insert({state, {}});

  EXPECT_EQ(track.creationTime(), state.timestamp());
}

TEST(Track, validReturnsCorrectValue)
{
  Track track;
  EXPECT_FALSE(track.valid());

  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  track.insert({state, {}});
  EXPECT_TRUE(track.valid());
}

TEST(Track, containsTimestamp)
{
  Track track;
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(1234567890));
  track.insert({state, {}});

  EXPECT_TRUE(track.contains(state.timestamp()));
  EXPECT_FALSE(track.contains(ufil::from_nanoseconds<ufil::type::Timestamp>(987654321)));
}

TEST(Track, pruneHistory)
{
  Track track;
  State state1(ufil::from_nanoseconds<ufil::type::Timestamp>(1000));
  State state2(ufil::from_nanoseconds<ufil::type::Timestamp>(2000));
  track.insert({state1, {}});
  track.insert({state2, {}});

  track.pruneBefore(ufil::from_nanoseconds<ufil::type::Timestamp>(1500));

  EXPECT_EQ(track.history().size(), 1);
  EXPECT_EQ(track.currentState().timestamp(), state2.timestamp());
}

TEST(Track, eraseHistoryEntry)
{
  Track track;
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(1234567890));
  track.insert({state, {}});

  EXPECT_TRUE(track.contains(state.timestamp()));
  track.erase(state.timestamp());
  EXPECT_FALSE(track.contains(state.timestamp()));
}

TEST(Track, lastUpdatedWithAssociatedMeasurement)
{
  Track track;
  State state1(ufil::from_nanoseconds<ufil::type::Timestamp>(1000));
  State state2(ufil::from_nanoseconds<ufil::type::Timestamp>(2000));
  track.insert({state1, {}});
  track.insert({state2, {}});

  EXPECT_EQ(track.lastUpdated(), ufil::type::Timestamp::min());

  auto & entry = track.at(state2.timestamp());
  entry.associatedMeasurement() = Measurement();  // Set an associated measurement

  EXPECT_EQ(track.lastUpdated(), state2.timestamp());
}

TEST(Track, equalityOperator)
{
  Track track1;
  Track track2;

  EXPECT_NE(track1, track2);

  Track track3 = track1;
  EXPECT_EQ(track1, track3);
}

TEST(Track, comparisonOperators)
{
  Track track1;
  Track track2;

  EXPECT_TRUE(track1<track2 || track1> track2 || track1 == track2);
  EXPECT_FALSE(track1 < track1);
  EXPECT_FALSE(track1 > track1);
  EXPECT_TRUE(track1 <= track1);
  EXPECT_TRUE(track1 >= track1);
}

TEST(Track, currentAndPreviousState)
{
  Track track;
  State state1(ufil::from_nanoseconds<ufil::type::Timestamp>(1000));
  State state2(ufil::from_nanoseconds<ufil::type::Timestamp>(2000));
  track.insert({state1, {}});
  track.insert({state2, {}});

  EXPECT_EQ(track.currentState().timestamp(), state2.timestamp());
  EXPECT_EQ(track.previousState().timestamp(), state1.timestamp());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
