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

#include <sstream>

#include <ufil_object_tracking/management/deleter/existence_deleter.hpp>
#include <ufil_object_tracking/management/deleter/json_history_deleter.hpp>
#include <ufil_object_tracking/management/deleter/timed_deleter.hpp>
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

TEST(JsonHistoryDeleter, dumpsHistoryBeforeDeletion)
{
  Track track;

  State state1(ufil::from_nanoseconds<ufil::type::Timestamp>(1000));
  state1.x() = 1.0;
  state1.y() = 2.0;
  track.insert({state1, {}});

  State state2(ufil::from_nanoseconds<ufil::type::Timestamp>(2000));
  state2.x() = 3.0;
  state2.y() = 4.0;
  track.insert({state2, {}});
  track.currentExistenceProbability().existence() = 0.0;

  std::map<ufil::type::Id, Track> tracks;
  tracks.emplace(track.uuid(), track);

  std::ostringstream json_output;
  ufil::management::JsonHistoryExistenceDeleter<Track> deleter(json_output,
    ufil::management::ExistenceDeleter<Track>(0.5));

  deleter.deleteTracks(ufil::from_nanoseconds<ufil::type::Timestamp>(3000), tracks);

  EXPECT_TRUE(tracks.empty());
  const std::string json = json_output.str();
  EXPECT_NE(json.find("\"deleted_tracks\""), std::string::npos);
  EXPECT_NE(json.find("\"track_id\""), std::string::npos);
  EXPECT_NE(json.find("\"history\""), std::string::npos);
  EXPECT_NE(json.find("\"state_vector\""), std::string::npos);
  EXPECT_NE(json.find("\"creation_time_ns\""), std::string::npos);
}

TEST(JsonHistoryDeleter, dumpsHistoryBeforeTimedDeletion)
{
  Track track;

  State state1(ufil::from_nanoseconds<ufil::type::Timestamp>(1000));
  state1.x() = 1.0;
  state1.y() = 2.0;
  track.insert({state1, {}});

  State state2(ufil::from_nanoseconds<ufil::type::Timestamp>(2000));
  state2.x() = 3.0;
  state2.y() = 4.0;
  track.insert({state2, {}});

  std::map<ufil::type::Id, Track> tracks;
  tracks.emplace(track.uuid(), track);

  std::ostringstream json_output;
  ufil::management::JsonHistoryTimedDeleter<Track> deleter(json_output,
    ufil::management::TimedDeleter<Track>(ufil::from_nanoseconds<ufil::type::Duration>(500)));

  deleter.deleteTracks(ufil::from_nanoseconds<ufil::type::Timestamp>(3000), tracks);

  EXPECT_TRUE(tracks.empty());
  const std::string json = json_output.str();
  EXPECT_NE(json.find("\"deleted_tracks\""), std::string::npos);
  EXPECT_NE(json.find("\"track_id\""), std::string::npos);
  EXPECT_NE(json.find("\"history\""), std::string::npos);
  EXPECT_NE(json.find("\"state_vector\""), std::string::npos);
  EXPECT_NE(json.find("\"creation_time_ns\""), std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
TEST(JsonHistoryDeleter, canOmitHistoryFromSerializedTrack)
{
  Track track;

  State state1(ufil::from_nanoseconds<ufil::type::Timestamp>(1000));
  state1.x() = 1.0;
  state1.y() = 2.0;
  track.insert({state1, {}});

  const std::string json = ufil::serialization::to_json(track, false);
  EXPECT_NE(json.find("\"track_id\""), std::string::npos);
  EXPECT_NE(json.find("\"creation_time_ns\""), std::string::npos);
  EXPECT_EQ(json.find("\"history\""), std::string::npos);
}
