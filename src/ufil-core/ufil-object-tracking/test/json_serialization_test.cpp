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

#include <ufil_object_tracking/serialization/json.hpp>
#include <ufil_object_tracking/types/classification.hpp>
#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/existence_probability.hpp>
#include <ufil_object_tracking/types/history_entry.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_object_tracking/types/track.hpp>

namespace
{

using TestState = ufil::type::state::PoseVelocityAcceleration2D;
using TestDimension = ufil::type::dimension::Dimension3D;
using TestClassification = ufil::type::classification::ObjectClassification;
using TestExistenceProbability = ufil::type::ExistenceProbability;
using TestControl = ufil::type::control::None;
using TestMeasurement = ufil::type::measurement::Pose2DWithDimension3D;
using TestHistoryEntry =
  ufil::type::HistoryEntry<TestState, TestDimension, TestClassification, TestExistenceProbability,
    TestControl, TestMeasurement>;
using TestTrack = ufil::type::Track<TestState, TestDimension, TestClassification,
    TestExistenceProbability, TestControl, TestMeasurement>;

}  // namespace

TEST(JsonSerialization, serializesTrack)
{
  TestTrack track;

  TestState state;
  state.timestamp() = ufil::from_nanoseconds<ufil::type::Timestamp>(987654321);
  state.x() = 1.5;
  state.y() = -2.5;
  state.vx() = 0.25;
  state.vy() = -0.5;
  state.ax() = 0.0;
  state.ay() = 0.0;
  state.yaw() = 0.75;
  state.yawRate() = 0.01;

  TestDimension dimension;
  dimension.length() = 4.2;
  dimension.width() = 1.8;
  dimension.height() = 1.4;

  TestExistenceProbability existence_probability;
  existence_probability.existence() = 0.9;

  TestClassification classification;
  classification.car() = 1.0;

  TestHistoryEntry entry(state, dimension, existence_probability, classification);
  entry.occluded() = true;
  track.insert(std::move(entry));

  const std::string json = ufil::serialization::to_json(track);
  EXPECT_NE(json.find("\"track_id\""), std::string::npos);
  EXPECT_NE(json.find("\"creation_time_ns\""), std::string::npos);
  EXPECT_NE(json.find("\"history\""), std::string::npos);
  EXPECT_NE(json.find("\"track_width\""), std::string::npos);
  EXPECT_NE(json.find("\"classification_vector\""), std::string::npos);
}
