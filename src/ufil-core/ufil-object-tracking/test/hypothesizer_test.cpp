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

#include <cmath>

#include <ufil_object_tracking/association/hypothesizer.hpp>
#include <ufil_object_tracking/association/association_functions.hpp>
#include <ufil_object_tracking/types/classification.hpp>
#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/existence_probability.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_object_tracking/types/track.hpp>

// Type definitions for better readability
using State = ufil::type::state::Pose2D;
using Dimension = ufil::type::dimension::Dimension2D;
using Classification = ufil::type::classification::ObjectClassification;
using ExistenceProbability = ufil::type::ExistenceProbability;
using Control = ufil::type::control::None;
using Measurement = ufil::type::measurement::Pose2D;
using Track = ufil::type::Track<State, Dimension, Classification, ExistenceProbability, Control,
    Measurement>;

TEST(Hypothesizer, euclideanDistanceZero)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  state.covariance() = State::CovarianceMatrixType::Identity();

  Track track;
  track.insert({state, {}});

  Measurement measurement;
  measurement.x() = 0.0f;
  measurement.y() = 0.0f;  // Euclidean distance should be 5
  measurement.covariance() = Measurement::CovarianceMatrixType::Identity();

  std::map<ufil::type::Id, Track> tracks{{track.uuid(), track}};
  std::map<ufil::type::Id, Measurement> measurements{{measurement.uuid(), measurement}};

  ufil::association::FunctionHypothesizer<Track> hypothesizer(&ufil::association::euclidean<Track>);
  ufil::type::CostMatrix cost_matrix = ufil::type::CostMatrix::Zero(1, 1);
  hypothesizer.hypothesize(tracks, measurements, cost_matrix);

  ASSERT_NEAR(cost_matrix(0, 0), 0.0, 1e-6);
}

TEST(Hypothesizer, euclideanDistanceNonZero)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  state.covariance() = State::CovarianceMatrixType::Identity();

  Track track;
  track.insert({state, {}});

  Measurement measurement;
  measurement.x() = 3.0f;
  measurement.y() = 4.0f;  // Euclidean distance should be 5
  measurement.covariance() = Measurement::CovarianceMatrixType::Identity();

  std::map<ufil::type::Id, Track> tracks{{track.uuid(), track}};
  std::map<ufil::type::Id, Measurement> measurements{{measurement.uuid(), measurement}};

  ufil::association::FunctionHypothesizer<Track> hypothesizer(&ufil::association::euclidean<Track>);
  ufil::type::CostMatrix cost_matrix = ufil::type::CostMatrix::Zero(1, 1);
  hypothesizer.hypothesize(tracks, measurements, cost_matrix);

  ASSERT_NEAR(cost_matrix(0, 0), 5.0, 1e-6);
}

TEST(Hypothesizer, moreTracksThanMeasurements)
{
  State state1(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state1.x() = 0.0f;
  state1.y() = 0.0f;
  state1.covariance() = State::CovarianceMatrixType::Identity();

  State state2(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state2.x() = 1.0f;
  state2.y() = 1.0f;
  state2.covariance() = State::CovarianceMatrixType::Identity();

  Track track1;
  track1.insert({state1, {}});
  Track track2;
  track2.insert({state2, {}});

  Measurement measurement;
  measurement.x() = 0.5f;
  measurement.y() = 0.5f;
  measurement.covariance() = Measurement::CovarianceMatrixType::Identity();

  std::map<ufil::type::Id, Track> tracks{{track1.uuid(), track1}, {track2.uuid(), track2}};
  std::map<ufil::type::Id, Measurement> measurements{{measurement.uuid(), measurement}};

  ufil::association::FunctionHypothesizer<Track> hypothesizer(&ufil::association::euclidean<Track>);
  ufil::type::CostMatrix cost_matrix = ufil::type::CostMatrix::Zero(tracks.size(),
    measurements.size());
  hypothesizer.hypothesize(tracks, measurements, cost_matrix);

  ASSERT_EQ(cost_matrix.rows(), 2);
  ASSERT_EQ(cost_matrix.cols(), 1);
}

TEST(Hypothesizer, emptyInput)
{
  std::map<ufil::type::Id, Track> tracks;
  std::map<ufil::type::Id, Measurement> measurements;

  ufil::association::FunctionHypothesizer<Track> hypothesizer(&ufil::association::euclidean<Track>);
  ufil::type::CostMatrix cost_matrix;
  hypothesizer.hypothesize(tracks, measurements, cost_matrix);

  ASSERT_EQ(cost_matrix.rows(), 0);
  ASSERT_EQ(cost_matrix.cols(), 0);
}

TEST(Hypothesizer, symmetricCost)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 2.0f;
  state.y() = 3.0f;
  state.covariance() = State::CovarianceMatrixType::Identity();

  Track track;
  track.insert({state, {}});

  Measurement measurement;
  measurement.x() = 5.0f;
  measurement.y() = 7.0f;
  measurement.covariance() = Measurement::CovarianceMatrixType::Identity();

  std::map<ufil::type::Id, Track> tracks{{track.uuid(), track}};
  std::map<ufil::type::Id, Measurement> measurements{{measurement.uuid(), measurement}};

  ufil::association::FunctionHypothesizer<Track> hypothesizer(&ufil::association::euclidean<Track>);
  ufil::type::CostMatrix cost_matrix = ufil::type::CostMatrix::Zero(1, 1);
  hypothesizer.hypothesize(tracks, measurements, cost_matrix);

  double expected = std::hypot(5.0 - 2.0, 7.0 - 3.0);  // should be 5.0
  ASSERT_NEAR(cost_matrix(0, 0), expected, 1e-6);
}

TEST(Hypothesizer, multipleTracksAndMeasurements)
{
  State s1(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  s1.x() = 0.0f;
  s1.y() = 0.0f;

  State s2(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  s2.x() = 1.0f;
  s2.y() = 1.0f;

  Track t1;
  t1.insert({s1, {}});
  Track t2;
  t2.insert({s2, {}});

  Measurement m1, m2;
  m1.x() = 1.0f;
  m1.y() = 0.0f;
  m2.x() = 0.0f;
  m2.y() = 1.0f;

  std::map<ufil::type::Id, Track> tracks{{t1.uuid(), t1}, {t2.uuid(), t2}};
  std::map<ufil::type::Id, Measurement> measurements{{m1.uuid(), m1}, {m2.uuid(), m2}};

  ufil::association::FunctionHypothesizer<Track> hypothesizer(&ufil::association::euclidean<Track>);
  ufil::type::CostMatrix cost_matrix = ufil::type::CostMatrix::Zero(2, 2);
  hypothesizer.hypothesize(tracks, measurements, cost_matrix);

  ASSERT_EQ(cost_matrix.rows(), 2);
  ASSERT_EQ(cost_matrix.cols(), 2);
  ASSERT_NEAR(cost_matrix(0, 0), 1.0, 1e-6);
  ASSERT_NEAR(cost_matrix(0, 1), 1.0, 1e-6);
  ASSERT_NEAR(cost_matrix(1, 0), std::sqrt(0.0 + 1.0), 1e-6);
  ASSERT_NEAR(cost_matrix(1, 1), std::sqrt(1.0 + 0.0), 1e-6);
}

TEST(Hypothesizer, costMatrixReuse)
{
  State s(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  s.x() = 0.0f;
  s.y() = 0.0f;
  Track t;
  t.insert({s, {}});

  Measurement m;
  m.x() = 3.0f;
  m.y() = 4.0f;

  std::map<ufil::type::Id, Track> tracks{{t.uuid(), t}};
  std::map<ufil::type::Id, Measurement> measurements{{m.uuid(), m}};

  ufil::association::FunctionHypothesizer<Track> hypothesizer(&ufil::association::euclidean<Track>);
  ufil::type::CostMatrix cost_matrix = ufil::type::CostMatrix::Ones(1, 1) * 42;  // garbage data
  hypothesizer.hypothesize(tracks, measurements, cost_matrix);

  ASSERT_NEAR(cost_matrix(0, 0), 5.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
