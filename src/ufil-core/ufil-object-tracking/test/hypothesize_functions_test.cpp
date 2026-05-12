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

#include <ufil_object_tracking/association/hypothesize_functions.hpp>
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
using IoU_Measurement = ufil::type::measurement::Pose2DWithDimension2D;
using Track = ufil::type::Track<State, Dimension, Classification, ExistenceProbability, Control,
    Measurement>;
using IoU_Track = ufil::type::Track<State, Dimension, Classification, ExistenceProbability, Control,
    IoU_Measurement>;

TEST(hypothesize_functions, intersection_over_union_distance_default)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  Dimension dimension;
  dimension.length() = 1.0f;
  dimension.width() = 1.0f;
  // Create track from state
  IoU_Track track;
  track.insert({state, dimension});

  // Create a measurement
  IoU_Measurement measurement;
  measurement.x() = 0.0f;
  measurement.y() = 0.0f;
  measurement.dimension().length() = 1.0f;
  measurement.dimension().width() = 1.0f;

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::intersection_over_union_2D(track, measurement, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 0.0f);
}

TEST(hypothesize_functions, intersection_over_union_distance_100)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  Dimension dimension;
  dimension.length() = 1.0f;
  dimension.width() = 1.0f;
  // Create track from state
  IoU_Track track;
  track.insert({state, dimension});

  // Create a measurement
  IoU_Measurement measurement;
  measurement.x() = 20.0f;
  measurement.y() = 20.0f;
  measurement.dimension().length() = 1.0f;
  measurement.dimension().width() = 1.0f;

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::intersection_over_union_2D(track, measurement, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 100.0f);
}

TEST(hypothesize_functions, intersection_over_union_distance_without_angle)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.5f;
  state.y() = 0.0f;
  Dimension dimension;
  dimension.length() = 1.0f;
  dimension.width() = 1.0f;
  // Create track from state
  IoU_Track track;
  track.insert({state, dimension});

  // Create a measurement
  IoU_Measurement measurement;
  measurement.x() = 0.0f;
  measurement.y() = 0.0f;
  measurement.dimension().length() = 1.0f;
  measurement.dimension().width() = 1.0f;

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::intersection_over_union_2D(track, measurement, distance);

  // Check if the distance is correct
  ASSERT_GT(distance, 0.0f);
  ASSERT_LT(distance, 100.0f);
}

TEST(hypothesize_functions, intersection_over_union_with_angle)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  state.yaw() = 1.57f;  // ca 90 degree
  Dimension dimension;
  dimension.length() = 1.0f;
  dimension.width() = 1.0f;
  // Create track from state
  IoU_Track track;
  track.insert({state, dimension});

  // Create a measurement
  IoU_Measurement measurement;
  measurement.x() = 0.0f;
  measurement.y() = 0.0f;
  measurement.dimension().length() = 1.0f;
  measurement.dimension().width() = 1.0f;

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::intersection_over_union_2D(track, measurement, distance);

  // Check if the distance is correct
  ASSERT_GT(distance, 0.0f);
  ASSERT_LT(distance, 100.0f);
}

TEST(hypothesize_functions, intersection_over_union_negativ)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  state.yaw() = 0.0f;  // ca 90 degree
  Dimension dimension;
  dimension.length() = -1.0f;
  dimension.width() = -1.0f;
  // Create track from state
  IoU_Track track;
  track.insert({state, dimension});

  // Create a measurement
  IoU_Measurement measurement;
  measurement.x() = 0.0f;
  measurement.y() = 0.0f;
  measurement.dimension().length() = -1.0f;
  measurement.dimension().width() = -1.0f;

  // Calculate the distance
  ufil::type::Scalar distance;

  // Check if error is thrown
  EXPECT_THROW({
    ufil::association::intersection_over_union_2D(track, measurement, distance);
  }, std::runtime_error);
}

TEST(hypothesize_functions, intersection_over_union_all_dimension_zero)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  state.yaw() = 1.57f;  // ca 90 degree
  Dimension dimension;
  dimension.length() = 0.0f;
  dimension.width() = 0.0f;
  // Create track from state
  IoU_Track track;
  track.insert({state, dimension});

  // Create a measurement
  IoU_Measurement measurement;
  measurement.x() = 0.0f;
  measurement.y() = 0.0f;
  measurement.dimension().length() = 0.0f;
  measurement.dimension().width() = 0.0f;

  // Calculate the distance
  ufil::type::Scalar distance;

  // Check if error is thrown
  EXPECT_THROW({
    ufil::association::intersection_over_union_2D(track, measurement, distance);
  }, std::runtime_error);
}

TEST(hypothesize_functions, intersection_over_union_one_dimension_zero)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  state.yaw() = 1.57f;  // ca 90 degree
  Dimension dimension;
  dimension.length() = 0.0f;
  dimension.width() = 0.0f;
  // Create track from state
  IoU_Track track;
  track.insert({state, dimension});

  // Create a measurement
  IoU_Measurement measurement;
  measurement.x() = 0.0f;
  measurement.y() = 0.0f;
  measurement.dimension().length() = 1.0f;
  measurement.dimension().width() = 1.0f;

  // Calculate the distance
  ufil::type::Scalar distance;

  // Check if error is thrown
  EXPECT_THROW({
    ufil::association::intersection_over_union_2D(track, measurement, distance);
  }, std::runtime_error);
}

TEST(HypothesizeFunctions, euclideanDistanceDefault)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::euclidean(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 0.0);
}

TEST(HypothesizeFunctions, euclideanDistanceSingleDimension)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 10.0f;
  state.y() = 0.0f;

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::euclidean(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 10.0);
}

TEST(HypothesizeFunctions, euclideanDistanceDiagonal)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 10.0f;
  state.y() = 0.0f;

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 10.0f;

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::euclidean(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 14.1421356f);
}

TEST(HypothesizeFunctions, euclideanDistanceNegativeDiagonal)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = -10.0f;
  state.y() = -10.0f;

  Track track;
  track.insert({state, {}});

  Measurement measurment;
  measurment.x() = -5.0f;
  measurment.y() = -5.0f;

  ufil::type::Scalar distance;
  ufil::association::euclidean(track, measurment, distance);

  ASSERT_FLOAT_EQ(distance, 7.071068f);
}

TEST(HypothesizeFunctions, mahalanobisDistanceDefault)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  state.covariance() = State::CovarianceMatrixType::Identity();

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::mahalanobis(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 0.0);
}

TEST(HypothesizeFunctions, mahalanobisDistanceZeroCovariance)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  state.covariance() = State::CovarianceMatrixType::Zero();

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Zero();

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::mahalanobis(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, ufil::scalar::ScalarMax);
}

TEST(HypothesizeFunctions, mahalanobisDistanceSingleDimension)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 10.0f;
  state.y() = 0.0f;
  state.covariance() = State::CovarianceMatrixType::Identity();

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::mahalanobis(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 10.0);
}

TEST(HypothesizeFunctions, mahalanobisDistanceUnequalCovariance)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 10.0f;
  state.y() = 0.0f;
  state.covariance() = 2.0f * State::CovarianceMatrixType::Identity();

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::mahalanobis(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 8.16496658f);
}

TEST(HypothesizeFunctions, mahalanobisSmallCovariance)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 10.0f;
  state.y() = 0.0f;
  state.covariance() = 1e-6 * State::CovarianceMatrixType::Identity();  // Tiny covariance

  Track track;
  track.insert({state, {}});

  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  ufil::type::Scalar distance;
  ufil::association::mahalanobis(track, measurment, distance);

  ASSERT_GT(distance, 0.0f);  // Should still return a positive distance
}

TEST(HypothesizeFunctions, mahalanobisLargeCovariance)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 10.0f;
  state.y() = 0.0f;
  state.covariance() = 1e+6 * State::CovarianceMatrixType::Identity();  // Large covariance

  Track track;
  track.insert({state, {}});

  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  ufil::type::Scalar distance;
  ufil::association::mahalanobis(track, measurment, distance);

  ASSERT_GT(distance, 0.0f);  // Should still return a valid distance
}

TEST(HypothesizeFunctions, mahalanobisOffDiagonalCovariance)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 10.0f;
  state.y() = 0.0f;
  state.covariance() << 2.0f, 1.0f, 1.0f,  // Off-diagonal covariance
      1.0f, 2.0f, 1.0f,
      1.0f, 1.0f, 2.0f;

  Track track;
  track.insert({state, {}});

  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  ufil::type::Scalar distance;
  ufil::association::mahalanobis(track, measurment, distance);

  ASSERT_GT(distance, 0.0f);  // Should return a valid distance
}

TEST(HypothesizeFunctions, mahalanobisMaxFloat)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = ufil::scalar::ScalarMax;
  state.y() = ufil::scalar::ScalarMax;
  state.covariance() = ufil::scalar::ScalarMax * Measurement::CovarianceMatrixType::Identity();

  Track track;
  track.insert({state, {}});

  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  ufil::type::Scalar distance;
  ufil::association::mahalanobis(track, measurment, distance);

  ASSERT_GT(distance, 0.0f);  // Valid non-zero distance should be computed
}

TEST(HypothesizeFunctions, wassersteinDistanceDefault)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  state.covariance() = State::CovarianceMatrixType::Identity();

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::wasserstein(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 0.0);
}

TEST(HypothesizeFunctions, wassersteinDistanceZeroCovariance)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0f;
  state.y() = 0.0f;
  state.covariance() = State::CovarianceMatrixType::Zero();

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Zero();

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::wasserstein(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 0.0);
}

TEST(HypothesizeFunctions, wassersteinDistanceSingleDimension)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 10.0f;
  state.y() = 0.0f;
  state.covariance() = State::CovarianceMatrixType::Identity();

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;

  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::wasserstein(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 10.0);
}

TEST(HypothesizeFunctions, wassersteinDistanceUnequalCovariance)
{
  // Create state
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 10.0f;
  state.y() = 0.0f;
  state.covariance() = 2.0 * State::CovarianceMatrixType::Identity();

  // Create track from state
  Track track;
  track.insert({state, {}});

  // Create a measurement
  Measurement measurment;
  measurment.x() = 0.0f;
  measurment.y() = 0.0f;

  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  // Calculate the distance
  ufil::type::Scalar distance;
  ufil::association::wasserstein(track, measurment, distance);

  // Check if the distance is correct
  ASSERT_EQ(distance, 10.0257025f);
}

TEST(HypothesizeFunctions, wassersteinIdentityCovariance)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 5.0f;
  state.y() = 5.0f;
  state.covariance() = State::CovarianceMatrixType::Identity();

  Track track;
  track.insert({state, {}});

  Measurement measurment;
  measurment.x() = 5.0f;
  measurment.y() = 5.0f;
  measurment.covariance() = Measurement::CovarianceMatrixType::Identity();

  ufil::type::Scalar distance;
  ufil::association::wasserstein(track, measurment, distance);

  ASSERT_EQ(distance, 0.0);  // Should be zero as the positions and covariances are identical
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
