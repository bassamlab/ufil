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
#include <random>

#include <ufil_object_tracking/types/classification.hpp>
#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/existence_probability.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_object_tracking/types/track.hpp>
#include <ufil_object_tracking/update/dimension/dimension_gridmap_updater.hpp>

// Type aliases (same style as your file)
using State = ufil::type::state::Pose2D;
using Dimension = ufil::type::dimension::Dimension2D;
using Classification = ufil::type::classification::ObjectClassification;
using ExistenceProbability = ufil::type::ExistenceProbability;
using Control = ufil::type::control::None;
using Measurement = ufil::type::measurement::Pose2DWithDimension2D;
using Track = ufil::type::Track<State, Dimension, Classification, ExistenceProbability, Control,
    Measurement>;

using Updater = ufil::update::dimension::DimensionGridmapUpdater<Track>;


// Helper to create a simple track
Track createTrack(ufil::type::Scalar length = 1.0, ufil::type::Scalar width = 1.0)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  state.x() = 0.0;
  state.y() = 0.0;

  Dimension dim;
  dim.length() = length;
  dim.width() = width;

  Track track;
  track.insert({state, dim});
  return track;
}

// Helper to create measurement
Measurement createMeasurement(
  ufil::type::Scalar length, ufil::type::Scalar width,
  ufil::type::Scalar var = 0.01)
{
  Measurement m;
  m.x() = 0.0;
  m.y() = 0.0;

  m.dimension().length() = length;
  m.dimension().width() = width;

  m.dimension().covariance() = Dimension::CovarianceMatrixType::Identity() * var;

  return m;
}

// Check that probabilities in the gridmap are normalized to 1
TEST(DimensionGridmapUpdaterTest, ProbabilitiesNormalized)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto meas = createMeasurement(2.0, 1.0);

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  updater.update(track, meas, timestamp, result);

  const auto & buffer = result.dimensionGridmapBuffer().at(0);

  ufil::type::Scalar sum = 0.0;
  for (const auto & cell : buffer) {
    sum += cell.probability;
  }

  ASSERT_NEAR(sum, 1.0, 1e-6);
}

// Check that probabilities are zero after cutoff
TEST(DimensionGridmapUpdaterTest, TailProbabilitiesAreZero)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto meas = createMeasurement(1.0, 1.0);

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  updater.update(track, meas, timestamp, result);

  const auto & buffer = result.dimensionGridmapBuffer().at(0);

  bool seen_zero = false;

  for (size_t i = 1; i < buffer.size(); ++i) {
    if (buffer[i].probability == 0.0) {
      seen_zero = true;
    }

    if (seen_zero) {
      EXPECT_NEAR(buffer[i].probability, 0.0, 1e-6)
        << "All probabilities after cutoff must be zero";
    }
  }
}

// Check that the estimate moves towards the measurement
TEST(DimensionGridmapUpdaterTest, EstimateMovesTowardsMeasurement)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  auto meas1 = createMeasurement(1.0, 1.0);
  updater.update(track, meas1, timestamp, result);
  ufil::type::Scalar estimate1 = result.dimensionVector()(0);

  auto meas2 = createMeasurement(3.0, 1.0);
  updater.update(track, meas2, timestamp, result);
  ufil::type::Scalar estimate2 = result.dimensionVector()(0);

  ASSERT_GT(estimate2, estimate1)
    << "Estimate should move toward new measurement " << "(estimate1=" << estimate1 <<
    ", estimate2=" << estimate2 << ")";
}

// Check that variance remains non-negative and finite after update
TEST(DimensionGridmapUpdaterTest, VarianceIsNonNegative)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  auto meas = createMeasurement(2.0, 1.0);
  updater.update(track, meas, timestamp, result);

  ufil::type::Scalar var = result.covariance()(0, 0);
  ufil::type::Scalar estimte = result.dimensionVector()(0);

  // // Print gridmap buffer to help debug if this test fails
  // const auto & buffer = result.dimensionGridmapBuffer().at(0);
  // std::cerr << "Gridmap buffer after update:" << std::endl;
  // for (size_t i = 0; i < 100; ++i) {
  //   std::cerr << "Cell " << i << ": probability=" << buffer[i].probability << std::endl;
  // }

  ASSERT_NEAR(estimte, 2.0, 0.1) << "Estimate should be close to measurement";
  ASSERT_GE(var, 0.0) << "Variance should be non-negative";
  ASSERT_TRUE(std::isfinite(var));
}

// Check that a measurement identical to the track does not change the estimate
TEST(DimensionGridmapUpdaterTest, IdentityMeasurementDoesNotChangeEstimate)
{
  Updater updater;

  Track track = createTrack(2.0, 1.0);
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  auto meas = createMeasurement(2.0, 1.0, 0.1);
  updater.update(track, meas, timestamp, result);

  ufil::type::Scalar estimate = result.dimensionVector()(0);
  ASSERT_NEAR(estimate, 2.0, 0.1)
      << "Estimate should remain the same for identical measurement " << "(estimate=" << estimate <<
    ")";
}

// Check that very large measurements do not produce NaNs
TEST(DimensionGridmapUpdaterTest, LargeMeasurementDoesNotCrash)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  auto meas = createMeasurement(1e6, 1e6);
  updater.update(track, meas, timestamp, result);

  ufil::type::Scalar estimate = result.dimensionVector()(0);
  ASSERT_TRUE(std::isfinite(estimate))
      << "Estimate must remain finite even for extremely large measurements";
}

// Check that very small measurements do not produce negative probabilities
TEST(DimensionGridmapUpdaterTest, TinyMeasurementNoNegativeProbabilities)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  auto meas = createMeasurement(1e-6, 1.0);
  updater.update(track, meas, timestamp, result);

  const auto & buffer = result.dimensionGridmapBuffer().at(0);
  for (const auto & cell : buffer) {
    ASSERT_GE(cell.probability, 0.0)
        << "Probability must not be negative";
  }
}

// Check that the sum of probabilities remains 1 after multiple updates
TEST(DimensionGridmapUpdaterTest, ProbabilitiesNormalizedAfterMultipleUpdates)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  for (ufil::type::Scalar l : {1.0, 2.0, 0.5, 1.5}) {
    auto meas = createMeasurement(l, 1.0);
    updater.update(track, meas, timestamp, result);
  }

  const auto & buffer = result.dimensionGridmapBuffer().at(0);
  ufil::type::Scalar sum = 0.0;
  for (const auto & cell : buffer) {
    sum += cell.probability;
}

  ASSERT_NEAR(sum, 1.0, 1e-6)
      << "Sum of probabilities must remain normalized after multiple updates";
}

// Check that measurement variance affects the grid update
TEST(DimensionGridmapUpdaterTest, HigherVarianceSpreadsProbabilities)
{
  Updater updater;

  Track track = createTrack();
  Dimension result_low_var, result_high_var;
  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);


  auto meas_low = createMeasurement(1.0, 1.0, 0.001);
  auto meas_high = createMeasurement(1.0, 1.0, 1.0);

  updater.update(track, meas_low, timestamp, result_low_var);
  updater.update(track, meas_high, timestamp, result_high_var);

  const auto & buffer_low = result_low_var.dimensionGridmapBuffer().at(0);
  const auto & buffer_high = result_high_var.dimensionGridmapBuffer().at(0);

  // High variance should have smaller max probability (more spread)
  ufil::type::Scalar max_low = 0.0, max_high = 0.0;
  for (size_t i = 0; i < buffer_low.size(); ++i) {
    if (buffer_low[i].probability > max_low) {max_low = buffer_low[i].probability;}
    if (buffer_high[i].probability > max_high) {max_high = buffer_high[i].probability;}
  }

  ASSERT_LT(max_high, max_low)
      << "Higher measurement variance should spread probability over more cells";
}

TEST(DimensionGridmapUpdaterTest, OscillatingMeasurementsRemainStable)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  for (int i = 0; i < 50; ++i) {
    auto meas = createMeasurement((i % 2 == 0) ? 1.0 : 5.0, 1.0);
    updater.update(track, meas, timestamp, result);

    auto estimate = result.dimensionVector()(0);

    ASSERT_TRUE(std::isfinite(estimate));
    ASSERT_LT(estimate, 20.0)
        << "Estimate exploded during oscillating updates: " << estimate;
  }
}

TEST(DimensionGridmapUpdaterTest, RepeatedSameMeasurementDoesNotDrift)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  auto meas = createMeasurement(2.0, 1.0);

  for (int i = 0; i < 50; ++i) {
    updater.update(track, meas, timestamp, result);
    auto estimate = result.dimensionVector()(0);

    ASSERT_TRUE(std::isfinite(estimate));
    ASSERT_NEAR(estimate, 2.0, 0.1)
        << "Estimate drifting under identical measurements";
  }
}

TEST(DimensionGridmapUpdaterTest, NoProbabilityMassAtFarCells)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  auto meas = createMeasurement(1.0, 1.0);

  updater.update(track, meas, timestamp, result);

  const auto & buffer = result.dimensionGridmapBuffer().at(0);

  for (size_t i = 0; i < buffer.size(); ++i) {
    if (buffer[i].position > 10.0) {
      ASSERT_LT(buffer[i].probability, 1e-6)
          << "Far cells should not carry probability mass (i=" << i << ")";
    }
  }
}

TEST(DimensionGridmapUpdaterTest, CDFIsMonotonic)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto meas = createMeasurement(2.0, 1.0);
  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  updater.update(track, meas, timestamp, result);

  const auto & buffer = result.dimensionGridmapBuffer().at(0);

  ufil::type::Scalar cumulative = 0.0;

  for (size_t i = 0; i < buffer.size(); ++i) {
    cumulative += buffer[i].probability;

    ASSERT_LE(cumulative, 1.000001)
        << "CDF exceeded 1 → invalid probability construction";
  }
}

TEST(DimensionGridmapUpdaterTest, NoExtremeEstimateJump)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  auto meas1 = createMeasurement(1.0, 1.0);
  updater.update(track, meas1, timestamp, result);

  auto meas2 = createMeasurement(1.1, 1.0);
  updater.update(track, meas2, timestamp, result);

  auto estimate = result.dimensionVector()(0);

  ASSERT_LT(estimate, 10.0)
      << "Estimate jumped unreasonably: " << estimate;
}

TEST(DimensionGridmapUpdaterTest, RandomStressTest)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(0.01, 10.0);

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  for (int i = 0; i < 200; ++i) {
    double m = dist(rng);
    double var = dist(rng) * 0.1;

    auto meas = createMeasurement(m, 1.0, var);
    updater.update(track, meas, timestamp, result);

    auto estimate = result.dimensionVector()(0);

    ASSERT_TRUE(std::isfinite(estimate));
    ASSERT_GE(estimate, 0.0);
    ASSERT_LT(estimate, 100.0)
        << "Estimate exploded in stress test: " << estimate;
  }
}

TEST(DimensionGridmapUpdaterTest, NearZeroVarianceExplosion)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  // extremely confident measurement
  auto meas = createMeasurement(2.0, 1.0, 1e-12);

  updater.update(track, meas, timestamp, result);

  auto estimate = result.dimensionVector()(0);

  ASSERT_TRUE(std::isfinite(estimate));
  ASSERT_LT(estimate, 100.0)
      << "Estimate exploded with near-zero variance: " << estimate;
}

TEST(DimensionGridmapUpdaterTest, LogOddsOverflow)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  auto meas = createMeasurement(2.0, 1.0, 0.001);

  for (int i = 0; i < 500; ++i) {
    updater.update(track, meas, timestamp, result);
  }

  auto estimate = result.dimensionVector()(0);

  ASSERT_TRUE(std::isfinite(estimate))
      << "Estimate became NaN/Inf after repeated updates";

  ASSERT_LT(estimate, 50.0)
      << "Estimate drifted excessively: " << estimate;
}

// Expect that if the measurement is out of buffer range the probabilities are set to zero and the
// estimate is not NaN or negative
TEST(DimensionGridmapUpdaterTest, DegenerateNormalization)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  // Measurement very far away → probabilities ~0 everywhere
  auto meas = createMeasurement(1e9, 1.0);

  updater.update(track, meas, timestamp, result);

  const auto & buffer = result.dimensionGridmapBuffer().at(0);

  double sum = 0.0;
  for (const auto & cell : buffer) {
    sum += cell.probability;
  }

  ASSERT_NEAR(sum, 0.0, 1e-6);

  auto estimate = result.dimensionVector()(0);

  ASSERT_TRUE(std::isfinite(estimate))
      << "Estimate broke after degenerate normalization";
}

TEST(DimensionGridmapUpdaterTest, NoCDFInversion)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  auto meas = createMeasurement(2.0, 1.0, 0.01);
  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  updater.update(track, meas, timestamp, result);

  const auto & buffer = result.dimensionGridmapBuffer().at(0);

  for (size_t i = 1; i < buffer.size(); ++i) {
    ASSERT_GE(buffer[i - 1].probability + 1e-12, 0.0);
  }
}

TEST(DimensionGridmapUpdaterTest, ChaosTest)
{
  Updater updater;

  Track track = createTrack();
  Dimension result;

  std::mt19937 rng(123);

  std::uniform_real_distribution<double> meas_dist(0.0, 50.0);
  std::uniform_real_distribution<double> var_dist(1e-12, 10.0);

  auto timestamp = ufil::from_seconds<ufil::type::Timestamp>(0.0);

  for (int i = 0; i < 1000; ++i) {
    double m = meas_dist(rng);
    double v = var_dist(rng);

    auto meas = createMeasurement(m, 1.0, v);

    updater.update(track, meas, timestamp, result);

    auto estimate = result.dimensionVector()(0);

    if (!std::isfinite(estimate) || estimate > 1000.0) {
      FAIL() << "Explosion detected at iteration " << i
             << " (m=" << m << ", v=" << v << ", est=" << estimate << ")";
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
