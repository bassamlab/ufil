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
#include <ufil_object_tracking/types/history_entry.hpp>
#include <ufil_object_tracking/types/classification.hpp>
#include <ufil_object_tracking/types/control.hpp>
#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/measurement.hpp>
#include <ufil_object_tracking/types/existence_probability.hpp>
#include <ufil_object_tracking/types/state.hpp>

using State = ufil::type::state::Pose2D;
using Dimension = ufil::type::dimension::Dimension2D;
using Classification = ufil::type::classification::ObjectClassification;
using Control = ufil::type::control::None;
using Measurement = ufil::type::measurement::Pose2D;
using ExistenceProbability = ufil::type::ExistenceProbability;
using HistoryEntry = ufil::type::HistoryEntry<State, Dimension, Classification,
    ExistenceProbability, Control,
    Measurement>;

TEST(HistoryEntry, defaultConstructor)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  Dimension dimension;

  HistoryEntry entry(state, dimension);

  EXPECT_EQ(entry.state().timestamp(), state.timestamp());
  EXPECT_FLOAT_EQ(entry.existenceProbability().existence(), 0.5);

  // Check if the classification vector is all zeros
  EXPECT_TRUE(entry.classification().classificationVector().isZero());

  EXPECT_FALSE(entry.hasControlInput());
  EXPECT_FALSE(entry.hasAssociatedMeasurement());

  // Verify that the dimension is assigned (indirectly, if possible)
  EXPECT_NO_THROW(auto dimension = entry.dimension());
}

TEST(HistoryEntry, parameterizedConstructor)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(1234567890));
  Dimension dimension;
  ExistenceProbability existence_probability;
  existence_probability.existence() = 0.9;
  Classification classification;
  classification.classificationVector() << 1.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  Control control_input;
  Measurement measurement;

  HistoryEntry entry(state, dimension, existence_probability, classification, control_input,
    measurement);

  EXPECT_EQ(entry.state().timestamp(), state.timestamp());
  EXPECT_FLOAT_EQ(entry.existenceProbability().existence(), 0.9);
  EXPECT_EQ(entry.classification().classificationVector(), classification.classificationVector());
  EXPECT_TRUE(entry.hasControlInput());
  EXPECT_TRUE(entry.hasAssociatedMeasurement());

  // Verify that the dimension is assigned (indirectly, if possible)
  EXPECT_NO_THROW(auto dimension = entry.dimension());
}

TEST(HistoryEntry, modifyState)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  Dimension dimension;

  HistoryEntry entry(state, dimension);

  entry.state().x() = 1.0f;
  entry.state().y() = 2.0f;

  EXPECT_FLOAT_EQ(entry.state().x(), 1.0f);
  EXPECT_FLOAT_EQ(entry.state().y(), 2.0f);
}

TEST(HistoryEntry, modifyDimension)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  Dimension dimension;
  dimension.width() = 1.0f;
  dimension.length() = 4.0f;

  HistoryEntry entry(state, dimension);

  EXPECT_FLOAT_EQ(entry.dimension().width(), 1.0f);
  EXPECT_FLOAT_EQ(entry.dimension().length(), 4.0f);

  Dimension new_dimension;
  new_dimension.width() = 2.0f;
  new_dimension.length() = 5.0f;
  entry.dimension() = new_dimension;

  EXPECT_FLOAT_EQ(entry.dimension().width(), 2.0f);
  EXPECT_FLOAT_EQ(entry.dimension().length(), 5.0f);
}

TEST(HistoryEntry, modifyExistenceProbability)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  Dimension dimension;

  HistoryEntry entry(state, dimension);

  entry.existenceProbability().existence() = 0.8;

  EXPECT_FLOAT_EQ(entry.existenceProbability().existence(), 0.8);
}

TEST(HistoryEntry, modifyClassification)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  Dimension dimension;

  HistoryEntry entry(state, dimension);

  Classification classification;
  classification.classificationVector() << 3.0f, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      0.0f;
  entry.classification() = classification;

  EXPECT_EQ(entry.classification().classificationVector(), classification.classificationVector());
}

TEST(HistoryEntry, controlInputHandling)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  Dimension dimension;

  HistoryEntry entry(state, dimension);

  EXPECT_FALSE(entry.hasControlInput());

  Control control_input;
  entry.controlInput() = control_input;

  EXPECT_TRUE(entry.hasControlInput());
  EXPECT_TRUE(entry.controlInput().has_value());
}

TEST(HistoryEntry, associatedMeasurementHandling)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  Dimension dimension;

  HistoryEntry entry(state, dimension);

  EXPECT_FALSE(entry.hasAssociatedMeasurement());

  Measurement measurement;
  entry.associatedMeasurement() = measurement;

  EXPECT_TRUE(entry.hasAssociatedMeasurement());
  EXPECT_TRUE(entry.associatedMeasurement().has_value());
}

TEST(HistoryEntry, resetControlInput)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  Dimension dimension;

  HistoryEntry entry(state, dimension);

  Control control_input;
  entry.controlInput() = control_input;

  EXPECT_TRUE(entry.hasControlInput());

  entry.controlInput().reset();

  EXPECT_FALSE(entry.hasControlInput());
}

TEST(HistoryEntry, resetAssociatedMeasurement)
{
  State state(ufil::from_nanoseconds<ufil::type::Timestamp>(0));
  Dimension dimension;

  HistoryEntry entry(state, dimension);

  Measurement measurement;
  entry.associatedMeasurement() = measurement;

  EXPECT_TRUE(entry.hasAssociatedMeasurement());

  entry.associatedMeasurement().reset();

  EXPECT_FALSE(entry.hasAssociatedMeasurement());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
