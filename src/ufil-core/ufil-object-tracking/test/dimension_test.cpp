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

#include <ufil_object_tracking/types/dimension.hpp>

TEST(Dimension, defaultConstructor)
{
  ufil::type::Dimension<3> dimension;

  // Verify that the dimension vector is initialized to zero
  EXPECT_TRUE(dimension.dimensionVector().isZero());

  // Verify that the covariance matrix is initialized to zero
  EXPECT_TRUE(dimension.covariance().isZero());

  // Verify that the gridmap buffer is empty
  EXPECT_TRUE(dimension.dimensionGridmapBuffer().empty());
}

TEST(Dimension, modifyDimensionVector)
{
  ufil::type::Dimension<3> dimension;

  // Modify the dimension vector
  dimension.dimensionVector() << 1.0, 2.0, 3.0;

  // Verify the values
  EXPECT_FLOAT_EQ(dimension.dimensionVector()[0], 1.0);
  EXPECT_FLOAT_EQ(dimension.dimensionVector()[1], 2.0);
  EXPECT_FLOAT_EQ(dimension.dimensionVector()[2], 3.0);
}

TEST(Dimension, modifyCovarianceMatrix)
{
  ufil::type::Dimension<3> dimension;

  // Modify the covariance matrix
  dimension.covariance()(0, 0) = 1.0;
  dimension.covariance()(1, 1) = 2.0;
  dimension.covariance()(2, 2) = 3.0;

  // Verify the values
  EXPECT_FLOAT_EQ(dimension.covariance()(0, 0), 1.0);
  EXPECT_FLOAT_EQ(dimension.covariance()(1, 1), 2.0);
  EXPECT_FLOAT_EQ(dimension.covariance()(2, 2), 3.0);
}

TEST(Dimension2D, defaultConstructor)
{
  ufil::type::dimension::Dimension2D dimension;

  // Verify that the dimension vector is initialized to zero
  EXPECT_TRUE(dimension.dimensionVector().isZero());

  // Verify that the gridmap buffer has two empty rows
  EXPECT_EQ(dimension.dimensionGridmapBuffer().size(), 2);
  EXPECT_TRUE(dimension.dimensionGridmapBuffer()[0].empty());
  EXPECT_TRUE(dimension.dimensionGridmapBuffer()[1].empty());
}

TEST(Dimension2D, accessors)
{
  ufil::type::dimension::Dimension2D dimension;

  // Set values
  dimension.length() = 5.0;
  dimension.width() = 3.0;

  // Verify values
  EXPECT_FLOAT_EQ(dimension.length(), 5.0);
  EXPECT_FLOAT_EQ(dimension.width(), 3.0);
}

TEST(Dimension3D, defaultConstructor)
{
  ufil::type::dimension::Dimension3D dimension;

  // Verify that the dimension vector is initialized to zero
  EXPECT_TRUE(dimension.dimensionVector().isZero());

  // Verify that the gridmap buffer has three empty rows
  EXPECT_EQ(dimension.dimensionGridmapBuffer().size(), 3);
  EXPECT_TRUE(dimension.dimensionGridmapBuffer()[0].empty());
  EXPECT_TRUE(dimension.dimensionGridmapBuffer()[1].empty());
  EXPECT_TRUE(dimension.dimensionGridmapBuffer()[2].empty());
}

TEST(Dimension3D, accessors)
{
  ufil::type::dimension::Dimension3D dimension;

  // Set values
  dimension.length() = 7.0;
  dimension.width() = 4.0;
  dimension.height() = 2.0;

  // Verify values
  EXPECT_FLOAT_EQ(dimension.length(), 7.0);
  EXPECT_FLOAT_EQ(dimension.width(), 4.0);
  EXPECT_FLOAT_EQ(dimension.height(), 2.0);
}

TEST(DimensionGridmapCell, defaultConstructor)
{
  ufil::type::DimensionGridmapCell cell;

  // Verify default values
  EXPECT_FLOAT_EQ(cell.position, 0.0);
  EXPECT_FLOAT_EQ(cell.log_odds, 0.0);
  EXPECT_FLOAT_EQ(cell.probability, 0.0);
}

TEST(DimensionGridmapCell, modifyValues)
{
  ufil::type::DimensionGridmapCell cell;

  // Modify values
  cell.position = 1.5;
  cell.log_odds = 0.8;
  cell.probability = 0.3;

  // Verify values
  EXPECT_FLOAT_EQ(cell.position, 1.5);
  EXPECT_FLOAT_EQ(cell.log_odds, 0.8);
  EXPECT_FLOAT_EQ(cell.probability, 0.3);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
