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

#include <ufil_object_tracking/types/classification.hpp>

TEST(Classification, defaultConstructor)
{
  ufil::type::Classification<5> classification;

  // Verify that the classification vector is initialized to zero
  EXPECT_TRUE(classification.classificationVector().isZero());
}

TEST(Classification, modifyClassificationVector)
{
  ufil::type::Classification<5> classification;

  // Modify the classification vector
  classification.classificationVector() << 0.1, 0.2, 0.3, 0.4, 0.5;

  // Verify the values
  EXPECT_FLOAT_EQ(classification.classificationVector()[0], 0.1);
  EXPECT_FLOAT_EQ(classification.classificationVector()[1], 0.2);
  EXPECT_FLOAT_EQ(classification.classificationVector()[2], 0.3);
  EXPECT_FLOAT_EQ(classification.classificationVector()[3], 0.4);
  EXPECT_FLOAT_EQ(classification.classificationVector()[4], 0.5);
}

TEST(Classification, normalizeClassificationVector)
{
  ufil::type::Classification<3> classification;

  // Set values
  classification.classificationVector() << 1.0, 2.0, 3.0;

  // Normalize the vector
  classification.normalize();

  // Verify that the vector is normalized (magnitude = 1)
  EXPECT_FLOAT_EQ(classification.classificationVector().norm(), 1.0);
}

TEST(ObjectClassification, defaultConstructor)
{
  ufil::type::classification::ObjectClassification classification;

  // Verify that the classification vector is initialized to zero
  EXPECT_TRUE(classification.classificationVector().isZero());
}

TEST(ObjectClassification, accessors)
{
  ufil::type::classification::ObjectClassification classification;

  // Set values using accessors
  classification.car() = 0.3;
  classification.truck() = 0.2;
  classification.pedestrian() = 0.1;
  classification.bicycle() = 0.05;
  classification.motorcycle() = 0.15;
  classification.stationary() = 0.1;
  classification.other() = 0.1;

  // Verify values using accessors
  EXPECT_FLOAT_EQ(classification.car(), 0.3);
  EXPECT_FLOAT_EQ(classification.truck(), 0.2);
  EXPECT_FLOAT_EQ(classification.pedestrian(), 0.1);
  EXPECT_FLOAT_EQ(classification.bicycle(), 0.05);
  EXPECT_FLOAT_EQ(classification.motorcycle(), 0.15);
  EXPECT_FLOAT_EQ(classification.stationary(), 0.1);
  EXPECT_FLOAT_EQ(classification.other(), 0.1);
}

TEST(ObjectClassification, normalizeClassificationVector)
{
  ufil::type::classification::ObjectClassification classification;

  // Set values
  classification.car() = 1.0;
  classification.truck() = 2.0;
  classification.pedestrian() = 3.0;

  // Normalize the vector
  classification.normalize();

  // Verify that the vector is normalized (magnitude = 1)
  EXPECT_FLOAT_EQ(classification.classificationVector().norm(), 1.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
