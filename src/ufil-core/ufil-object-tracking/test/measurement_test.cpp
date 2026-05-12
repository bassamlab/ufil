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

#include <ufil_object_tracking/types/measurement.hpp>

TEST(Measurement, position2dSetAndGet)
{
  ufil::type::measurement::Position2D measurement;
  EXPECT_EQ(measurement.x(), 0.0f);
  EXPECT_EQ(measurement.y(), 0.0f);

  measurement.x() = 1.1f;
  measurement.y() = 2.2f;

  const auto & vec = measurement.measurementVector();
  EXPECT_EQ(vec.size(), 2);
  EXPECT_EQ(vec[ufil::type::measurement::Position2D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::measurement::Position2D::Y], 2.2f);

  const auto pos = measurement.position();
  EXPECT_EQ(pos.size(), 2);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);

  EXPECT_TRUE(measurement.covariance().isIdentity());
}

TEST(Measurement, position3dSetAndGet)
{
  ufil::type::measurement::Position3D measurement;
  EXPECT_EQ(measurement.x(), 0.0f);
  EXPECT_EQ(measurement.y(), 0.0f);
  EXPECT_EQ(measurement.z(), 0.0f);

  measurement.x() = 1.1f;
  measurement.y() = 2.2f;
  measurement.z() = 3.3f;

  const auto & vec = measurement.measurementVector();
  EXPECT_EQ(vec.size(), 3);
  EXPECT_EQ(vec[ufil::type::measurement::Position3D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::measurement::Position3D::Y], 2.2f);
  EXPECT_EQ(vec[ufil::type::measurement::Position3D::Z], 3.3f);

  const auto pos = measurement.position();
  EXPECT_EQ(pos.size(), 3);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);
  EXPECT_EQ(pos[2], 3.3f);

  EXPECT_TRUE(measurement.covariance().isIdentity());
}

TEST(Measurement, pose2dSetAndGet)
{
  ufil::type::measurement::Pose2D measurement;
  EXPECT_EQ(measurement.x(), 0.0f);
  EXPECT_EQ(measurement.y(), 0.0f);
  EXPECT_EQ(measurement.yaw(), 0.0f);

  measurement.x() = 1.1f;
  measurement.y() = 2.2f;
  measurement.yaw() = 3.3f;

  const auto & vec = measurement.measurementVector();
  EXPECT_EQ(vec.size(), 3);
  EXPECT_EQ(vec[ufil::type::measurement::Pose2D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::measurement::Pose2D::Y], 2.2f);
  EXPECT_EQ(vec[ufil::type::measurement::Pose2D::YAW], 3.3f);

  const auto pos = measurement.position();
  EXPECT_EQ(pos.size(), 2);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);

  EXPECT_TRUE(measurement.covariance().isIdentity());
}

TEST(measurement, pose2d_with_dimension2d_set_and_get)
{
  ufil::type::measurement::Pose2DWithDimension2D measurement;
  EXPECT_EQ(measurement.x(), 0.0f);
  EXPECT_EQ(measurement.y(), 0.0f);
  EXPECT_EQ(measurement.yaw(), 0.0f);
  EXPECT_EQ(measurement.dimension().length(), 0.0f);
  EXPECT_EQ(measurement.dimension().width(), 0.0f);

  measurement.x() = 1.1f;
  measurement.y() = 2.2f;
  measurement.yaw() = 3.3f;
  measurement.dimension().length() = 4.4f;
  measurement.dimension().width() = 5.5f;

  const auto & vec = measurement.measurementVector();
  EXPECT_EQ(vec.size(), 3);
  EXPECT_EQ(vec[ufil::type::measurement::Pose2D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::measurement::Pose2D::Y], 2.2f);
  EXPECT_EQ(vec[ufil::type::measurement::Pose2D::YAW], 3.3f);

  const auto pos = measurement.position();
  EXPECT_EQ(pos.size(), 2);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);

  const auto dim = measurement.dimension().dimensionVector();
  EXPECT_EQ(dim.size(), 2);
  EXPECT_EQ(dim[0], 4.4f);
  EXPECT_EQ(dim[1], 5.5f);

  EXPECT_TRUE(measurement.covariance().isIdentity());
}

TEST(measurement, pose2d_with_dimension3d_set_and_get)
{
  ufil::type::measurement::Pose2DWithDimension3D measurement;
  EXPECT_EQ(measurement.x(), 0.0f);
  EXPECT_EQ(measurement.y(), 0.0f);
  EXPECT_EQ(measurement.yaw(), 0.0f);
  EXPECT_EQ(measurement.dimension().length(), 0.0f);
  EXPECT_EQ(measurement.dimension().width(), 0.0f);
  EXPECT_EQ(measurement.dimension().height(), 0.0f);

  measurement.x() = 1.1f;
  measurement.y() = 2.2f;
  measurement.yaw() = 3.3f;
  measurement.dimension().length() = 4.4f;
  measurement.dimension().width() = 5.5f;
  measurement.dimension().height() = 6.6f;

  const auto & vec = measurement.measurementVector();
  EXPECT_EQ(vec.size(), 3);
  EXPECT_EQ(vec[ufil::type::measurement::Pose2D::X], 1.1f);
  EXPECT_EQ(vec[ufil::type::measurement::Pose2D::Y], 2.2f);
  EXPECT_EQ(vec[ufil::type::measurement::Pose2D::YAW], 3.3f);

  const auto pos = measurement.position();
  EXPECT_EQ(pos.size(), 2);
  EXPECT_EQ(pos[0], 1.1f);
  EXPECT_EQ(pos[1], 2.2f);

  const auto dim = measurement.dimension().dimensionVector();
  EXPECT_EQ(dim.size(), 3);
  EXPECT_EQ(dim[0], 4.4f);
  EXPECT_EQ(dim[1], 5.5f);
  EXPECT_EQ(dim[2], 6.6f);

  EXPECT_TRUE(measurement.covariance().isIdentity());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
