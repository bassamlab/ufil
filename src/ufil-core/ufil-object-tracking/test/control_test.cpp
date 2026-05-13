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

#include <ufil_object_tracking/types/control.hpp>

TEST(Control, defaultConstructor)
{
  ufil::type::Control<3> control;

  // Verify that the control vector is initialized to zero
  EXPECT_TRUE(control.controlVector().isZero());
}

TEST(Control, modifyControlVector)
{
  ufil::type::Control<3> control;

  // Modify the control vector
  control.controlVector() << 1.0, 2.0, 3.0;

  // Verify the values
  EXPECT_FLOAT_EQ(control.controlVector()[0], 1.0);
  EXPECT_FLOAT_EQ(control.controlVector()[1], 2.0);
  EXPECT_FLOAT_EQ(control.controlVector()[2], 3.0);
}

TEST(None, defaultConstructor)
{
  ufil::type::control::None none;

  // Verify that the control vector is empty (size 0)
  EXPECT_EQ(none.controlVector().size(), 0);
}

TEST(Acceleration2D, defaultConstructor)
{
  ufil::type::control::Acceleration2D acceleration;

  // Verify that the control vector is initialized to zero
  EXPECT_TRUE(acceleration.controlVector().isZero());
}

TEST(Acceleration2D, accessors)
{
  ufil::type::control::Acceleration2D acceleration;

  // Set values
  acceleration.ax() = 5.0;
  acceleration.ay() = 3.0;

  // Verify values
  EXPECT_FLOAT_EQ(acceleration.ax(), 5.0);
  EXPECT_FLOAT_EQ(acceleration.ay(), 3.0);
}

TEST(Acceleration2D, modifyControlVector)
{
  ufil::type::control::Acceleration2D acceleration;

  // Modify the control vector directly
  acceleration.controlVector() << 7.0, 4.0;

  // Verify values using accessors
  EXPECT_FLOAT_EQ(acceleration.ax(), 7.0);
  EXPECT_FLOAT_EQ(acceleration.ay(), 4.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
