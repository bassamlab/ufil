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
#include <limits>

#include <ufil_object_tracking/types/scalar.hpp>

using ufil::type::Scalar;
namespace scalar = ufil::scalar;

TEST(Scalar, scalarConstantsAreCorrect)
{
  EXPECT_TRUE(std::isinf(scalar::ScalarInf));
  EXPECT_FLOAT_EQ(scalar::ScalarMax, std::numeric_limits<Scalar>::max());
  EXPECT_FLOAT_EQ(scalar::ScalarMin, std::numeric_limits<Scalar>::min());
}

TEST(Scalar, isInfiniteDetectsInfinity)
{
  EXPECT_TRUE(scalar::is_infinite(scalar::ScalarInf));
  EXPECT_TRUE(scalar::is_infinite(-scalar::ScalarInf));
  EXPECT_FALSE(scalar::is_infinite(0.0f));
  EXPECT_FALSE(scalar::is_infinite(scalar::ScalarMax));
}

TEST(Scalar, isNanDetectsNaN)
{
  EXPECT_TRUE(scalar::is_nan(std::numeric_limits<Scalar>::quiet_NaN()));
  EXPECT_FALSE(scalar::is_nan(0.0f));
  EXPECT_FALSE(scalar::is_nan(scalar::ScalarInf));
}

TEST(Scalar, isValidWorksCorrectly)
{
  EXPECT_TRUE(scalar::is_valid(1.0f));
  EXPECT_TRUE(scalar::is_valid(scalar::ScalarMax));
  EXPECT_FALSE(scalar::is_valid(scalar::ScalarInf));
  EXPECT_FALSE(scalar::is_valid(std::numeric_limits<Scalar>::quiet_NaN()));
}

TEST(Scalar, isInvalidIsNegationOfIsValid)
{
  Scalar values[] = {1.0f, scalar::ScalarMax, scalar::ScalarInf,
    std::numeric_limits<Scalar>::quiet_NaN()};
  for (const auto & v : values) {
    EXPECT_EQ(scalar::is_invalid(v), !scalar::is_valid(v));
  }
}

TEST(Scalar, clampReturnsCorrectValue)
{
  EXPECT_FLOAT_EQ(scalar::clamp(5.0f, 1.0f, 10.0f), 5.0f);
  EXPECT_FLOAT_EQ(scalar::clamp(-1.0f, 0.0f, 2.0f), 0.0f);
  EXPECT_FLOAT_EQ(scalar::clamp(5.0f, 6.0f, 8.0f), 6.0f);
}

TEST(Scalar, clampThrowsOnInvalidInput)
{
  Scalar nan = std::numeric_limits<Scalar>::quiet_NaN();
  Scalar inf = scalar::ScalarInf;

  EXPECT_THROW(scalar::clamp(nan, 0.0f, 1.0f), std::invalid_argument);
  EXPECT_THROW(scalar::clamp(0.5f, nan, 1.0f), std::invalid_argument);
  EXPECT_THROW(scalar::clamp(0.5f, 0.0f, nan), std::invalid_argument);
  EXPECT_THROW(scalar::clamp(inf, 0.0f, 1.0f), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
