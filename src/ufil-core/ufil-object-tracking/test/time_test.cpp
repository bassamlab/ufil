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

#include <limits>

#include <ufil_object_tracking/types/time.hpp>

TEST(Time, timestampFromAndToNanoseconds)
{
  auto time = ufil::from_nanoseconds<ufil::type::Timestamp>(1234567890);
  int64_t nanoseconds = ufil::to_nanoseconds(time);
  EXPECT_EQ(nanoseconds, 1234567890);
}

TEST(Time, durationFromAndToNanoseconds)
{
  auto duration = ufil::from_nanoseconds<ufil::type::Duration>(1234567890);
  int64_t nanoseconds = ufil::to_nanoseconds(duration);
  EXPECT_EQ(nanoseconds, 1234567890);
}

TEST(Time, durationFromAndToSeconds)
{
  auto duration = ufil::from_seconds<ufil::type::Duration>(12.456789);
  double seconds = ufil::to_seconds(duration);
  EXPECT_EQ(seconds, 12.456789);
}

TEST(Time, zeroTimestampConversion)
{
  auto time = ufil::from_nanoseconds<ufil::type::Timestamp>(0);
  int64_t nanoseconds = ufil::to_nanoseconds(time);
  EXPECT_EQ(nanoseconds, 0);
}

TEST(Time, zeroDurationConversion)
{
  auto duration = ufil::from_nanoseconds<ufil::type::Duration>(0);
  int64_t nanoseconds = ufil::to_nanoseconds(duration);
  EXPECT_EQ(nanoseconds, 0);
}

TEST(Time, negativeDurationSecondsConversion)
{
  auto duration = ufil::from_seconds<ufil::type::Duration>(-1.5);
  double seconds = ufil::to_seconds(duration);
  EXPECT_NEAR(seconds, -1.5, 1e-9);
}

TEST(Time, fractionalSecondRounding)
{
  double input = 0.123456789;
  auto duration = ufil::from_seconds<ufil::type::Duration>(input);
  double output = ufil::to_seconds(duration);
  EXPECT_NEAR(output, input, 1e-9);
}

TEST(Time, largeNanosecondTimestamp)
{
  // Avoid overflow
  int64_t large_ns = std::numeric_limits<int64_t>::max() / 1000;
  auto timestamp = ufil::from_nanoseconds<ufil::type::Timestamp>(large_ns);
  int64_t out = ufil::to_nanoseconds(timestamp);
  EXPECT_EQ(out, large_ns);
}

TEST(Time, largeNegativeDuration)
{
  int64_t large_negative = std::numeric_limits<int64_t>::min() / 1000;
  auto duration = ufil::from_nanoseconds<ufil::type::Duration>(large_negative);
  int64_t out = ufil::to_nanoseconds(duration);
  EXPECT_EQ(out, large_negative);
}

TEST(Time, consistencyBetweenNanosecondsAndSeconds)
{
  int64_t input_ns = 9876543210;
  auto duration = ufil::from_nanoseconds<ufil::type::Duration>(input_ns);
  double seconds = ufil::to_seconds(duration);
  auto duration_back = ufil::from_seconds<ufil::type::Duration>(seconds);
  // off-by-1 possible due to rounding
  EXPECT_NEAR(ufil::to_nanoseconds(duration_back), input_ns, 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
