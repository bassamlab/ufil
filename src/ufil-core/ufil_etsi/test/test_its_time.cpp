// Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#include <chrono>

#include "ufil_etsi/its_time.hpp"

// Core library tests, no ROS 2 required.

using std::chrono::milliseconds;
using std::chrono::seconds;
using ufil_etsi::Time;

TEST(ItsTime, ItsTimeCountsFrom2004IncludingLeapSeconds)
{
  EXPECT_EQ(ufil_etsi::itsTimeFromUnixTime(ufil_etsi::kItsEpoch), 5000u);
  EXPECT_EQ(ufil_etsi::itsTimeFromUnixTime(ufil_etsi::kItsEpoch + milliseconds(1234)), 6234u);
  // Same value as etsi_its_msgs_utils: 2024-01-01T00:00:00Z.
  EXPECT_EQ(
    ufil_etsi::itsTimeFromUnixTime(seconds(1704067200)), (1704067200ull - 1072915200 + 5) * 1000);
  EXPECT_FALSE(ufil_etsi::itsTimeFromUnixTime(Time(0)).has_value());
  EXPECT_FALSE(ufil_etsi::generationDeltaTime(Time(0)).has_value());
}

TEST(ItsTime, GenerationDeltaTimeIsModulo65536)
{
  const Time time = ufil_etsi::kItsEpoch - seconds(5) + milliseconds(65536 * 3 + 100);
  EXPECT_EQ(ufil_etsi::generationDeltaTime(time), 100);
  EXPECT_EQ(ufil_etsi::generationDeltaTime(time + std::chrono::microseconds(999)), 100);
}

TEST(ItsTime, ReconstructionPicksTheClosestTime)
{
  const Time generation = seconds(1704067200) + milliseconds(123);
  const auto gdt = *ufil_etsi::generationDeltaTime(generation);
  // Received shortly after or before (clock offset), also across a wrap of the 65.536 s.
  for (const auto offset : {milliseconds(0), milliseconds(50), milliseconds(-50),
      milliseconds(30000), milliseconds(-30000)})
  {
    EXPECT_EQ(ufil_etsi::unixTimeFromGenerationDeltaTime(gdt, generation + offset), generation)
      << offset.count();
  }
  const Time near_wrap = ufil_etsi::kItsEpoch - seconds(5) + milliseconds(65536 * 10 - 10);
  const auto gdt_near_wrap = *ufil_etsi::generationDeltaTime(near_wrap);
  EXPECT_EQ(
    ufil_etsi::unixTimeFromGenerationDeltaTime(gdt_near_wrap, near_wrap + milliseconds(20)),
    near_wrap);
  EXPECT_FALSE(ufil_etsi::unixTimeFromGenerationDeltaTime(0, Time(0)).has_value());
}
