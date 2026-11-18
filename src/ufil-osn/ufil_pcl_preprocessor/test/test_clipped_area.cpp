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

#include <numbers>
#include <sstream>
#include <stdexcept>

#include "ufil_pcl_preprocessor/clipped_area.hpp"

using ufil_pcl_preprocessor::ClippedArea;
using ufil_pcl_preprocessor::loadClippedAreasFromCsv;
using ufil_pcl_preprocessor::parseClippedAreasCsv;

TEST(ClippedArea, ContainsRespectsYaw)
{
  // 4 m long along x before rotation, rotated by 90 degrees: long along y.
  const ClippedArea area{1.0f, 2.0f, 0.0f, std::numbers::pi_v<float>/ 2.0f, 4.0f, 1.0f, 2.0f};
  EXPECT_TRUE(area.contains(1.0f, 2.0f, 0.0f));
  EXPECT_TRUE(area.contains(1.0f, 3.9f, 0.5f));
  EXPECT_FALSE(area.contains(2.9f, 2.0f, 0.0f));
  EXPECT_FALSE(area.contains(1.0f, 2.0f, 1.5f));
}

TEST(ClippedArea, ParsesCsvWithHeaderAndComments)
{
  std::istringstream csv(
    "x, y, z, yaw, length, width, height\n"
    "1 , 26.5, 1.5, 0, 4, 120, 3\n"
    "\n"
    "# comment\n"
    "38.5, 26.5, 5, 0.5, 40, 120, 10\r\n");
  const auto areas = parseClippedAreasCsv(csv);
  ASSERT_EQ(areas.size(), 2u);
  EXPECT_FLOAT_EQ(areas[0].x, 1.0f);
  EXPECT_FLOAT_EQ(areas[0].width, 120.0f);
  EXPECT_FLOAT_EQ(areas[1].yaw, 0.5f);
  EXPECT_FLOAT_EQ(areas[1].height, 10.0f);
}

TEST(ClippedArea, ParsesCsvWithoutHeader)
{
  std::istringstream csv("1, 2, 3, 0, 1, 1, 1\n");
  EXPECT_EQ(parseClippedAreasCsv(csv).size(), 1u);
}

TEST(ClippedArea, RejectsMalformedRows)
{
  std::istringstream too_short("x,y,z,yaw,length,width,height\n1, 2, 3\n");
  EXPECT_THROW(parseClippedAreasCsv(too_short), std::runtime_error);

  std::istringstream not_numeric("x,y,z,yaw,length,width,height\n1, 2, 3, 0, a, 1, 1\n");
  EXPECT_THROW(parseClippedAreasCsv(not_numeric), std::runtime_error);
}

TEST(ClippedArea, MissingFileThrows)
{
  EXPECT_THROW(loadClippedAreasFromCsv("/nonexistent/clipped_areas.csv"), std::runtime_error);
}
