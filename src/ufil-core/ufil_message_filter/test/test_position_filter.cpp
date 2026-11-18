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

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ufil_message_filter/position_filter.hpp"

using ufil_message_filter::FilterStatistics;
using ufil_message_filter::Position;
using ufil_message_filter::PositionFilter;
using ufil_message_filter::PositionFilterConfig;
using ufil_message_filter::validate;

namespace
{

/// The area of interest of the CPM lab examples, counter-clockwise.
PositionFilterConfig cpmLabArea()
{
  return PositionFilterConfig{{{81.0, 30.0}, {81.0, 46.0}, {0.0, 46.0}, {0.0, 30.0}}};
}

}  // namespace

TEST(PositionFilterTest, ContainsOnlyPositionsStrictlyInside)
{
  const PositionFilter filter(cpmLabArea());
  EXPECT_TRUE(filter.enabled());
  EXPECT_TRUE(filter.contains({40.0, 38.0}));
  EXPECT_TRUE(filter.contains({80.9, 30.1}));
  EXPECT_FALSE(filter.contains({100.0, 38.0}));
  EXPECT_FALSE(filter.contains({40.0, 29.9}));
  // The boundary is outside.
  EXPECT_FALSE(filter.contains({81.0, 38.0}));
  EXPECT_FALSE(filter.contains({0.0, 30.0}));
}

TEST(PositionFilterTest, OrientationDoesNotMatter)
{
  auto clockwise = cpmLabArea();
  std::reverse(clockwise.area_of_interest.begin(), clockwise.area_of_interest.end());
  const PositionFilter filter(clockwise);
  EXPECT_TRUE(filter.contains({40.0, 38.0}));
  EXPECT_FALSE(filter.contains({100.0, 38.0}));
}

TEST(PositionFilterTest, NonConvexArea)
{
  // L-shaped area without the square [5, 10] x [5, 10].
  const PositionFilter filter(
    PositionFilterConfig{{{0.0, 0.0}, {10.0, 0.0}, {10.0, 5.0}, {5.0, 5.0}, {5.0, 10.0},
      {0.0, 10.0}}});
  EXPECT_TRUE(filter.contains({2.0, 8.0}));
  EXPECT_TRUE(filter.contains({8.0, 2.0}));
  EXPECT_FALSE(filter.contains({8.0, 8.0}));
}

TEST(PositionFilterTest, EmptyAreaDisablesFilter)
{
  const PositionFilter filter(PositionFilterConfig{});
  EXPECT_FALSE(filter.enabled());
  EXPECT_TRUE(filter.contains({1e9, -1e9}));
}

TEST(PositionFilterTest, FilterReturnsPassingIndicesAndCounts)
{
  PositionFilter filter(cpmLabArea());
  const std::vector<Position> positions{{40.0, 38.0}, {100.0, 38.0}, {1.0, 31.0}};
  EXPECT_EQ(filter.filter(positions), (std::vector<std::size_t>{0, 2}));
  EXPECT_TRUE(filter.filter({{-1.0, -1.0}}).empty());
  EXPECT_TRUE(filter.filter({}).empty());

  const FilterStatistics & statistics = filter.statistics();
  EXPECT_EQ(statistics.lists, 3u);
  EXPECT_EQ(statistics.empty_lists, 2u);
  EXPECT_EQ(statistics.objects, 4u);
  EXPECT_EQ(statistics.passed_objects, 2u);
  EXPECT_EQ(statistics.rejected_objects, 2u);
}

TEST(PositionFilterTest, MovedFilterKeepsArea)
{
  PositionFilter filter(cpmLabArea());
  PositionFilter moved(std::move(filter));
  EXPECT_TRUE(moved.contains({40.0, 38.0}));
  EXPECT_FALSE(moved.contains({100.0, 38.0}));
}

TEST(PositionFilterConfigTest, Validation)
{
  EXPECT_FALSE(validate(PositionFilterConfig{}));
  EXPECT_FALSE(validate(cpmLabArea()));

  // Too few vertices.
  EXPECT_TRUE(validate(PositionFilterConfig{{{0.0, 0.0}, {1.0, 1.0}}}));
  // Not finite.
  EXPECT_TRUE(
    validate(
      PositionFilterConfig{{{0.0, 0.0}, {1.0, 0.0},
        {std::numeric_limits<double>::quiet_NaN(), 1.0}}}));
  // Self-intersecting (bow tie).
  EXPECT_TRUE(validate(PositionFilterConfig{{{0.0, 0.0}, {1.0, 1.0}, {1.0, 0.0}, {0.0, 1.0}}}));
  // Without area.
  EXPECT_TRUE(validate(PositionFilterConfig{{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}}));

  EXPECT_THROW(
    PositionFilter(PositionFilterConfig{{{0.0, 0.0}, {1.0, 1.0}}}), std::invalid_argument);
}
