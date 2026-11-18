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

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "ssl_wheel_tracker/wheel_detector.hpp"

using ssl_wheel_tracker::Cell;
using ssl_wheel_tracker::DetectorConfig;
using ssl_wheel_tracker::PressureGrid;
using ssl_wheel_tracker::WheelDetector;

namespace
{

constexpr double kPi = 3.14159265358979323846;

PressureGrid makeGrid(std::size_t columns, std::size_t rows, double resolution = 0.2)
{
  PressureGrid grid;
  grid.columns = columns;
  grid.rows = rows;
  grid.resolution = resolution;
  grid.values.assign(columns * rows, 0U);
  return grid;
}

/// Wheel contact: peak at the center, lower pressure on the 8 neighbours.
void addContact(PressureGrid & grid, std::size_t column, std::size_t row, std::uint8_t peak = 20)
{
  for (std::size_t c = column - 1; c <= column + 1; ++c) {
    for (std::size_t r = row - 1; r <= row + 1; ++r) {
      grid.at(c, r) = 12;
    }
  }
  grid.at(column, row) = peak;
}

}  // namespace

TEST(WheelDetector, EmptyGridHasNoWheels)
{
  const WheelDetector detector{DetectorConfig{}};
  EXPECT_TRUE(detector.detect(makeGrid(10, 8)).empty());
  EXPECT_TRUE(detector.detect(PressureGrid{}).empty());
}

TEST(WheelDetector, ConstantGridHasNoWheels)
{
  auto grid = makeGrid(10, 8);
  grid.values.assign(grid.values.size(), 50U);
  EXPECT_TRUE(WheelDetector(DetectorConfig{}).detect(grid).empty());
}

TEST(WheelDetector, SymmetricContactIsLocatedAtItsCenter)
{
  auto grid = makeGrid(10, 8);
  grid.origin = {1.0, 2.0, 0.0};
  addContact(grid, 4, 3);

  const auto wheels = WheelDetector(DetectorConfig{}).detect(grid);
  ASSERT_EQ(wheels.size(), 1u);
  EXPECT_NEAR(wheels[0].position.x, 1.0 + 4.5 * 0.2, 1e-9);
  EXPECT_NEAR(wheels[0].position.y, 2.0 + 3.5 * 0.2, 1e-9);
  // The contact radius 3 covers the whole blob, the background is 0.
  EXPECT_DOUBLE_EQ(wheels[0].pressure, 20.0 + 8 * 12.0);
  EXPECT_EQ(wheels[0].cells, 25u);
}

TEST(WheelDetector, CenterIsPressureWeighted)
{
  auto grid = makeGrid(10, 8, 1.0);
  grid.at(4, 3) = 30;
  grid.at(5, 3) = 10;

  const auto wheels = WheelDetector(DetectorConfig{}).detect(grid);
  ASSERT_EQ(wheels.size(), 1u);
  EXPECT_NEAR(wheels[0].position.x, (4.0 * 30 + 5.0 * 10) / 40.0 + 0.5, 1e-9);
  EXPECT_NEAR(wheels[0].position.y, 3.5, 1e-9);
}

TEST(WheelDetector, BackgroundIsSubtractedFromThePressure)
{
  auto grid = makeGrid(10, 8, 1.0);
  grid.values.assign(grid.values.size(), 2U);
  grid.at(4, 3) = 30;

  const auto wheels = WheelDetector(DetectorConfig{}).detect(grid);
  ASSERT_EQ(wheels.size(), 1u);
  EXPECT_EQ(wheels[0].cells, 25u);
  EXPECT_DOUBLE_EQ(wheels[0].pressure, 28.0);
}

TEST(WheelDetector, ThresholdIsExclusive)
{
  auto grid = makeGrid(10, 8);
  grid.at(4, 3) = 10;
  EXPECT_TRUE(WheelDetector(DetectorConfig{}).detect(grid).empty());
  grid.at(4, 3) = 11;
  EXPECT_EQ(WheelDetector(DetectorConfig{}).detect(grid).size(), 1u);
}

TEST(WheelDetector, MaximaInTheBorderAreIgnored)
{
  auto grid = makeGrid(10, 8);
  grid.at(0, 3) = 20;
  grid.at(9, 3) = 20;
  grid.at(4, 7) = 20;
  EXPECT_TRUE(WheelDetector(DetectorConfig{}).detect(grid).empty());

  DetectorConfig config;
  config.exclude_border = 0;
  EXPECT_EQ(WheelDetector(config).detect(grid).size(), 3u);
}

TEST(WheelDetector, PlateauIsOneWheel)
{
  auto grid = makeGrid(10, 8, 1.0);
  grid.at(4, 3) = 20;
  grid.at(5, 3) = 20;
  grid.at(5, 4) = 20;

  const auto wheels = WheelDetector(DetectorConfig{}).detect(grid);
  ASSERT_EQ(wheels.size(), 1u);
}

TEST(WheelDetector, CloseMaximaKeepTheStrongerOne)
{
  // Two maxima within the peak window: only the stronger one is a local maximum.
  auto grid = makeGrid(12, 8, 1.0);
  grid.at(4, 3) = 25;
  grid.at(6, 3) = 20;
  const auto peaks = ssl_wheel_tracker::findPeaks(grid, DetectorConfig{});
  ASSERT_EQ(peaks.size(), 1u);
  EXPECT_EQ(peaks[0], (Cell{4, 3}));
}

TEST(WheelDetector, WheelsAreOrderedColumnMajor)
{
  auto grid = makeGrid(20, 10, 1.0);
  addContact(grid, 14, 2, 30);
  addContact(grid, 4, 6, 20);
  addContact(grid, 4, 2, 25);

  const auto peaks = ssl_wheel_tracker::findPeaks(grid, DetectorConfig{});
  ASSERT_EQ(peaks.size(), 3u);
  EXPECT_EQ(peaks[0], (Cell{14, 2}));  // by decreasing pressure

  const auto wheels = WheelDetector(DetectorConfig{}).detect(grid);
  ASSERT_EQ(wheels.size(), 3u);
  EXPECT_NEAR(wheels[0].position.x, 4.5, 1e-9);
  EXPECT_NEAR(wheels[0].position.y, 2.5, 1e-9);
  EXPECT_NEAR(wheels[1].position.x, 4.5, 1e-9);
  EXPECT_NEAR(wheels[1].position.y, 6.5, 1e-9);
  EXPECT_NEAR(wheels[2].position.x, 14.5, 1e-9);
  EXPECT_NEAR(wheels[2].position.y, 2.5, 1e-9);
}

TEST(WheelDetector, CellsBetweenWheelsGoToTheNearestPeak)
{
  // Peaks 4 cells apart: each gets its neighbour towards the other one.
  auto grid = makeGrid(20, 10, 1.0);
  grid.at(7, 4) = 5;
  grid.at(9, 4) = 5;
  grid.at(6, 4) = 20;
  grid.at(10, 4) = 20;

  const auto wheels = WheelDetector(DetectorConfig{}).detect(grid);
  ASSERT_EQ(wheels.size(), 2u);
  EXPECT_LT(wheels[0].position.x, 8.5);
  EXPECT_GT(wheels[1].position.x, 8.5);
  EXPECT_NEAR(wheels[0].position.x + wheels[1].position.x, 2 * 8.5, 1e-9);
}

TEST(WheelDetector, RotatedOrigin)
{
  auto grid = makeGrid(10, 8);
  grid.origin = {1.0, 2.0, kPi / 2.0};
  addContact(grid, 4, 3);

  const auto wheels = WheelDetector(DetectorConfig{}).detect(grid);
  ASSERT_EQ(wheels.size(), 1u);
  // The column axis points along map y, the row axis along map -x.
  EXPECT_NEAR(wheels[0].position.x, 1.0 - 3.5 * 0.2, 1e-9);
  EXPECT_NEAR(wheels[0].position.y, 2.0 + 4.5 * 0.2, 1e-9);
}

TEST(WheelDetector, InvalidConfigurationThrows)
{
  DetectorConfig config;
  config.threshold = 255;
  EXPECT_THROW(WheelDetector{config}, std::invalid_argument);
  config = DetectorConfig{};
  config.min_peak_distance = 0;
  EXPECT_THROW(WheelDetector{config}, std::invalid_argument);
  config = DetectorConfig{};
  config.contact_radius = 0;
  EXPECT_THROW(WheelDetector{config}, std::invalid_argument);
}
