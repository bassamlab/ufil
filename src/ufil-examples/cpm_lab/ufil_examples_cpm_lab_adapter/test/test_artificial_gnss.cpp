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

#include <Eigen/Core>
#include <numbers>  // listed with the C headers, cpplint does not know this C++20 header

#include <stdexcept>

#include "ufil_examples_cpm_lab_adapter/artificial_gnss.hpp"

using ufil_examples_cpm_lab_adapter::ArtificialGnss;
using ufil_examples_cpm_lab_adapter::ArtificialGnssConfig;

TEST(ArtificialGnssTest, WithoutErrorsReturnsTheReceiverPosition)
{
  ArtificialGnssConfig config;
  config.clock_error = 0.0;
  config.ionospheric_error = 0.0;
  config.ephemeris_error = 0.0;
  ArtificialGnss gnss(config, 1);

  // The receiver is 7.5 cm behind the vehicle centre.
  const auto east = gnss.generateFix({2.0, 3.0, 0.4}, 0.0);
  EXPECT_NEAR(east.x(), 1.925, 1e-3);
  EXPECT_NEAR(east.y(), 3.0, 1e-3);
  EXPECT_DOUBLE_EQ(east.z(), 0.0);

  const auto north = gnss.generateFix({2.0, 3.0, 0.0}, std::numbers::pi / 2.0);
  EXPECT_NEAR(north.x(), 2.0, 1e-3);
  EXPECT_NEAR(north.y(), 2.925, 1e-3);
}

TEST(ArtificialGnssTest, ErrorsAreBoundedAndReproducible)
{
  ArtificialGnss first({}, 42);
  ArtificialGnss second({}, 42);
  for (int i = 0; i < 100; ++i) {
    const auto fix = first.generateFix({1.0, 1.0, 0.0}, 0.0);
    EXPECT_EQ(fix, second.generateFix({1.0, 1.0, 0.0}, 0.0));
    // Real-world errors of a few metres are a few decimetres in the lab.
    EXPECT_LT((fix - Eigen::Vector3d(0.925, 1.0, 0.0)).norm(), 2.0);
  }
}

TEST(ArtificialGnssTest, RejectsInvalidConfiguration)
{
  ArtificialGnssConfig config;
  config.number_of_frequency_bands = 0;
  EXPECT_THROW(ArtificialGnss{config}, std::invalid_argument);
  config = {};
  config.clock_error = -1.0;
  EXPECT_THROW(ArtificialGnss{config}, std::invalid_argument);
}
