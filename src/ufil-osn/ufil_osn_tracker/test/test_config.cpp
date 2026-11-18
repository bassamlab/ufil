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

#include <functional>
#include <limits>
#include <vector>

#include "ufil_osn_tracker/config.hpp"

using ufil_osn_tracker::TrackerConfig;
using ufil_osn_tracker::validate;

TEST(Config, DefaultsAreValid)
{
  EXPECT_EQ(validate(TrackerConfig{}), std::nullopt);
}

TEST(Config, RejectsInvalidValues)
{
  const std::vector<std::function<void(TrackerConfig &)>> mutations = {
    [](TrackerConfig & c) {c.association_threshold = 0.0;},
    [](TrackerConfig & c) {c.history_duration = -1.0;},
    [](TrackerConfig & c) {c.cluster_min_points = 0;},
    [](TrackerConfig & c) {c.cluster_max_points = c.cluster_min_points - 1;},
    [](TrackerConfig & c) {c.cluster_tolerance = 0.0;},
    [](TrackerConfig & c) {c.dimension_covariance_multiplier = 0.0;},
    [](TrackerConfig & c) {c.position_covariance_multiplier = -1.0;},
    [](TrackerConfig & c) {c.orientation_covariance_multiplier = 0.0;},
    [](TrackerConfig & c) {c.time_jump_threshold = 0.0;},
    [](TrackerConfig & c) {c.association_threshold = std::numeric_limits<double>::infinity();},
  };
  for (std::size_t i = 0; i < mutations.size(); ++i) {
    TrackerConfig config;
    mutations[i](config);
    EXPECT_NE(validate(config), std::nullopt) << "mutation " << i;
  }
}
