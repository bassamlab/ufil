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

#include "ufil_central_fusion/config.hpp"

using ufil_central_fusion::FusionConfig;
using ufil_central_fusion::SensorConfig;
using ufil_central_fusion::validate;

TEST(Config, DefaultsAreValid)
{
  EXPECT_EQ(validate(FusionConfig{}), std::nullopt);
  EXPECT_EQ(validate(std::vector<SensorConfig>{SensorConfig{"osn", false, {}}}), std::nullopt);
}

TEST(Config, RejectsInvalidFusionValues)
{
  const std::vector<std::function<void(FusionConfig &)>> mutations = {
    [](FusionConfig & c) {c.association_threshold = 0.0;},
    [](FusionConfig & c) {c.existence_min_weight = 0.9;},
    [](FusionConfig & c) {c.existence_max_weight = 1.1;},
    [](FusionConfig & c) {c.existence_decay_factor = -0.1;},
    [](FusionConfig & c) {c.dimension_grid_resolution = 0.0;},
    [](FusionConfig & c) {c.dimension_alpha = 0.0;},
    [](FusionConfig & c) {c.dimension_p_min = 0.6;},
    [](FusionConfig & c) {c.dimension_p_max = 0.4;},
    [](FusionConfig & c) {c.buffer_duration = 0.0;},
    [](FusionConfig & c) {c.time_jump_threshold = std::numeric_limits<double>::infinity();},
  };
  for (std::size_t i = 0; i < mutations.size(); ++i) {
    FusionConfig config;
    mutations[i](config);
    EXPECT_NE(validate(config), std::nullopt) << "mutation " << i;
  }
}

TEST(Config, RejectsInvalidSensors)
{
  EXPECT_NE(validate(std::vector<SensorConfig>{}), std::nullopt);
  EXPECT_NE(validate(std::vector<SensorConfig>{SensorConfig{"", false, {}}}), std::nullopt);
  EXPECT_NE(
    validate(std::vector<SensorConfig>{SensorConfig{"osn", false, {}},
      SensorConfig{"osn", false, {}}}), std::nullopt);

  SensorConfig untrusted{"osn", false, {}};
  untrusted.trust.pedestrian = 1.5;
  EXPECT_NE(validate(std::vector<SensorConfig>{untrusted}), std::nullopt);
}
