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

#include "ufil_obu_object_aggregation/config.hpp"

using ufil_obu_object_aggregation::AggregatorConfig;
using ufil_obu_object_aggregation::validate;

TEST(Config, DefaultsAreValid)
{
  EXPECT_EQ(validate(AggregatorConfig{}), std::nullopt);
}

TEST(Config, RejectsInvalidValues)
{
  const std::vector<std::function<void(AggregatorConfig &)>> mutations = {
    [](AggregatorConfig & c) {c.object_length = 0.0;},
    [](AggregatorConfig & c) {c.object_width = -1.0;},
    [](AggregatorConfig & c) {c.object_width = std::numeric_limits<double>::quiet_NaN();},
    [](AggregatorConfig & c) {c.object_class = -1;},
    [](AggregatorConfig & c) {c.object_class = 7;},
    [](AggregatorConfig & c) {c.utm_zone = 0;},
    [](AggregatorConfig & c) {c.utm_zone = 61;},
  };
  for (std::size_t i = 0; i < mutations.size(); ++i) {
    AggregatorConfig config;
    mutations[i](config);
    EXPECT_NE(validate(config), std::nullopt) << "mutation " << i;
  }
}

TEST(Config, AcceptsForcedZone)
{
  AggregatorConfig config;
  config.utm_zone = 32;
  EXPECT_EQ(validate(config), std::nullopt);
}
