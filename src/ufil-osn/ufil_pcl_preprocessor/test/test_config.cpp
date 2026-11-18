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

#include "ufil_pcl_preprocessor/config.hpp"

using ufil_pcl_preprocessor::PreprocessorConfig;
using ufil_pcl_preprocessor::validate;

TEST(Config, DefaultsAreValid)
{
  EXPECT_EQ(validate(PreprocessorConfig{}), std::nullopt);
}

TEST(Config, RejectsInvalidValues)
{
  const std::vector<std::function<void(PreprocessorConfig &)>> mutations = {
    [](PreprocessorConfig & c) {c.filter.voxel_size = 0.0;},
    [](PreprocessorConfig & c) {c.filter.outlier_radius = -1.0;},
    [](PreprocessorConfig & c) {c.filter.min_neighbors = -1;},
    [](PreprocessorConfig & c) {c.filter.z_min = c.filter.z_max;},
    [](PreprocessorConfig & c) {c.filter.z_max = std::numeric_limits<double>::quiet_NaN();},
    [](PreprocessorConfig & c) {c.fov.num_bins = 0;},
    [](PreprocessorConfig & c) {c.fov.history_length = 0;},
    [](PreprocessorConfig & c) {c.fov.max_range = 0.0;},
    [](PreprocessorConfig & c) {c.occlusion_grid.min_range = -0.1;},
    [](PreprocessorConfig & c) {c.occlusion_grid.max_range = c.occlusion_grid.min_range;},
    [](PreprocessorConfig & c) {c.occlusion_grid.resolution = 0.0;},
    [](PreprocessorConfig & c) {c.occlusion_grid.size_x = 0.1;},
    [](PreprocessorConfig & c) {c.occlusion_grid.decay_rate = 1.5;},
  };
  for (std::size_t i = 0; i < mutations.size(); ++i) {
    PreprocessorConfig config;
    mutations[i](config);
    EXPECT_NE(validate(config), std::nullopt) << "mutation " << i;
  }
}

TEST(Config, AcceptsNegativeHeightRange)
{
  PreprocessorConfig config;
  config.filter.z_min = -1.0;
  config.filter.z_max = -0.5;
  EXPECT_EQ(validate(config), std::nullopt);
}
