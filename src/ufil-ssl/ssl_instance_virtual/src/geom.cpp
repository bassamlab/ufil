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


/**
 * @file geom.hpp
 * @author Marius Molz (marius.molz@rwth-aachen.de)
 * @brief Custom compile-time geometry library
 * @version 1.0
 * @date 2024-01-12
 *
 */
#include "geom.hpp"

[[nodiscard, gnu::pure]] auto
rasterizeGeometry(
  cppdriver::dimension_t const & size_x,
  cppdriver::dimension_t const & size_y,
  ring_xy const & geometry,
  float const & grid_resolution)
-> std::vector<std::pair<cppdriver::dimension_t, cppdriver::dimension_t>>
{
  std::vector<std::pair<cppdriver::dimension_t, cppdriver::dimension_t>> points{};

  auto const &[p0, p1, p2, p3] = geometry;
  auto const [min_x, max_x] = std::minmax({p0.first, p1.first, p2.first, p3.first});
  auto const [min_y, max_y] = std::minmax({p0.second, p1.second, p2.second, p3.second});

  if ((min_x < 0. && max_x < 0.) || (min_y < 0. && max_y < 0.)) {
    return {};
  }

  size_t const min_x_ = fmax(floor((min_x / grid_resolution)), 0.);
  size_t const min_y_ = fmax(floor((min_y / grid_resolution)), 0.);
  size_t const max_x_ = fmin(floor((max_x / grid_resolution)), size_x);
  size_t const max_y_ = fmin(floor((max_y / grid_resolution)), size_y);

    // Now the points are only limited to valid on-mat points
    // Ray cast over x-axis @ https://en.wikipedia.org/wiki/Even%E2%80%93odd_rule
  for (cppdriver::dimension_t x(min_x_); x < max_x_; x++) {
        // center of grid cell
    auto const x_ = (x + .5f) * grid_resolution;
    for (cppdriver::dimension_t y(min_y_); y < max_y_; y++) {
      auto const y_ = (y + .5f) * grid_resolution;
      if (pointInRing({x_, y_}, geometry)) {
        points.emplace_back(x, y);
      }
    }
  }

  return points;
}
