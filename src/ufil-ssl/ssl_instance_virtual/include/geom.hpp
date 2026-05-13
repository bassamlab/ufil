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
#pragma once

#include <algorithm>
#include <cmath>
#include <tuple>
#include <utility>
#include <vector>
#include <gcem.hpp>

#include "driver_interface.hpp"

/// @brief Internal representation of two-dimensional points
using p_xy = std::pair<float, float>;
/// @brief Linepoints for ring
using ring_xy = std::tuple<p_xy, p_xy, p_xy, p_xy>;
using vector_xy = std::vector<p_xy>;

/// @brief Reinterpret tuple as array, source: https://stackoverflow.com/a/54487034
template<typename tuple_t>
constexpr auto
get_array_from_tuple(tuple_t && tuple)
{
  constexpr auto get_array = [](auto &&... x) {
      return std::array{std::forward<decltype(x)>(x)...};
    };
  return std::apply(get_array, std::forward<tuple_t>(tuple));
}

/// @brief Templated 3x3 matrix, to be used for affine transformations
template<typename T>
using matrix_3x3 = std::array<std::array<T, 3>, 3>;

/// @brief Overload subtraction
template<typename T>
constexpr auto
operator-(std::pair<T, T> const & lhs, std::pair<T, T> const & rhs) -> std::pair<T, T>
{
  return {lhs.first - rhs.first, lhs.second - rhs.second};
}

/// @brief Overload addition
template<typename T>
constexpr auto
operator+(std::pair<T, T> const & lhs, std::pair<T, T> const & rhs) -> std::pair<T, T>
{
  return {lhs.first + rhs.first, lhs.second + rhs.second};
}

/// @brief Overload scalar - vector multiplication
template<typename T>
constexpr auto
operator*(T const & lhs, std::pair<T, T> const & rhs) -> std::pair<T, T>
{
  return {lhs * rhs.first, lhs * rhs.second};
}

/// @brief Overload matrix - vector multiplication
template<typename T>
constexpr auto
operator*(matrix_3x3<T> const & matrix, std::pair<T, T> const & point) -> std::pair<T, T>
{
  return {
    matrix[0][0] * std::get<0>(point) + matrix[0][1] * std::get<1>(point) + matrix[0][2] * 1.f,
    matrix[1][0] * std::get<0>(point) + matrix[1][1] * std::get<1>(point) + matrix[1][2] * 1.f,
  };
}

/// @brief Overload matrix - ring_xy multiplication, used to transform ring
constexpr auto
operator*(matrix_3x3<float> const & matrix, ring_xy const & points) -> ring_xy
{
  return {
    matrix * std::get<0>(points),
    matrix * std::get<1>(points),
    matrix * std::get<2>(points),
    matrix * std::get<3>(points),
  };
}

template<typename T>
auto operator*(matrix_3x3<T> const & matrix, vector_xy const & points) -> vector_xy
{
  vector_xy transformed_points(points.size());
  std::transform(points.begin(), points.end(), transformed_points.begin(),
    [&matrix](const auto & point) {
      return matrix * point;
                   });
  return transformed_points;
}

/// @brief Overload scalar - ring_xy multiplication, used to scale ring
constexpr auto
operator*(float const scale, ring_xy const & points) -> ring_xy
{
  return {
    scale * std::get<0>(points),
    scale * std::get<1>(points),
    scale * std::get<2>(points),
    scale * std::get<3>(points),
  };
}

/// @brief Overload matrix - matrix multiplication
template<typename T>
constexpr auto
operator*(matrix_3x3<T> const & lhs, matrix_3x3<T> const & rhs) -> matrix_3x3<T>
{
  return {{
    {lhs[0][0] * rhs[0][0] + lhs[0][1] * rhs[1][0] + lhs[0][2] * rhs[2][0],
      lhs[0][0] * rhs[0][1] + lhs[0][1] * rhs[1][1] + lhs[0][2] * rhs[2][1],
      lhs[0][0] * rhs[0][2] + lhs[0][1] * rhs[1][2] + lhs[0][2] * rhs[2][2]},
    {lhs[1][0] * rhs[0][0] + lhs[1][1] * rhs[1][0] + lhs[1][2] * rhs[2][0],
      lhs[1][0] * rhs[0][1] + lhs[1][1] * rhs[1][1] + lhs[1][2] * rhs[2][1],
      lhs[1][0] * rhs[0][2] + lhs[1][1] * rhs[1][2] + lhs[1][2] * rhs[2][2]},
    {lhs[2][0] * rhs[0][0] + lhs[2][1] * rhs[1][0] + lhs[2][2] * rhs[2][0],
      lhs[2][0] * rhs[0][1] + lhs[2][1] * rhs[1][1] + lhs[2][2] * rhs[2][1],
      lhs[2][0] * rhs[0][2] + lhs[2][1] * rhs[1][2] + lhs[2][2] * rhs[2][2]},
  }};
}

/// @brief create affine translation matrix
constexpr auto
TRANSLATION_MATRIX(p_xy const & translation) -> matrix_3x3<float>
{
  return {{{1.f, 0.f, std::get<0>(translation)},
    {0.f, 1.f, std::get<1>(translation)},
    {0.f, 0.f, 1.f}}};
}

/// @brief create affine rotation matrix
constexpr auto
ROTATION_MATRIX(float const radian_ccw) -> matrix_3x3<float>
{
  return {{{gcem::cos(radian_ccw), -gcem::sin(radian_ccw), 0.f},
    {gcem::sin(radian_ccw), gcem::cos(radian_ccw), 0.f},
    {0.f, 0.f, 1.f}}};
}


/// @brief check if point lies inside the closed geometry
constexpr auto
pointInRing(p_xy const & point, ring_xy const & ring) -> bool
{
    // Adapted, see: https://en.wikipedia.org/wiki/Even%E2%80%93odd_rule
  auto const & poly = get_array_from_tuple(ring);
  auto const &[x, y] = point;
  uint8_t j = 3U;
  bool c = false;
  for (uint8_t i(0); i < 4U; i++) {
    if ((x == poly[i].first) && (y == poly[i].second)) {
      return true;
    } else if ((poly[i].second > y) != (poly[j].second > y)) {
      auto const slope = (x - poly[i].first) * (poly[j].second - poly[i].second) -
        (poly[j].first - poly[i].first) * (y - poly[i].second);
      if (FP_ZERO == std::fpclassify(slope)) {
        return true;
      } else if ((slope < 0) != (poly[j].second < poly[i].second)) {
        c = !c;
      }
    }
    j = i;
  }
  return c;
}

/// @brief rasterize continous geometry to set of discrete points
[[nodiscard, gnu::pure]] auto
rasterizeGeometry(
  cppdriver::dimension_t const & size_x,
  cppdriver::dimension_t const & size_y,
  ring_xy const & geometry,
  float const & grid_resolution)
-> std::vector<std::pair<cppdriver::dimension_t, cppdriver::dimension_t>>;


struct Axle
{
  bool single_track;
  float track_width;
  float center_to_axle;
};

using Axles = std::vector<Axle>;
