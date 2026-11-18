// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWHT Aachen University
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
#ifndef UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__INTERSECTION_OVER_UNION_FUNCTIONS_HPP_
#define UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__INTERSECTION_OVER_UNION_FUNCTIONS_HPP_

#include <vector>
#include <cmath>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "ufil_object_tracking/types/scalar.hpp"

namespace bg = boost::geometry;


inline bg::model::d2::point_xy<ufil::type::Scalar> rotate_point(
  const bg::model::d2::point_xy<ufil::type::Scalar> & p,
  const bg::model::d2::point_xy<ufil::type::Scalar> & center, ufil::type::Scalar angle_rad)
{
  ufil::type::Scalar cos_p = std::cos(angle_rad);
  ufil::type::Scalar sin_p = std::sin(angle_rad);
  ufil::type::Scalar x = bg::get<0>(p) - bg::get<0>(center);
  ufil::type::Scalar y = bg::get<1>(p) - bg::get<1>(center);

  //  rotation matrix
  ufil::type::Scalar x_rot = cos_p * x - sin_p * y + bg::get<0>(center);
  ufil::type::Scalar y_rot = sin_p * x + cos_p * y + bg::get<1>(center);

  return bg::model::d2::point_xy<ufil::type::Scalar>(x_rot, y_rot);
}

inline bg::model::polygon<bg::model::d2::point_xy<ufil::type::Scalar>> create_rotated_rectangle(
  ufil::type::Scalar x, ufil::type::Scalar y, ufil::type::Scalar length,
  ufil::type::Scalar width, ufil::type::Scalar angle_rad)
{
  std::vector<bg::model::d2::point_xy<ufil::type::Scalar>> corners;
  corners.reserve(5);  // reserve space to avoid reallocations
  ufil::type::Scalar half_length = length / 2.0;
  ufil::type::Scalar half_width = width / 2.0;
  // create vector
  std::vector<bg::model::d2::point_xy<ufil::type::Scalar>> position = {
    bg::model::d2::point_xy<ufil::type::Scalar>(x - half_length, y - half_width),  // topLeft
    bg::model::d2::point_xy<ufil::type::Scalar>(x + half_length, y - half_width),  // topRight
    bg::model::d2::point_xy<ufil::type::Scalar>(x + half_length, y + half_width),  // bottomRight
    bg::model::d2::point_xy<ufil::type::Scalar>(x - half_length, y + half_width)  // bottomLeft
  };
  // rotate
  for(const auto & p : position) {
    corners.emplace_back(rotate_point(p, bg::model::d2::point_xy<ufil::type::Scalar>(x, y),
                                      angle_rad));
  }
  corners.emplace_back(corners[0]);  // otherwise polygon would be open.
  bg::model::polygon<bg::model::d2::point_xy<ufil::type::Scalar>> poly;
  bg::assign_points(poly, corners);
  bg::correct(poly);  // Ensure polygon is valid & orientation is correct
  return poly;
}

#endif  // UFIL_OBJECT_TRACKING__ASSOCIATION__IMPL__INTERSECTION_OVER_UNION_FUNCTIONS_HPP_
