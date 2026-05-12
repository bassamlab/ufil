// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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


#ifndef POSITION_FILTER_HPP_
#define POSITION_FILTER_HPP_

#include <vector>
#include <boost/geometry.hpp>

#include "boolean_filter.hpp"

namespace bg = boost::geometry;

template<class M, typename T>
class PositionFilter : public BooleanFilter<M>
{
public:
  using Point = bg::model::d2::point_xy<T>;

  using Polygon = bg::model::polygon<Point>;

protected:
  Polygon polygon_;

  virtual void toPoint(const M & message, Point & point) = 0;

public:
  explicit PositionFilter(std::vector<Point> && polygon_points)
  {
    bg::assign_points(this->polygon_, polygon_points);
  }

  bool filter(const M & message) override
  {
    Point point;
    this->toPoint(message, point);

    return bg::within(point, this->polygon_);
  }
};

#endif  // POSITION_FILTER_HPP_
