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

#include "ufil_message_filter/position_filter.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/geometry.hpp>

namespace ufil_message_filter
{

namespace
{

namespace bg = boost::geometry;

using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

/// Builds a closed polygon with the orientation boost::geometry expects.
Polygon toPolygon(const std::vector<Position> & vertices)
{
  Polygon polygon;
  for (const auto & vertex : vertices) {
    bg::append(polygon.outer(), Point(vertex.x, vertex.y));
  }
  bg::correct(polygon);
  return polygon;
}

}  // namespace

struct PositionFilter::Area
{
  Polygon polygon;
};

std::optional<std::string> validate(const PositionFilterConfig & config)
{
  const auto & vertices = config.area_of_interest;
  if (vertices.empty()) {
    return std::nullopt;
  }
  if (vertices.size() < 3) {
    return "the area of interest needs at least 3 vertices, got " +
           std::to_string(vertices.size());
  }
  for (const auto & vertex : vertices) {
    if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y)) {
      return "the area of interest has a vertex that is not finite";
    }
  }
  std::string reason;
  if (!bg::is_valid(toPolygon(vertices), reason)) {
    return "the area of interest is not a simple polygon: " + reason;
  }
  return std::nullopt;
}

PositionFilter::PositionFilter(PositionFilterConfig config)
: config_(std::move(config))
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument(*error);
  }
  if (!config_.area_of_interest.empty()) {
    area_ = std::make_unique<Area>(Area{toPolygon(config_.area_of_interest)});
  }
}

PositionFilter::~PositionFilter() = default;
PositionFilter::PositionFilter(PositionFilter &&) noexcept = default;
PositionFilter & PositionFilter::operator=(PositionFilter &&) noexcept = default;

bool PositionFilter::enabled() const
{
  return area_ != nullptr;
}

bool PositionFilter::contains(const Position & position) const
{
  return !area_ || bg::within(Point(position.x, position.y), area_->polygon);
}

std::vector<std::size_t> PositionFilter::filter(const std::vector<Position> & positions)
{
  std::vector<std::size_t> passed;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (contains(positions[i])) {
      passed.push_back(i);
    }
  }

  ++statistics_.lists;
  if (passed.empty()) {
    ++statistics_.empty_lists;
  }
  statistics_.objects += positions.size();
  statistics_.passed_objects += passed.size();
  statistics_.rejected_objects += positions.size() - passed.size();
  return passed;
}

const PositionFilterConfig & PositionFilter::config() const
{
  return config_;
}

const FilterStatistics & PositionFilter::statistics() const
{
  return statistics_;
}

}  // namespace ufil_message_filter
