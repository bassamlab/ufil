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

#include "ufil_visualization/lanelet_map_markers.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "earcut.hpp"

namespace ufil_visualization
{

namespace
{

using Ring = std::vector<std::array<double, 2>>;

Marker makeMarker(
  const std::string & ns, std::int32_t id, MarkerType type, const Color & color, double width)
{
  Marker marker;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.color = color;
  if (type != MarkerType::kTriangleList) {
    marker.scale = {width, width, width};
  }
  marker.frame_locked = true;
  return marker;
}

void addTriangles(Marker & marker, const Ring & ring, double z)
{
  if (ring.size() < 3) {
    return;
  }
  const auto indices = mapbox::earcut<std::uint32_t>(std::vector<Ring>{ring});
  for (const auto index : indices) {
    marker.points.push_back({ring[index][0], ring[index][1], z});
  }
}

void addSegments(Marker & marker, const lanelet::ConstLineString3d & line, double z)
{
  for (std::size_t i = 0; i + 1 < line.size(); ++i) {
    marker.points.push_back({line[i].x(), line[i].y(), z});
    marker.points.push_back({line[i + 1].x(), line[i + 1].y(), z});
  }
}

double length2d(const lanelet::ConstLineString3d & line)
{
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < line.size(); ++i) {
    length += std::hypot(line[i + 1].x() - line[i].x(), line[i + 1].y() - line[i].y());
  }
  return length;
}

std::array<double, 2> pointAtDistance(const lanelet::ConstLineString3d & line, double s)
{
  for (std::size_t i = 0; i + 1 < line.size(); ++i) {
    const double dx = line[i + 1].x() - line[i].x();
    const double dy = line[i + 1].y() - line[i].y();
    const double segment = std::hypot(dx, dy);
    if (s <= segment || i + 2 == line.size()) {
      const double t = segment > 0.0 ? std::clamp(s / segment, 0.0, 1.0) : 0.0;
      return {line[i].x() + t * dx, line[i].y() + t * dy};
    }
    s -= segment;
  }
  return {line.front().x(), line.front().y()};
}

/// Zebra stripes run between both bounds and repeat along the walking direction.
void addCrosswalkStripes(
  Marker & marker, const lanelet::ConstLanelet & lanelet, const LaneletMapStyle & style)
{
  const auto left = lanelet.leftBound();
  const auto right = lanelet.rightBound();
  const double left_length = length2d(left);
  const double right_length = length2d(right);
  const double length = (left_length + right_length) / 2.0;
  const double width = style.crosswalk_stripe_width;
  const double gap = style.crosswalk_stripe_gap;
  if (length < width) {
    return;
  }

  const auto count = static_cast<int>(std::floor((length + gap) / (width + gap)));
  const double offset = (length - (count * width + (count - 1) * gap)) / 2.0;
  for (int i = 0; i < count; ++i) {
    const double f0 = (offset + i * (width + gap)) / length;
    const double f1 = (offset + i * (width + gap) + width) / length;
    addTriangles(
      marker, {pointAtDistance(left, f0 * left_length), pointAtDistance(left, f1 * left_length),
        pointAtDistance(right, f1 * right_length), pointAtDistance(right, f0 * right_length)},
      style.crosswalk_stripe_z);
  }
}

bool isCrosswalk(const lanelet::ConstLanelet & lanelet)
{
  return lanelet.attributeOr<std::string>("subtype", "") == "crosswalk";
}

}  // namespace

std::optional<std::string> validate(const LaneletMapStyle & style)
{
  const std::array<std::pair<const Color *, const char *>, 14> colors{{
    {&style.lanelet_fill_color, "lanelet fill color"},
    {&style.lanelet_border_color, "lanelet border color"},
    {&style.lanelet_separator_color, "lanelet separator color"},
    {&style.lanelet_id_color, "lanelet id color"},
    {&style.road_marking_color, "road marking color"},
    {&style.crosswalk_fill_color, "crosswalk fill color"},
    {&style.crosswalk_stripe_color, "crosswalk stripe color"},
    {&style.area_default_color, "default area color"},
    {&style.area_walkway_color, "walkway area color"},
    {&style.area_vegetation_color, "vegetation area color"},
    {&style.area_parking_color, "parking area color"},
    {&style.area_keepout_color, "keepout area color"},
    {&style.area_building_color, "building area color"},
    {&style.area_traffic_island_color, "traffic island area color"},
  }};
  for (const auto & [color, name] : colors) {
    if (const auto error = validate(*color)) {
      return std::string(name) + ": " + *error;
    }
  }
  const std::array<std::pair<double, const char *>, 4> sizes{{
    {style.line_width, "line width"},
    {style.lanelet_id_text_size, "lanelet id text size"},
    {style.stop_line_width, "stop line width"},
    {style.crosswalk_stripe_width, "crosswalk stripe width"},
  }};
  for (const auto & [value, name] : sizes) {
    if (!(value > 0.0)) {
      return std::string(name) + " must be > 0";
    }
  }
  if (!(style.crosswalk_stripe_gap >= 0.0)) {
    return "crosswalk stripe gap must be >= 0";
  }
  return std::nullopt;
}

Color areaColor(const std::string & subtype, const LaneletMapStyle & style)
{
  static const std::map<std::string, Color LaneletMapStyle::*> colors{
    {"walkway", &LaneletMapStyle::area_walkway_color},
    {"vegetation", &LaneletMapStyle::area_vegetation_color},
    {"parking", &LaneletMapStyle::area_parking_color},
    {"keepout", &LaneletMapStyle::area_keepout_color},
    {"building", &LaneletMapStyle::area_building_color},
    {"traffic_island", &LaneletMapStyle::area_traffic_island_color},
  };
  const auto it = colors.find(subtype);
  return it == colors.end() ? style.area_default_color : style.*(it->second);
}

std::vector<Marker> laneletMapMarkers(
  const lanelet::LaneletMap & map, const LaneletMapStyle & style)
{
  std::vector<Marker> markers;
  std::int32_t id = 0;
  auto push = [&markers](Marker marker) {
      if (!marker.points.empty()) {
        markers.push_back(std::move(marker));
      }
    };

  // Lanelet and crosswalk fills.
  auto fill = makeMarker(
    "lanelet_fill", id++, MarkerType::kTriangleList, style.lanelet_fill_color, 1.0);
  auto crosswalk_fill = makeMarker(
    "crosswalk_fill", id++, MarkerType::kTriangleList, style.crosswalk_fill_color, 1.0);
  auto crosswalk_stripes = makeMarker(
    "crosswalk_stripes", id++, MarkerType::kTriangleList, style.crosswalk_stripe_color, 1.0);
  for (const auto & lanelet : map.laneletLayer) {
    Ring ring;
    for (const auto & point : lanelet.leftBound()) {
      ring.push_back({point.x(), point.y()});
    }
    for (const auto & point : lanelet.rightBound().invert()) {
      ring.push_back({point.x(), point.y()});
    }
    if (isCrosswalk(lanelet)) {
      addTriangles(crosswalk_fill, ring, style.crosswalk_fill_z);
      addCrosswalkStripes(crosswalk_stripes, lanelet, style);
    } else {
      addTriangles(fill, ring, style.lanelet_fill_z);
    }
  }
  push(std::move(fill));
  push(std::move(crosswalk_fill));
  push(std::move(crosswalk_stripes));

  // Lanelet borders.
  auto border = makeMarker(
    "lanelet_border", id++, MarkerType::kLineList, style.lanelet_border_color, style.line_width);
  for (const auto & lanelet : map.laneletLayer) {
    addSegments(border, lanelet.leftBound(), style.lanelet_border_z);
    addSegments(border, lanelet.rightBound(), style.lanelet_border_z);
  }
  push(std::move(border));

  // Separators at shared edges where one lanelet ends and its successor starts.
  std::set<std::pair<lanelet::Id, lanelet::Id>> end_edges;
  for (const auto & lanelet : map.laneletLayer) {
    if (!isCrosswalk(lanelet)) {
      end_edges.emplace(lanelet.leftBound().back().id(), lanelet.rightBound().back().id());
    }
  }
  auto separator = makeMarker(
    "lanelet_separator", id++, MarkerType::kLineList, style.lanelet_separator_color,
    style.line_width);
  for (const auto & lanelet : map.laneletLayer) {
    const auto left = lanelet.leftBound().front();
    const auto right = lanelet.rightBound().front();
    if (!isCrosswalk(lanelet) && end_edges.erase({left.id(), right.id()}) > 0) {
      separator.points.push_back({left.x(), left.y(), style.lanelet_separator_z});
      separator.points.push_back({right.x(), right.y(), style.lanelet_separator_z});
    }
  }
  push(std::move(separator));

  // Road markings (solid line_thin / line_thick only) and stop lines.
  auto marking = makeMarker(
    "road_marking", id++, MarkerType::kLineList, style.road_marking_color, style.line_width);
  auto stop_line = makeMarker(
    "stop_line", id++, MarkerType::kLineList, style.road_marking_color, style.stop_line_width);
  for (const auto & line : map.lineStringLayer) {
    const auto type = line.attributeOr<std::string>("type", "");
    if ((type == "line_thin" || type == "line_thick") &&
      line.attributeOr<std::string>("subtype", "") == "solid")
    {
      addSegments(marking, line, style.road_marking_z);
    } else if (type == "stop_line") {
      addSegments(stop_line, line, style.road_marking_z);
    }
  }
  push(std::move(marking));
  push(std::move(stop_line));

  // Lanelet ids at the centreline midpoint.
  if (style.show_lanelet_ids) {
    for (const auto & lanelet : map.laneletLayer) {
      const auto centerline = lanelet.centerline();
      const auto mid = pointAtDistance(centerline, length2d(centerline) / 2.0);
      auto text = makeMarker(
        "lanelet_id", id++, MarkerType::kTextViewFacing, style.lanelet_id_color,
        style.lanelet_id_text_size);
      text.position = {mid[0], mid[1], style.lanelet_id_z};
      text.text = std::to_string(lanelet.id());
      markers.push_back(std::move(text));
    }
  }

  // Area fills, one marker per subtype.
  std::map<std::string, Marker> areas;
  for (const auto & area : map.areaLayer) {
    const auto subtype = area.attributeOr<std::string>("subtype", "default");
    auto it = areas.find(subtype);
    if (it == areas.end()) {
      it = areas.emplace(
        subtype, makeMarker(
          "area_" + subtype, id++, MarkerType::kTriangleList, areaColor(subtype, style),
          1.0)).first;
    }
    Ring ring;
    for (const auto & point : area.outerBoundPolygon()) {
      ring.push_back({point.x(), point.y()});
    }
    addTriangles(it->second, ring, style.area_z);
  }
  for (auto & [subtype, marker] : areas) {
    push(std::move(marker));
  }

  return markers;
}

}  // namespace ufil_visualization
