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

#include "ufil_visualization/object_markers.hpp"

#include <numbers>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ufil_visualization
{

namespace
{

Marker makeMarker(
  const std::string & ns, std::int32_t id, MarkerType type, const Color & color,
  double lifetime)
{
  Marker marker;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.color = color;
  marker.lifetime = lifetime;
  return marker;
}

Marker makeArrow(
  const std::string & ns, std::int32_t id, const Point & start, const Point & end,
  const Color & color, double shaft_diameter, double head_diameter, double head_length,
  double lifetime)
{
  Marker marker = makeMarker(ns, id, MarkerType::kArrow, color, lifetime);
  marker.points = {start, end};
  marker.scale = {shaft_diameter, head_diameter, head_length};
  return marker;
}

Marker makeEllipse(
  const std::string & ns, std::int32_t id, const Point & center, const Ellipse & ellipse,
  const Color & color, const ObjectMarkerConfig & config)
{
  Marker marker = makeMarker(ns, id, MarkerType::kLineStrip, color, config.marker_lifetime);
  marker.scale.x = config.covariance_line_width;
  const double cos_angle = std::cos(ellipse.angle);
  const double sin_angle = std::sin(ellipse.angle);
  for (int i = 0; i <= config.ellipse_segments; ++i) {
    const double t = 2.0 * std::numbers::pi * static_cast<double>(i) /
      static_cast<double>(config.ellipse_segments);
    const double x = ellipse.axis_a * std::cos(t);
    const double y = ellipse.axis_b * std::sin(t);
    marker.points.push_back(
      {center.x + x * cos_angle - y * sin_angle, center.y + x * sin_angle + y * cos_angle,
        center.z});
  }
  return marker;
}

Point offset(const Point & point, const Point & delta)
{
  return {point.x + delta.x, point.y + delta.y, point.z + delta.z};
}

/// Icon of an object class (escaped to keep the source ASCII). Lichtblick draws only the outline of
/// a glyph, so only emoji that are a shape on a transparent background work (not e.g. U+1F194, a
/// filled box), and only those presented as emoji by default (no variation selector needed).
std::string classIcon(ObjectClass object_class)
{
  switch (object_class) {
    case ObjectClass::kCar:
      return "\U0001F697";  // automobile
    case ObjectClass::kTruck:
      return "\U0001F69A";  // delivery truck
    case ObjectClass::kMotorcycle:
      return "\U0001F6F5";  // motor scooter
    case ObjectClass::kBicycle:
      return "\U0001F6B2";  // bicycle
    case ObjectClass::kPedestrian:
      return "\U0001F6B6";  // person walking
    case ObjectClass::kStationary:
      return "\U0001F6A7";  // construction barrier
    case ObjectClass::kOther:
      break;
  }
  return "\u2753";  // question mark
}

std::optional<std::string> requirePositive(double value, const std::string & name)
{
  if (!(value > 0.0)) {
    return name + " must be > 0";
  }
  return std::nullopt;
}

}  // namespace

std::optional<std::string> validate(const ObjectMarkerConfig & config)
{
  if (!(config.marker_lifetime >= 0.0)) {
    return "marker_lifetime must be >= 0";
  }
  if (const auto error = validate(config.color)) {
    return "color: " + *error;
  }
  if (!(config.body_alpha >= 0.0 && config.body_alpha <= 1.0)) {
    return "body alpha must be in [0, 1]";
  }
  const std::array<std::pair<double, const char *>, 8> sizes{{
    {config.velocity_shaft_diameter, "velocity shaft diameter"},
    {config.velocity_head_diameter, "velocity head diameter"},
    {config.velocity_head_length, "velocity head length"},
    {config.axis_shaft_diameter, "axis shaft diameter"},
    {config.axis_head_diameter, "axis head diameter"},
    {config.axis_head_length, "axis head length"},
    {config.covariance_line_width, "covariance line width"},
    {config.label_text_size, "label text size"},
  }};
  for (const auto & [value, name] : sizes) {
    if (const auto error = requirePositive(value, name)) {
      return error;
    }
  }
  if (config.ellipse_segments < 3 || config.ellipse_segments > 1024) {
    return "ellipse segments must be in [3, 1024]";
  }
  return std::nullopt;
}

Ellipse ellipseFromCovariance(const std::array<double, 4> & covariance)
{
  const double c00 = covariance[0];
  const double c11 = covariance[3];
  const double sym = 0.5 * (covariance[1] + covariance[2]);
  const double trace = c00 + c11;
  const double determinant = c00 * c11 - sym * sym;
  const double root = std::sqrt(std::max(0.0, trace * trace * 0.25 - determinant));
  const double lambda_1 = std::max(0.0, trace * 0.5 + root);
  const double lambda_2 = std::max(0.0, trace * 0.5 - root);

  Ellipse ellipse;
  ellipse.axis_a = 2.0 * std::sqrt(lambda_1);
  ellipse.axis_b = 2.0 * std::sqrt(lambda_2);
  ellipse.angle = 0.5 * std::atan2(2.0 * sym, c00 - c11);
  return ellipse;
}

ObjectClass mostLikelyClass(const std::array<double, kObjectClassCount> & probabilities)
{
  const auto best = std::max_element(probabilities.begin(), probabilities.end());
  return static_cast<ObjectClass>(std::distance(probabilities.begin(), best));
}

std::string toString(ObjectClass object_class)
{
  switch (object_class) {
    case ObjectClass::kCar:
      return "CAR";
    case ObjectClass::kTruck:
      return "TRUCK";
    case ObjectClass::kMotorcycle:
      return "MOTORCYCLE";
    case ObjectClass::kBicycle:
      return "BICYCLE";
    case ObjectClass::kPedestrian:
      return "PEDESTRIAN";
    case ObjectClass::kStationary:
      return "STATIONARY";
    case ObjectClass::kOther:
      break;
  }
  return "OTHER";
}

std::vector<Marker> objectMarkers(
  const Object & object, std::int32_t id, const ObjectMarkerConfig & config)
{
  const double lifetime = config.marker_lifetime;
  const Point center{object.x, object.y, 0.5 * object.height};
  const Quaternion orientation = quaternionFromYaw(object.yaw);
  std::vector<Marker> markers;

  const Color & color = config.color;
  Color body_color = color;
  body_color.a = static_cast<float>(color.a * config.body_alpha);
  Marker body = makeMarker("body", id, MarkerType::kCube, body_color, lifetime);
  body.position = center;
  body.orientation = orientation;
  body.scale = {object.length, object.width, object.height};
  markers.push_back(body);

  const Point ground{object.x, object.y, 0.0};
  markers.push_back(
    makeArrow(
      "velocity", id, ground, offset(ground, {object.vx, object.vy, 0.0}),
      color, config.velocity_shaft_diameter, config.velocity_head_diameter,
      config.velocity_head_length, lifetime));

  const std::array<std::pair<const char *, Point>, 3> axes{{
    {"body_axis_x", {0.5 * object.length + 1.0, 0.0, 0.0}},
    {"body_axis_y", {0.0, 0.5 * object.width + 1.0, 0.0}},
    {"body_axis_z", {0.0, 0.0, 0.5 * object.height + 1.0}},
  }};
  for (const auto & [ns, axis] : axes) {
    markers.push_back(
      makeArrow(
        ns, id, center, offset(center, rotate(orientation, axis)), color,
        config.axis_shaft_diameter, config.axis_head_diameter, config.axis_head_length,
        lifetime));
  }

  const auto & covariance = object.position_covariance;
  if (covariance[0] >= 0.0 && covariance[3] >= 0.0) {
    markers.push_back(
      makeEllipse(
        "covariance_position", id, center, ellipseFromCovariance(covariance), color,
        config));
  }

  // The yaw uncertainty is drawn as an ellipse above the object whose size grows with it.
  if (object.yaw_variance >= 0.0) {
    Ellipse yaw_ellipse;
    yaw_ellipse.axis_a = std::max(0.1, 1.6 * std::sqrt(object.yaw_variance));
    yaw_ellipse.axis_b = 0.5 * yaw_ellipse.axis_a;
    markers.push_back(
      makeEllipse(
        "covariance_orientation", id, offset(center, {0.0, 0.0, object.height + 0.1}),
        yaw_ellipse, color, config));
  }

  // One compact line "<class> #<id>  \u2203<existence>%" in a single marker, so the parts never
  // overlap (separate markers stacked in z would when seen from above) and the labels of many
  // objects stay small. The class is an icon, or its name without icons.
  const ObjectClass object_class = mostLikelyClass(object.class_probabilities);
  Marker label = makeMarker("label", id, MarkerType::kTextViewFacing, color, lifetime);
  label.position = offset(center, {0.0, 0.0, object.height + 0.3});
  label.scale.z = config.label_text_size;
  label.text = (config.label_icons ? classIcon(object_class) : toString(object_class)) + " #" +
    std::to_string(object.id) + "  \u2203" +  // there exists
    std::to_string(std::lround(100.0 * object.existence_probability)) + "%";
  markers.push_back(label);

  return markers;
}

std::vector<Marker> objectListMarkers(
  const std::vector<Object> & objects, const ObjectMarkerConfig & config)
{
  std::vector<Marker> markers;
  for (std::size_t i = 0; i < objects.size(); ++i) {
    auto object_markers = objectMarkers(objects[i], static_cast<std::int32_t>(i), config);
    markers.insert(
      markers.end(), std::make_move_iterator(object_markers.begin()),
      std::make_move_iterator(object_markers.end()));
  }
  return markers;
}

}  // namespace ufil_visualization
