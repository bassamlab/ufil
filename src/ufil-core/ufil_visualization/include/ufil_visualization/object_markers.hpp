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

#ifndef UFIL_VISUALIZATION__OBJECT_MARKERS_HPP_
#define UFIL_VISUALIZATION__OBJECT_MARKERS_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ufil_visualization/marker.hpp"
#include "ufil_visualization/visibility_control.hpp"

namespace ufil_visualization
{

/// Object classes, in the order of the classification vector of ufil_msgs/Classification.
enum class ObjectClass : std::uint8_t
{
  kCar = 0,
  kTruck,
  kMotorcycle,
  kBicycle,
  kPedestrian,
  kStationary,
  kOther,
};

inline constexpr std::size_t kObjectClassCount = 7;

/// Visualized properties of a tracked or detected object.
struct Object
{
  std::uint32_t id = 0;
  /// Position of the geometric centre on the ground [m].
  double x = 0.0;
  double y = 0.0;
  /// Yaw angle [rad].
  double yaw = 0.0;
  /// Absolute velocity [m/s].
  double vx = 0.0;
  double vy = 0.0;
  /// Box dimensions [m].
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  /// Row-major 2x2 covariance of x and y [m^2]. A negative variance means unknown.
  std::array<double, 4> position_covariance{};
  /// Variance of the yaw angle [rad^2]. Negative means unknown.
  double yaw_variance = -1.0;
  double existence_probability = 0.0;
  /// Probability per class, indexed by ObjectClass.
  std::array<double, kObjectClassCount> class_probabilities{};
};

/// Appearance of the object markers.
struct ObjectMarkerConfig
{
  /// Seconds until markers disappear if no new object list arrives, 0 for never.
  double marker_lifetime = 1.0;

  /// Colour of all markers of the object list. One colour per source (e.g. OSN, OBU) tells the
  /// sources apart in the viewer; the elements are told apart by their namespace.
  Color color{0.604F, 0.600F, 0.588F, 1.0F};
  /// Opacity of the box relative to color.a, so overlapping boxes of several sources and the
  /// axes inside the box stay visible.
  double body_alpha = 0.5;

  double velocity_shaft_diameter = 0.1;
  double velocity_head_diameter = 0.15;
  double velocity_head_length = 0.2;

  double axis_shaft_diameter = 0.1;
  double axis_head_diameter = 0.15;
  double axis_head_length = 0.2;

  double covariance_line_width = 0.05;
  /// Line segments per covariance ellipse.
  int ellipse_segments = 64;

  /// Height of the label texts [m]; the default matches the lanelet ids.
  double label_text_size = 0.8;
  /// Show the class as an icon (emoji) instead of its name.
  bool label_icons = true;
};

/// @return an error message if the configuration is invalid.
UFIL_VISUALIZATION_PUBLIC std::optional<std::string> validate(const ObjectMarkerConfig & config);

/// Ellipse with the semi-axes a, b, a rotated by angle [rad] against the x axis.
struct Ellipse
{
  double axis_a = 0.0;
  double axis_b = 0.0;
  double angle = 0.0;
};

/// Two-sigma ellipse of a 2x2 covariance matrix; the off-diagonal entries are symmetrised.
UFIL_VISUALIZATION_PUBLIC Ellipse ellipseFromCovariance(const std::array<double, 4> & covariance);

/// Most probable class; ties resolve to the lower class index.
UFIL_VISUALIZATION_PUBLIC ObjectClass mostLikelyClass(
  const std::array<double, kObjectClassCount> & probabilities);

/// Upper case name of the class, e.g. "PEDESTRIAN".
UFIL_VISUALIZATION_PUBLIC std::string toString(ObjectClass object_class);

/**
 * @brief Markers of one object, all with the given id and config.color.
 *
 * Namespaces: body (cube), velocity (arrow on the ground), body_axis_x/y/z (arrows),
 * covariance_position and covariance_orientation (line strips, omitted if the variance is
 * unknown) and label (text "<class> #<id>  \u2203<existence probability>%", the class as an
 * icon if config.label_icons is set, otherwise its name). The namespaces let the viewer show or
 * hide every element separately.
 */
UFIL_VISUALIZATION_PUBLIC std::vector<Marker> objectMarkers(
  const Object & object, std::int32_t id, const ObjectMarkerConfig & config);

/// Markers of all objects; the markers of objects[i] have the id i.
UFIL_VISUALIZATION_PUBLIC std::vector<Marker> objectListMarkers(
  const std::vector<Object> & objects, const ObjectMarkerConfig & config);

}  // namespace ufil_visualization

#endif  // UFIL_VISUALIZATION__OBJECT_MARKERS_HPP_
