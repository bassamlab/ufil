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

#ifndef UFIL_VISUALIZATION__MARKER_HPP_
#define UFIL_VISUALIZATION__MARKER_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ufil_visualization/visibility_control.hpp"

namespace ufil_visualization
{

/// RGBA colour, every channel in [0, 1].
struct Color
{
  float r = 0.0F;
  float g = 0.0F;
  float b = 0.0F;
  float a = 1.0F;
};

struct Point
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Vector3
{
  double x = 1.0;
  double y = 1.0;
  double z = 1.0;
};

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

/// Marker shapes, with the semantics of the corresponding visualization_msgs/Marker types.
enum class MarkerType
{
  kArrow,
  kCube,
  kLineStrip,
  kLineList,
  kTriangleList,
  kTextViewFacing,
};

/**
 * @brief ROS-independent description of one visualization marker.
 *
 * The fields follow visualization_msgs/Marker: an arrow is given by its start and end point in
 * points with scale = (shaft diameter, head diameter, head length); lines use scale.x as width;
 * text uses scale.z as height. Coordinates are in the frame of the visualized data.
 */
struct Marker
{
  std::string ns;
  std::int32_t id = 0;
  MarkerType type = MarkerType::kCube;
  Point position;
  Quaternion orientation;
  Vector3 scale;
  Color color;
  std::vector<Point> points;
  std::string text;
  /// Seconds until the marker is removed by the viewer, 0 for forever.
  double lifetime = 0.0;
  /// The marker is re-transformed into the fixed frame of the viewer every frame.
  bool frame_locked = false;
};

/// @return an error message if a channel of the colour is not in [0, 1].
UFIL_VISUALIZATION_PUBLIC std::optional<std::string> validate(const Color & color);

/// Rotation about the z axis by yaw [rad].
UFIL_VISUALIZATION_PUBLIC Quaternion quaternionFromYaw(double yaw);

/// Rotates the vector v by the unit quaternion q.
UFIL_VISUALIZATION_PUBLIC Point rotate(const Quaternion & q, const Point & v);

}  // namespace ufil_visualization

#endif  // UFIL_VISUALIZATION__MARKER_HPP_
