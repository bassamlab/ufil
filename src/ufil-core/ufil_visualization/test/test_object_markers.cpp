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

#include <numbers>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "ufil_visualization/marker.hpp"
#include "ufil_visualization/object_markers.hpp"

using ufil_visualization::Color;
using ufil_visualization::Ellipse;
using ufil_visualization::Marker;
using ufil_visualization::MarkerType;
using ufil_visualization::Object;
using ufil_visualization::ObjectClass;
using ufil_visualization::ObjectMarkerConfig;
using ufil_visualization::Point;

namespace
{

Object makeObject()
{
  Object object;
  object.id = 42;
  object.x = 10.0;
  object.y = -5.0;
  object.yaw = std::numbers::pi / 2.0;
  object.vx = 2.0;
  object.vy = 1.0;
  object.length = 4.0;
  object.width = 2.0;
  object.height = 1.5;
  object.position_covariance = {4.0, 0.0, 0.0, 1.0};
  object.yaw_variance = 0.25;
  object.existence_probability = 0.5;
  object.class_probabilities[static_cast<std::size_t>(ObjectClass::kPedestrian)] = 0.9;
  return object;
}

Marker find(const std::vector<Marker> & markers, const std::string & ns)
{
  for (const auto & marker : markers) {
    if (marker.ns == ns) {
      return marker;
    }
  }
  throw std::runtime_error("no marker " + ns);
}

bool hasMarker(const std::vector<Marker> & markers, const std::string & ns)
{
  for (const auto & marker : markers) {
    if (marker.ns == ns) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(MarkerTest, QuaternionFromYawRotatesAboutZ)
{
  const auto q = ufil_visualization::quaternionFromYaw(std::numbers::pi / 2.0);
  const Point rotated = ufil_visualization::rotate(q, {1.0, 0.0, 0.0});
  EXPECT_NEAR(rotated.x, 0.0, 1e-12);
  EXPECT_NEAR(rotated.y, 1.0, 1e-12);
  EXPECT_NEAR(rotated.z, 0.0, 1e-12);
  const Point up = ufil_visualization::rotate(q, {0.0, 0.0, 2.0});
  EXPECT_NEAR(up.z, 2.0, 1e-12);
}

TEST(MarkerTest, ValidatesColorRange)
{
  EXPECT_FALSE(ufil_visualization::validate(Color{0.0F, 0.5F, 1.0F, 1.0F}));
  EXPECT_TRUE(ufil_visualization::validate(Color{1.5F, 0.0F, 0.0F, 1.0F}));
  EXPECT_TRUE(ufil_visualization::validate(Color{0.0F, 0.0F, 0.0F, -0.1F}));
  EXPECT_TRUE(ufil_visualization::validate(Color{0.0F, 0.0F, 0.0F, std::nanf("")}));
}

TEST(ObjectMarkersTest, DefaultConfigIsValid)
{
  EXPECT_FALSE(ufil_visualization::validate(ObjectMarkerConfig{}));
}

TEST(ObjectMarkersTest, RejectsInvalidConfig)
{
  ObjectMarkerConfig config;
  config.ellipse_segments = 2;
  EXPECT_TRUE(ufil_visualization::validate(config));

  config = ObjectMarkerConfig{};
  config.covariance_line_width = 0.0;
  EXPECT_TRUE(ufil_visualization::validate(config));

  config = ObjectMarkerConfig{};
  config.marker_lifetime = -1.0;
  EXPECT_TRUE(ufil_visualization::validate(config));

  config = ObjectMarkerConfig{};
  config.body_alpha = -0.1;
  EXPECT_TRUE(ufil_visualization::validate(config));

  config = ObjectMarkerConfig{};
  config.color.a = 2.0F;
  const auto error = ufil_visualization::validate(config);
  ASSERT_TRUE(error);
  EXPECT_NE(error->find("color"), std::string::npos);
}

TEST(ObjectMarkersTest, EllipseOfDiagonalCovariance)
{
  const Ellipse ellipse = ufil_visualization::ellipseFromCovariance({4.0, 0.0, 0.0, 1.0});
  EXPECT_DOUBLE_EQ(ellipse.axis_a, 4.0);
  EXPECT_DOUBLE_EQ(ellipse.axis_b, 2.0);
  EXPECT_DOUBLE_EQ(ellipse.angle, 0.0);

  const Ellipse along_y = ufil_visualization::ellipseFromCovariance({1.0, 0.0, 0.0, 4.0});
  EXPECT_DOUBLE_EQ(along_y.axis_a, 4.0);
  EXPECT_NEAR(std::abs(along_y.angle), std::numbers::pi / 2.0, 1e-12);
}

TEST(ObjectMarkersTest, EllipseOfCorrelatedCovariance)
{
  // Eigenvalues 3 and 1 with the major axis along the diagonal.
  const Ellipse ellipse = ufil_visualization::ellipseFromCovariance({2.0, 1.0, 1.0, 2.0});
  EXPECT_NEAR(ellipse.axis_a, 2.0 * std::sqrt(3.0), 1e-12);
  EXPECT_NEAR(ellipse.axis_b, 2.0, 1e-12);
  EXPECT_NEAR(ellipse.angle, std::numbers::pi / 4.0, 1e-12);
}

TEST(ObjectMarkersTest, MostLikelyClass)
{
  std::array<double, ufil_visualization::kObjectClassCount> probabilities{};
  probabilities[static_cast<std::size_t>(ObjectClass::kTruck)] = 0.6;
  probabilities[static_cast<std::size_t>(ObjectClass::kCar)] = 0.4;
  EXPECT_EQ(ufil_visualization::mostLikelyClass(probabilities), ObjectClass::kTruck);
  EXPECT_EQ(ufil_visualization::toString(ObjectClass::kTruck), "TRUCK");
  EXPECT_EQ(ufil_visualization::toString(ObjectClass::kOther), "OTHER");
}

TEST(ObjectMarkersTest, MarkersOfOneObject)
{
  const ObjectMarkerConfig config;
  const auto markers = ufil_visualization::objectMarkers(makeObject(), 7, config);
  ASSERT_EQ(markers.size(), 8u);
  for (const auto & marker : markers) {
    EXPECT_EQ(marker.id, 7);
    EXPECT_DOUBLE_EQ(marker.lifetime, config.marker_lifetime);
  }

  const auto body = find(markers, "body");
  EXPECT_EQ(body.type, MarkerType::kCube);
  EXPECT_DOUBLE_EQ(body.position.x, 10.0);
  EXPECT_DOUBLE_EQ(body.position.y, -5.0);
  EXPECT_DOUBLE_EQ(body.position.z, 0.75);
  EXPECT_DOUBLE_EQ(body.scale.x, 4.0);
  EXPECT_DOUBLE_EQ(body.scale.y, 2.0);
  EXPECT_DOUBLE_EQ(body.scale.z, 1.5);
  EXPECT_NEAR(body.orientation.z, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(body.orientation.w, std::sqrt(0.5), 1e-12);
  EXPECT_FLOAT_EQ(body.color.b, config.color.b);
  EXPECT_FLOAT_EQ(body.color.a, static_cast<float>(config.color.a * config.body_alpha));
  for (const auto & marker : markers) {
    if (marker.ns != "body") {
      EXPECT_FLOAT_EQ(marker.color.r, config.color.r) << marker.ns;
      EXPECT_FLOAT_EQ(marker.color.g, config.color.g) << marker.ns;
      EXPECT_FLOAT_EQ(marker.color.b, config.color.b) << marker.ns;
      EXPECT_FLOAT_EQ(marker.color.a, config.color.a) << marker.ns;
    }
  }

  const auto velocity = find(markers, "velocity");
  EXPECT_EQ(velocity.type, MarkerType::kArrow);
  ASSERT_EQ(velocity.points.size(), 2u);
  EXPECT_DOUBLE_EQ(velocity.points[0].z, 0.0);
  EXPECT_DOUBLE_EQ(velocity.points[1].x, 12.0);
  EXPECT_DOUBLE_EQ(velocity.points[1].y, -4.0);
  EXPECT_DOUBLE_EQ(velocity.scale.x, config.velocity_shaft_diameter);

  // The longitudinal axis points along +y for a yaw of 90 degrees.
  const auto axis_x = find(markers, "body_axis_x");
  ASSERT_EQ(axis_x.points.size(), 2u);
  EXPECT_NEAR(axis_x.points[1].x, 10.0, 1e-12);
  EXPECT_NEAR(axis_x.points[1].y, -5.0 + 3.0, 1e-12);
  EXPECT_NEAR(find(markers, "body_axis_z").points[1].z, 0.75 + 1.75, 1e-12);

  const auto position_covariance = find(markers, "covariance_position");
  EXPECT_EQ(position_covariance.type, MarkerType::kLineStrip);
  ASSERT_EQ(position_covariance.points.size(),
    static_cast<std::size_t>(config.ellipse_segments + 1));
  EXPECT_NEAR(position_covariance.points.front().x, 10.0 + 4.0, 1e-12);
  EXPECT_NEAR(position_covariance.points.front().y, -5.0, 1e-12);

  const auto orientation_covariance = find(markers, "covariance_orientation");
  EXPECT_NEAR(orientation_covariance.points.front().x, 10.0 + 0.8, 1e-12);
  EXPECT_NEAR(orientation_covariance.points.front().z, 0.75 + 1.5 + 0.1, 1e-12);

  const auto label = find(markers, "label");
  EXPECT_EQ(label.type, MarkerType::kTextViewFacing);
  EXPECT_EQ(label.text, "\U0001F6B6 #42  \u220350%");
  EXPECT_DOUBLE_EQ(label.scale.z, config.label_text_size);
  EXPECT_NEAR(label.position.z, 0.75 + 1.5 + 0.3, 1e-12);
}

TEST(ObjectMarkersTest, LabelsWithoutIcons)
{
  ObjectMarkerConfig config;
  config.label_icons = false;
  const auto markers = ufil_visualization::objectMarkers(makeObject(), 0, config);
  EXPECT_EQ(find(markers, "label").text, "PEDESTRIAN #42  \u220350%");
}

TEST(ObjectMarkersTest, UnknownCovarianceIsNotDrawn)
{
  Object object = makeObject();
  object.position_covariance = {-1.0, 0.0, 0.0, -1.0};
  object.yaw_variance = -1.0;
  const auto markers = ufil_visualization::objectMarkers(object, 0, ObjectMarkerConfig{});
  EXPECT_EQ(markers.size(), 6u);
  EXPECT_FALSE(hasMarker(markers, "covariance_position"));
  EXPECT_FALSE(hasMarker(markers, "covariance_orientation"));
}

TEST(ObjectMarkersTest, ObjectListUsesIndexAsId)
{
  const auto markers = ufil_visualization::objectListMarkers(
    {makeObject(), makeObject()}, ObjectMarkerConfig{});
  ASSERT_EQ(markers.size(), 16u);
  EXPECT_EQ(markers.front().id, 0);
  EXPECT_EQ(markers.back().id, 1);
  EXPECT_TRUE(ufil_visualization::objectListMarkers({}, ObjectMarkerConfig{}).empty());
}
