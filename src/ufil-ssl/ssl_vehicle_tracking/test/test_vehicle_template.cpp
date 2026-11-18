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

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ssl_vehicle_tracking/vehicle_template.hpp"

using ssl_vehicle_tracking::Axle;
using ssl_vehicle_tracking::Point2D;
using ssl_vehicle_tracking::Pose2D;
using ssl_vehicle_tracking::TemplateDescription;
using ssl_vehicle_tracking::VehicleTemplate;

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string templatePath(const std::string & name)
{
  return std::string(TEMPLATE_DIR) + "/" + name + ".json";
}

TemplateDescription car()
{
  TemplateDescription description;
  description.name = "car";
  description.length = 4.0;
  description.width = 1.8;
  Axle front;
  front.track_width = 1.6;
  front.center_to_axle = 1.5;
  front.wheel_detected = {true, true};
  Axle rear = front;
  rear.track_width = 1.8;
  rear.center_to_axle = -1.2;
  description.axles = {front, rear};
  return description;
}

}  // namespace

TEST(VehicleTemplate, WheelLayout)
{
  const VehicleTemplate vehicle_template(car());
  ASSERT_EQ(vehicle_template.wheelCount(), 4u);
  EXPECT_DOUBLE_EQ(vehicle_template.wheels()[0].x, 1.5);
  EXPECT_DOUBLE_EQ(vehicle_template.wheels()[0].y, -0.8);
  EXPECT_DOUBLE_EQ(vehicle_template.wheels()[1].y, 0.8);
  EXPECT_DOUBLE_EQ(vehicle_template.wheels()[3].x, -1.2);
  EXPECT_DOUBLE_EQ(vehicle_template.wheels()[3].y, 0.9);

  EXPECT_NEAR(vehicle_template.distance(0, 3), std::hypot(2.7, 1.7), 1e-12);
  EXPECT_DOUBLE_EQ(vehicle_template.distance(1, 2), vehicle_template.distance(0, 3));
  EXPECT_NEAR(vehicle_template.angle(0, 1), kPi / 2.0, 1e-12);
  EXPECT_NEAR(vehicle_template.angle(1, 0), -kPi / 2.0, 1e-12);
  // The diagonals are the longest pairs; the first in row-major order is the longest pair.
  EXPECT_EQ(vehicle_template.longestPair(), (std::pair<std::size_t, std::size_t>{0, 3}));

  const auto & pairs = vehicle_template.pairsByDistance();
  ASSERT_EQ(pairs.size(), 6u);
  for (std::size_t i = 1; i < pairs.size(); ++i) {
    EXPECT_GE(
      vehicle_template.distance(pairs[i - 1].first, pairs[i - 1].second),
      vehicle_template.distance(pairs[i].first, pairs[i].second));
  }
  // Equal distances in descending row-major order.
  EXPECT_EQ(pairs[0], (std::pair<std::size_t, std::size_t>{3, 0}));
  EXPECT_EQ(pairs[1], (std::pair<std::size_t, std::size_t>{2, 1}));
}

TEST(VehicleTemplate, SingleTrackAxles)
{
  auto description = car();
  description.axles[0].single_track = true;
  description.axles[0].track_width = 0.0;
  const VehicleTemplate vehicle_template(description);
  ASSERT_EQ(vehicle_template.wheelCount(), 3u);
  EXPECT_DOUBLE_EQ(vehicle_template.wheels()[0].y, 0.0);
}

TEST(VehicleTemplate, InvalidDescriptionsThrow)
{
  auto one_wheel = car();
  one_wheel.axles = {one_wheel.axles[0]};
  one_wheel.axles[0].single_track = true;
  EXPECT_THROW(VehicleTemplate{one_wheel}, std::invalid_argument);

  auto classification = car();
  classification.classification = 7;
  EXPECT_THROW(VehicleTemplate{classification}, std::invalid_argument);

  auto track_width = car();
  track_width.axles[1].track_width = 0.0;
  EXPECT_THROW(VehicleTemplate{track_width}, std::invalid_argument);

  auto not_finite = car();
  not_finite.axles[1].center_to_axle = std::nan("");
  EXPECT_THROW(VehicleTemplate{not_finite}, std::invalid_argument);
}

TEST(VehicleTemplate, LoadsTheShippedTemplates)
{
  const auto car_carla = ssl_vehicle_tracking::loadTemplate(templatePath("car_carla"), "car");
  EXPECT_EQ(car_carla.wheelCount(), 4u);
  EXPECT_EQ(car_carla.description().name, "car");
  EXPECT_EQ(car_carla.description().axles[0].wheel_detected.size(), 2u);

  const auto truck = ssl_vehicle_tracking::loadTemplate(templatePath("truck_carla"), "truck");
  EXPECT_EQ(truck.wheelCount(), 6u);
  EXPECT_EQ(truck.description().classification, 1u);

  const auto motorcycle =
    ssl_vehicle_tracking::loadTemplate(templatePath("motorcycle_carla"), "motorcycle");
  EXPECT_EQ(motorcycle.wheelCount(), 2u);
  EXPECT_EQ(motorcycle.description().classification, 2u);

  EXPECT_EQ(ssl_vehicle_tracking::loadTemplate(templatePath("muCar_lab"), "mu").wheelCount(), 4u);
  EXPECT_EQ(
    ssl_vehicle_tracking::loadTemplate(templatePath("muCar_scaled_lab"), "mu").wheelCount(), 4u);
}

TEST(VehicleTemplate, LoadingInvalidFilesThrows)
{
  EXPECT_THROW(
    ssl_vehicle_tracking::loadTemplate("/nonexistent/template.json", "x"), std::runtime_error);
  EXPECT_THROW(
    ssl_vehicle_tracking::loadTemplate(std::string(TEMPLATE_DIR), "x"), std::runtime_error);
}

TEST(FitTemplate, RecoversThePose)
{
  const VehicleTemplate vehicle_template(car());
  const Pose2D pose{10.0, -3.0, 2.5};
  std::vector<Point2D> points;
  for (const auto & wheel : vehicle_template.wheels()) {
    points.push_back(ssl_vehicle_tracking::transformPoint(pose, wheel));
  }

  const Pose2D all = ssl_vehicle_tracking::fitTemplate(vehicle_template, points, {0, 1, 2, 3}, 2);
  EXPECT_NEAR(all.x, pose.x, 1e-9);
  EXPECT_NEAR(all.y, pose.y, 1e-9);
  EXPECT_NEAR(all.yaw, pose.yaw, 1e-9);

  // Any two wheels in any order suffice.
  const Pose2D two = ssl_vehicle_tracking::fitTemplate(
    vehicle_template, {points[3], points[1]}, {3, 1}, 2);
  EXPECT_NEAR(two.x, pose.x, 1e-9);
  EXPECT_NEAR(two.y, pose.y, 1e-9);
  EXPECT_NEAR(two.yaw, pose.yaw, 1e-9);
}

TEST(FitTemplate, AveragesTheRotationOfTheLongestPairs)
{
  const VehicleTemplate vehicle_template(car());
  auto points = vehicle_template.wheels();
  // Move wheel 0: the diagonal 3-0 rotates, the diagonal 2-1 does not.
  points[0].y += 0.2;
  const double rotated =
    std::atan2(points[0].y - points[3].y, points[0].x - points[3].x) -
    vehicle_template.angle(3, 0);
  const Pose2D pose = ssl_vehicle_tracking::fitTemplate(vehicle_template, points, {0, 1, 2, 3}, 2);
  // Both diagonals have the same length, so the mean is halfway.
  EXPECT_NEAR(pose.yaw, std::atan2(std::sin(rotated), std::cos(rotated) + 1.0), 1e-12);

  const Pose2D one = ssl_vehicle_tracking::fitTemplate(vehicle_template, points, {0, 1, 2, 3}, 1);
  EXPECT_NEAR(one.yaw, rotated, 1e-12);
}
