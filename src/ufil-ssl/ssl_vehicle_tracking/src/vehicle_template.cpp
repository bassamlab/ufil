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

#include "ssl_vehicle_tracking/vehicle_template.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ssl_vehicle_tracking
{

namespace
{

bool finite(double value)
{
  return std::isfinite(value);
}

void validateDescription(const TemplateDescription & description)
{
  const auto fail = [&description](const std::string & reason) {
      throw std::invalid_argument("vehicle template '" + description.name + "': " + reason);
    };
  if (!finite(description.length) || description.length < 0.0 ||
    !finite(description.width) || description.width < 0.0)
  {
    fail("length and width must be finite and >= 0");
  }
  if (description.classification >= kClassCount) {
    fail("classification must be < " + std::to_string(kClassCount));
  }
  for (const auto & axle : description.axles) {
    if (!finite(axle.center_to_axle) || !finite(axle.track_width) ||
      !finite(axle.track_width_variance) || !finite(axle.center_to_axle_variance))
    {
      fail("axle values must be finite");
    }
    if (!axle.single_track && axle.track_width <= 0.0) {
      fail("track_width of a two-track axle must be > 0");
    }
  }
}

}  // namespace

VehicleTemplate::VehicleTemplate(TemplateDescription description)
: description_(std::move(description))
{
  validateDescription(description_);
  for (const auto & axle : description_.axles) {
    if (axle.single_track) {
      wheels_.push_back({axle.center_to_axle, 0.0});
    } else {
      wheels_.push_back({axle.center_to_axle, -axle.track_width / 2.0});
      wheels_.push_back({axle.center_to_axle, axle.track_width / 2.0});
    }
  }
  const std::size_t n = wheels_.size();
  if (n < 2) {
    throw std::invalid_argument(
            "vehicle template '" + description_.name + "': at least two wheels are required");
  }

  distances_.resize(n * n);
  angles_.resize(n * n);
  for (std::size_t a = 0; a < n; ++a) {
    for (std::size_t b = 0; b < n; ++b) {
      const double dx = wheels_[b].x - wheels_[a].x;
      const double dy = wheels_[b].y - wheels_[a].y;
      distances_[a * n + b] = std::hypot(dx, dy);
      angles_[a * n + b] = std::atan2(dy, dx);
    }
  }

  std::size_t longest = 0;
  for (std::size_t i = 1; i < n * n; ++i) {
    if (distances_[i] > distances_[longest]) {
      longest = i;
    }
  }
  longest_pair_ = {longest / n, longest % n};

  // Longest first; equal distances in descending row-major order.
  std::vector<std::size_t> order(n * n);
  std::iota(order.begin(), order.end(), 0U);
  std::stable_sort(
    order.begin(), order.end(),
    [this](std::size_t i, std::size_t j) {return distances_[i] < distances_[j];});
  std::reverse(order.begin(), order.end());
  for (const auto index : order) {
    const std::size_t a = index / n;
    const std::size_t b = index % n;
    if (a == b ||
      std::find(
        pairs_by_distance_.begin(), pairs_by_distance_.end(),
        std::pair<std::size_t, std::size_t>{b, a}) != pairs_by_distance_.end())
    {
      continue;
    }
    pairs_by_distance_.emplace_back(a, b);
  }
}

VehicleTemplate loadTemplate(const std::string & path, const std::string & name)
{
  try {
    const YAML::Node root = YAML::LoadFile(path);
    TemplateDescription description;
    description.name = name;
    description.length = root["length"].as<double>();
    description.width = root["width"].as<double>();
    description.classification = root["classification"].as<std::size_t>();
    for (const auto & node : root["axles"]) {
      Axle axle;
      axle.single_track = node["single_track"].as<bool>();
      axle.track_width = node["track_width"].as<double>();
      axle.track_width_variance = node["track_width_variance"].as<double>();
      axle.center_to_axle = node["center_to_axle"].as<double>();
      axle.center_to_axle_variance = node["center_to_axle_variance"].as<double>();
      axle.wheel_detected = node["wheel_detected"].as<std::vector<bool>>();
      description.axles.push_back(axle);
    }
    return VehicleTemplate(std::move(description));
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot load vehicle template '" + path + "': " + error.what());
  } catch (const std::invalid_argument & error) {
    throw std::runtime_error(error.what());
  }
}

Pose2D fitTemplate(
  const VehicleTemplate & vehicle_template, const std::vector<Point2D> & points,
  const std::vector<std::size_t> & wheels, std::size_t max_pairs)
{
  const auto point_of = [&](std::size_t wheel) -> const Point2D * {
      const auto it = std::find(wheels.begin(), wheels.end(), wheel);
      return it == wheels.end() ? nullptr : &points[static_cast<std::size_t>(it - wheels.begin())];
    };

  double x = 0.0;
  double y = 0.0;
  std::size_t used = 0;
  for (const auto & [a, b] : vehicle_template.pairsByDistance()) {
    if (used >= max_pairs) {
      break;
    }
    const Point2D * pa = point_of(a);
    const Point2D * pb = point_of(b);
    if (pa == nullptr || pb == nullptr) {
      continue;
    }
    ++used;
    const double difference =
      std::atan2(pb->y - pa->y, pb->x - pa->x) - vehicle_template.angle(a, b);
    const double weight = vehicle_template.distance(a, b);
    x += std::cos(difference) * weight;
    y += std::sin(difference) * weight;
  }

  Pose2D pose;
  pose.yaw = std::atan2(y, x);
  if (!points.empty()) {
    const Point2D rotated = transformPoint({0.0, 0.0, pose.yaw},
        vehicle_template.wheels()[wheels[0]]);
    pose.x = points[0].x - rotated.x;
    pose.y = points[0].y - rotated.y;
  }
  return pose;
}

Point2D transformPoint(const Pose2D & pose, const Point2D & point)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * point.x - s * point.y, pose.y + s * point.x + c * point.y};
}

}  // namespace ssl_vehicle_tracking
