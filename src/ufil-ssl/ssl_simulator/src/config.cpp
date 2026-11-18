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

#include "ssl_simulator/config.hpp"

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

namespace ssl_simulator
{

namespace
{

/// Tolerates rounding errors such as 10.0 / 0.2 = 50.000000000000004.
constexpr double kCellEpsilon = 1e-9;
constexpr std::size_t kMaxCellsPerAxis = 65535;

std::size_t cellCount(double extent, double resolution)
{
  return static_cast<std::size_t>(std::ceil(extent / resolution - kCellEpsilon));
}

}  // namespace

std::size_t SimulatorConfig::columns() const
{
  return cellCount(width, resolution);
}

std::size_t SimulatorConfig::rows() const
{
  return cellCount(height, resolution);
}

std::optional<std::string> validate(const SimulatorConfig & config)
{
  const auto & pose = config.pose;
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z) ||
    !std::isfinite(pose.heading))
  {
    return "the mat pose (x, y, z, heading) must be finite";
  }
  if (!(config.width > 0.0) || !std::isfinite(config.width)) {
    return "width must be > 0";
  }
  if (!(config.height > 0.0) || !std::isfinite(config.height)) {
    return "height must be > 0";
  }
  if (!(config.resolution > 0.0) || !std::isfinite(config.resolution)) {
    return "resolution must be > 0";
  }
  if (config.columns() > kMaxCellsPerAxis || config.rows() > kMaxCellsPerAxis) {
    return "width / resolution and height / resolution must not exceed " +
           std::to_string(kMaxCellsPerAxis) + " cells";
  }
  if (config.threshold < 1 || config.threshold > 255) {
    return "threshold must be in [1, 255]";
  }
  if (!(config.stale_duration > 0.0)) {
    return "stale_seconds must be > 0";
  }
  if (!(config.wheel_contact_size > 0.0) || !std::isfinite(config.wheel_contact_size)) {
    return "the wheel contact size must be > 0";
  }
  return std::nullopt;
}

}  // namespace ssl_simulator
