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

#include "ufil_central_fusion/config.hpp"

#include <cmath>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace ufil_central_fusion
{

namespace
{

bool isProbability(double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

}  // namespace

std::optional<std::string> validate(const FusionConfig & config)
{
  if (!std::isfinite(config.association_threshold) || config.association_threshold <= 0.0) {
    return "association threshold must be > 0";
  }
  if (!isProbability(config.existence_min_weight) ||
    !isProbability(config.existence_max_weight) ||
    config.existence_min_weight > config.existence_max_weight)
  {
    return "existence weights must satisfy 0 <= min_weight <= max_weight <= 1";
  }
  if (!isProbability(config.existence_decay_factor)) {
    return "existence decay factor must be in [0, 1]";
  }
  if (!std::isfinite(config.dimension_grid_resolution) ||
    config.dimension_grid_resolution <= 0.0)
  {
    return "dimension grid resolution must be > 0";
  }
  if (!isProbability(config.dimension_alpha) || config.dimension_alpha == 0.0) {
    return "dimension alpha must be in (0, 1]";
  }
  if (!isProbability(config.dimension_p_min) || !isProbability(config.dimension_p_ref) ||
    !isProbability(config.dimension_p_max) || config.dimension_p_min > config.dimension_p_ref ||
    config.dimension_p_ref > config.dimension_p_max)
  {
    return "dimension probabilities must satisfy 0 <= p_min <= p_ref <= p_max <= 1";
  }
  if (!std::isfinite(config.buffer_duration) || config.buffer_duration <= 0.0) {
    return "buffer duration must be > 0";
  }
  if (!std::isfinite(config.time_jump_threshold) || config.time_jump_threshold <= 0.0) {
    return "time jump threshold must be > 0";
  }
  return std::nullopt;
}

std::optional<std::string> validate(const std::vector<SensorConfig> & sensors)
{
  if (sensors.empty()) {
    return "at least one sensor must be configured";
  }
  std::set<std::string> names;
  for (const auto & sensor : sensors) {
    if (sensor.name.empty()) {
      return "sensor names must not be empty";
    }
    if (!names.insert(sensor.name).second) {
      return "sensor name '" + sensor.name + "' is not unique";
    }
    const auto & t = sensor.trust;
    for (const double value : {t.existence, t.car, t.truck, t.motorcycle, t.bicycle,
        t.pedestrian, t.stationary, t.other})
    {
      if (!isProbability(value)) {
        return "trust values of sensor '" + sensor.name + "' must be in [0, 1]";
      }
    }
  }
  return std::nullopt;
}

}  // namespace ufil_central_fusion
