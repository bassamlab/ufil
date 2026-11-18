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

#include "ssl_wheel_tracker/config.hpp"

#include <cmath>
#include <optional>
#include <string>

namespace ssl_wheel_tracker
{

namespace
{

bool positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

std::optional<std::string> validate(const DetectorConfig & config)
{
  if (config.threshold < 0 || config.threshold > 254) {
    return "threshold must be in [0, 254]";
  }
  if (config.min_peak_distance < 1) {
    return "min_peak_distance must be >= 1";
  }
  if (config.contact_radius < 1) {
    return "contact_radius must be >= 1";
  }
  return std::nullopt;
}

std::optional<std::string> validate(const TrackerConfig & config)
{
  if (!positive(config.process_noise)) {
    return "process_noise must be > 0";
  }
  if (!positive(config.measurement_noise)) {
    return "measurement_noise must be > 0";
  }
  if (!positive(config.gate_distance)) {
    return "gate_distance must be > 0";
  }
  if (config.confirmation_hits < 1) {
    return "confirmation_hits must be >= 1";
  }
  if (!positive(config.deletion_timeout)) {
    return "deletion_timeout must be > 0";
  }
  if (config.history_length < 1) {
    return "history_length must be >= 1";
  }
  if (!std::isfinite(config.initial_velocity.x) || !std::isfinite(config.initial_velocity.y)) {
    return "initial_velocity must be finite";
  }
  if (!positive(config.initial_velocity_variance)) {
    return "initial_velocity_variance must be > 0";
  }
  return std::nullopt;
}

}  // namespace ssl_wheel_tracker
