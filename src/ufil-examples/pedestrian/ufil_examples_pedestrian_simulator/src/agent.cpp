// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#include "ufil_examples_pedestrian_simulator/agent.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace ufil_examples_pedestrian_simulator
{

namespace
{
constexpr double kTimeTolerance = 1e-9;
}  // namespace

Agent::Agent(
  const std::int64_t id, const Eigen::Vector3f & dimension, std::vector<Waypoint> waypoints,
  const std::optional<double> orientation, std::vector<Occlusion> occlusions)
: id_(id), dimension_(dimension), orientation_(orientation), waypoints_(std::move(waypoints)),
  occlusions_(std::move(occlusions))
{
  const std::string name = "Agent " + std::to_string(id_);
  if (waypoints_.size() < 2) {
    throw std::invalid_argument(name + " needs at least two waypoints.");
  }
  for (std::size_t i = 1; i < waypoints_.size(); ++i) {
    if (!(waypoints_[i].time > waypoints_[i - 1].time)) {
      throw std::invalid_argument(name + " waypoint times must strictly increase.");
    }
  }
  for (const auto & occlusion : occlusions_) {
    if (!(occlusion.end > occlusion.start)) {
      throw std::invalid_argument(name + " occlusion end must be after its start.");
    }
  }

  // Standing still keeps the heading of the previous (or, before any movement, the next) segment.
  segment_headings_.assign(waypoints_.size() - 1, 0.0);
  std::optional<double> heading;
  for (std::size_t i = 0; i < segment_headings_.size(); ++i) {
    const Eigen::Vector2f direction = waypoints_[i + 1].position - waypoints_[i].position;
    if (direction.norm() > 1e-6F) {
      const double moving_heading = std::atan2(direction.y(), direction.x());
      if (!heading) {
        std::fill_n(segment_headings_.begin(), i, moving_heading);
      }
      heading = moving_heading;
    }
    segment_headings_[i] = heading.value_or(0.0);
  }
}

std::optional<AgentState> Agent::state(const double time) const
{
  if (time < waypoints_.front().time - kTimeTolerance ||
    time > waypoints_.back().time + kTimeTolerance)
  {
    return std::nullopt;
  }

  // Find the segment containing time; the last segment also covers the end time.
  std::size_t i = 0;
  while (i + 2 < waypoints_.size() && time >= waypoints_[i + 1].time) {
    ++i;
  }
  const auto & from = waypoints_[i];
  const auto & to = waypoints_[i + 1];
  const double factor = std::clamp((time - from.time) / (to.time - from.time), 0.0, 1.0);

  AgentState state;
  state.position = from.position + static_cast<float>(factor) * (to.position - from.position);
  state.dimension = dimension_;
  state.orientation = orientation_.value_or(segment_headings_[i]);
  return state;
}

bool Agent::visible(const double time) const
{
  return std::none_of(
    occlusions_.begin(), occlusions_.end(), [time](const Occlusion & occlusion) {
      return time >= occlusion.start && time < occlusion.end;
    });
}

std::int64_t Agent::id() const
{
  return id_;
}

}  // namespace ufil_examples_pedestrian_simulator
