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

#ifndef UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__AGENT_HPP_
#define UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__AGENT_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <vector>

#include "ufil_examples_pedestrian_simulator/visibility_control.hpp"

namespace ufil_examples_pedestrian_simulator
{

/// State of a simulated pedestrian in the scenario frame.
struct AgentState
{
  /// Length, width and height [m].
  Eigen::Vector3f dimension = Eigen::Vector3f::Zero();
  /// Position [m].
  Eigen::Vector2f position = Eigen::Vector2f::Zero();
  /// Heading [rad].
  double orientation = 0.0;
};

/// The agent is at position at the given scenario time.
struct Waypoint
{
  Eigen::Vector2f position = Eigen::Vector2f::Zero();
  /// Scenario time [s].
  double time = 0.0;
};

/// The agent is not detected in the scenario time interval [start, end) [s].
struct Occlusion
{
  double start = 0.0;
  double end = 0.0;
};

/**
 * @brief A pedestrian that walks along a piecewise linear trajectory.
 *
 * The agent exists from its first to its last waypoint and moves with constant velocity between
 * two waypoints.
 */
class UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR_PUBLIC Agent
{
public:
  /**
   * @param id Unique identifier of the agent.
   * @param dimension Length, width and height [m].
   * @param waypoints At least two waypoints with strictly increasing times.
   * @param orientation Fixed heading [rad]; without one the agent faces its walking direction.
   * @param occlusions Intervals in which the agent is not detected.
   * @throws std::invalid_argument if the waypoints or occlusions are invalid.
   */
  Agent(
    std::int64_t id, const Eigen::Vector3f & dimension, std::vector<Waypoint> waypoints,
    std::optional<double> orientation = std::nullopt, std::vector<Occlusion> occlusions = {});

  /// Ground truth at the given scenario time; empty before the first or after the last waypoint.
  std::optional<AgentState> state(double time) const;
  /// False while the agent is inside one of its occlusion intervals.
  bool visible(double time) const;
  std::int64_t id() const;

private:
  std::int64_t id_;
  Eigen::Vector3f dimension_;
  std::optional<double> orientation_;
  std::vector<Waypoint> waypoints_;
  /// Heading of each segment between two waypoints [rad].
  std::vector<double> segment_headings_;
  std::vector<Occlusion> occlusions_;
};

}  // namespace ufil_examples_pedestrian_simulator

#endif  // UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__AGENT_HPP_
