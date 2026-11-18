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

#ifndef UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__PEDESTRIAN_SIMULATOR_HPP_
#define UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__PEDESTRIAN_SIMULATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "ufil_examples_pedestrian_simulator/agent.hpp"
#include "ufil_examples_pedestrian_simulator/scenario.hpp"
#include "ufil_examples_pedestrian_simulator/visibility_control.hpp"

namespace ufil_examples_pedestrian_simulator
{

/// An agent or a false detection in one simulation step.
struct SimulatedObject
{
  std::int64_t id = 0;
  /// Only valid if has_ground_truth.
  AgentState ground_truth;
  /// Only valid if detected.
  AgentState measured;
  /// False for clutter (false detections).
  bool has_ground_truth = true;
  /// False for missed detections (occlusion or detection probability).
  bool detected = true;
};

/// Result of one simulation step.
struct SimulationStep
{
  /// Scenario time of the step [s].
  double time = 0.0;
  std::vector<SimulatedObject> objects;
};

/// Progress of the simulation, e.g. for diagnostics.
struct SimulatorStatus
{
  std::string scenario;
  /// Scenario time of the last step [s].
  double time = 0.0;
  double runtime = 0.0;
  bool finished = false;
  std::uint64_t steps = 0;
  /// Agents in the scene, detected agents and false detections in the last step.
  std::size_t agents_in_scene = 0;
  std::size_t detected_agents = 0;
  std::size_t clutter = 0;
};

/**
 * @brief Simulates a scenario step by step.
 *
 * Every step yields the ground truth of all agents in the scene and noisy measurements of the
 * detected ones, plus clutter. The scenario time advances by the timestep per step.
 */
class UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR_PUBLIC PedestrianSimulator
{
public:
  /// @throws std::invalid_argument if the timestep is not positive.
  PedestrianSimulator(ScenarioDescription scenario, double timestep);

  /// The next step, or nothing once the runtime of the scenario is exceeded.
  std::optional<SimulationStep> step();

  const ScenarioDescription & scenario() const;
  const SimulatorStatus & status() const;

private:
  /// Ids of false detections start here to avoid clashes with agent ids.
  static constexpr std::int64_t kFirstClutterId = 1000000;

  ScenarioDescription scenario_;
  double timestep_;
  SimulatorStatus status_;
  std::int64_t next_clutter_id_ = kFirstClutterId;

  std::default_random_engine generator_;
  std::normal_distribution<double> noise_;
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
};

}  // namespace ufil_examples_pedestrian_simulator

#endif  // UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__PEDESTRIAN_SIMULATOR_HPP_
