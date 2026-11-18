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

#ifndef UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__SCENARIO_HPP_
#define UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__SCENARIO_HPP_

#include <array>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ufil_examples_pedestrian_simulator/agent.hpp"
#include "ufil_examples_pedestrian_simulator/visibility_control.hpp"

namespace ufil_examples_pedestrian_simulator
{

/// False detections, uniformly distributed over an area.
struct Clutter
{
  /// Mean number of false detections per simulation step (Poisson distributed).
  double mean_count = 0.0;
  /// x_min, y_min, x_max, y_max [m].
  std::array<double, 4> area{};
};

/**
 * @brief Description of a scenario, as read from a scenario file.
 *
 * Scenario files are YAML files with the keys name, runtime [s], error_covariance (standard
 * deviation of the position noise [m]), optional seed, optional detection_probability, optional
 * clutter (mean_count, area) and agents. Each agent has an id, a dimension (length, width,
 * height), waypoints (position [x, y], time) or starting/stopping waypoints, an optional fixed
 * orientation [deg] and optional occlusions (start, end).
 */
struct ScenarioDescription
{
  std::string name;
  /// The scenario ends after this scenario time [s].
  double runtime = 0.0;
  /// Standard deviation of the measured position noise [m].
  double position_noise = 0.0;
  /// Seed of the noise; without one the default seed of the random engine is used.
  std::optional<unsigned int> seed;
  /// Probability that a visible agent is detected in a step.
  double detection_probability = 1.0;
  Clutter clutter;
  std::vector<Agent> agents;
};

/// A scenario could not be read or is invalid.
class UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR_PUBLIC ScenarioError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// @throws ScenarioError if the YAML text is no valid scenario.
UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR_PUBLIC
ScenarioDescription parseScenario(const std::string & yaml);

/// @throws ScenarioError if the file cannot be read or is no valid scenario.
UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR_PUBLIC
ScenarioDescription loadScenarioFile(const std::string & path);

}  // namespace ufil_examples_pedestrian_simulator

#endif  // UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__SCENARIO_HPP_
