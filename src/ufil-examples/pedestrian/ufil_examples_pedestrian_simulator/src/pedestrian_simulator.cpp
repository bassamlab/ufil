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

#include "ufil_examples_pedestrian_simulator/pedestrian_simulator.hpp"

#include <numbers>
#include <stdexcept>
#include <utility>

namespace ufil_examples_pedestrian_simulator
{

namespace
{
constexpr double kTimeTolerance = 1e-9;
/// The measured orientation is disturbed much less than the position.
constexpr double kOrientationNoiseFactor = 0.001;
/// Size of a false detection [m].
constexpr float kClutterSize = 1.0F;
}  // namespace

PedestrianSimulator::PedestrianSimulator(ScenarioDescription scenario, const double timestep)
: scenario_(std::move(scenario)), timestep_(timestep), noise_(0.0, scenario_.position_noise)
{
  if (!(timestep_ > 0.0)) {
    throw std::invalid_argument("the timestep must be positive.");
  }
  if (scenario_.seed) {
    generator_.seed(*scenario_.seed);
  }
  status_.scenario = scenario_.name;
  status_.runtime = scenario_.runtime;
}

std::optional<SimulationStep> PedestrianSimulator::step()
{
  // An integer step counter avoids accumulating floating point error over long runs.
  const double time = static_cast<double>(status_.steps) * timestep_;
  if (time > scenario_.runtime + kTimeTolerance) {
    status_.finished = true;
    return std::nullopt;
  }
  ++status_.steps;
  status_.time = time;

  SimulationStep result;
  result.time = time;
  std::size_t detected_agents = 0;
  for (const auto & agent : scenario_.agents) {
    const auto ground_truth = agent.state(time);
    if (!ground_truth) {
      continue;
    }

    SimulatedObject object;
    object.id = agent.id();
    object.ground_truth = *ground_truth;
    object.measured = *ground_truth;
    object.measured.position.x() += static_cast<float>(noise_(generator_));
    object.measured.position.y() += static_cast<float>(noise_(generator_));
    object.measured.orientation += noise_(generator_) * kOrientationNoiseFactor;
    object.detected =
      agent.visible(time) && uniform_(generator_) < scenario_.detection_probability;
    detected_agents += object.detected ? 1 : 0;
    result.objects.push_back(object);
  }
  status_.agents_in_scene = result.objects.size();
  status_.detected_agents = detected_agents;

  status_.clutter = 0;
  if (scenario_.clutter.mean_count > 0.0) {
    const auto & area = scenario_.clutter.area;
    std::poisson_distribution<int> count(scenario_.clutter.mean_count);
    for (int i = count(generator_); i > 0; --i) {
      SimulatedObject clutter;
      clutter.id = next_clutter_id_++;
      clutter.has_ground_truth = false;
      clutter.measured.position <<
        static_cast<float>(area[0] + uniform_(generator_) * (area[2] - area[0])),
        static_cast<float>(area[1] + uniform_(generator_) * (area[3] - area[1]));
      clutter.measured.dimension.setConstant(kClutterSize);
      clutter.measured.orientation = (uniform_(generator_) * 2.0 - 1.0) * std::numbers::pi;
      result.objects.push_back(clutter);
      ++status_.clutter;
    }
  }
  return result;
}

const ScenarioDescription & PedestrianSimulator::scenario() const
{
  return scenario_;
}

const SimulatorStatus & PedestrianSimulator::status() const
{
  return status_;
}

}  // namespace ufil_examples_pedestrian_simulator
