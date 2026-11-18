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

#include <stdexcept>
#include <utility>

#include "ufil_examples_pedestrian_simulator/pedestrian_simulator.hpp"

using ufil_examples_pedestrian_simulator::Agent;
using ufil_examples_pedestrian_simulator::PedestrianSimulator;
using ufil_examples_pedestrian_simulator::ScenarioDescription;
using ufil_examples_pedestrian_simulator::Waypoint;

namespace
{

/// One agent walking from (0, 0) at 1 s to (10, 0) at 3 s; the scenario runs 4 s.
ScenarioDescription scenario()
{
  ScenarioDescription description;
  description.name = "unit";
  description.runtime = 4.0;
  description.position_noise = 0.0;
  description.seed = 1;
  description.agents.emplace_back(
    3, Eigen::Vector3f(1.0F, 1.0F, 1.0F),
    std::vector<Waypoint>{{Eigen::Vector2f(0, 0), 1.0}, {Eigen::Vector2f(10, 0), 3.0}});
  return description;
}

}  // namespace

TEST(PedestrianSimulatorTest, StepsUntilTheRuntimeIsReached)
{
  PedestrianSimulator simulator(scenario(), 0.5);
  EXPECT_EQ(simulator.status().scenario, "unit");
  EXPECT_FALSE(simulator.status().finished);

  std::size_t steps = 0;
  std::size_t steps_with_agent = 0;
  while (const auto step = simulator.step()) {
    EXPECT_DOUBLE_EQ(step->time, 0.5 * static_cast<double>(steps));
    ++steps;
    if (!step->objects.empty()) {
      ++steps_with_agent;
      ASSERT_EQ(step->objects.size(), 1u);
      const auto & object = step->objects.front();
      EXPECT_EQ(object.id, 3);
      EXPECT_TRUE(object.has_ground_truth);
      EXPECT_TRUE(object.detected);
      // Without noise the measurement equals the ground truth.
      EXPECT_EQ(object.measured.position, object.ground_truth.position);
    }
  }
  // 0, 0.5, ..., 4.0 s; the agent exists from 1 s to 3 s.
  EXPECT_EQ(steps, 9u);
  EXPECT_EQ(steps_with_agent, 5u);
  EXPECT_TRUE(simulator.status().finished);
  EXPECT_EQ(simulator.status().steps, 9u);
  EXPECT_FALSE(simulator.step());
}

TEST(PedestrianSimulatorTest, NoiseIsReproducibleWithASeed)
{
  auto noisy = scenario();
  noisy.position_noise = 0.5;
  PedestrianSimulator first(noisy, 0.5);
  PedestrianSimulator second(noisy, 0.5);
  bool differs_from_truth = false;
  while (const auto a = first.step()) {
    const auto b = second.step();
    ASSERT_TRUE(b);
    ASSERT_EQ(a->objects.size(), b->objects.size());
    for (std::size_t i = 0; i < a->objects.size(); ++i) {
      EXPECT_EQ(a->objects[i].measured.position, b->objects[i].measured.position);
      differs_from_truth |= a->objects[i].measured.position != a->objects[i].ground_truth.position;
    }
  }
  EXPECT_TRUE(differs_from_truth);
}

TEST(PedestrianSimulatorTest, MissedDetectionsAndClutter)
{
  auto description = scenario();
  description.detection_probability = 0.0;
  description.clutter.mean_count = 3.0;
  description.clutter.area = {10.0, 20.0, 11.0, 21.0};
  PedestrianSimulator simulator(description, 0.5);

  std::size_t clutter = 0;
  while (const auto step = simulator.step()) {
    for (const auto & object : step->objects) {
      if (object.has_ground_truth) {
        EXPECT_FALSE(object.detected);
      } else {
        ++clutter;
        EXPECT_TRUE(object.detected);
        EXPECT_GE(object.id, 1000000);
        EXPECT_GE(object.measured.position.x(), 10.0F);
        EXPECT_LE(object.measured.position.y(), 21.0F);
      }
    }
  }
  EXPECT_GT(clutter, 0u);
}

TEST(PedestrianSimulatorTest, RejectsInvalidTimestep)
{
  EXPECT_THROW(PedestrianSimulator(scenario(), 0.0), std::invalid_argument);
}
