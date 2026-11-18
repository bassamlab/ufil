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

#include <filesystem>
#include <string>

#include "ufil_examples_pedestrian_simulator/scenario.hpp"

using ufil_examples_pedestrian_simulator::loadScenarioFile;
using ufil_examples_pedestrian_simulator::parseScenario;
using ufil_examples_pedestrian_simulator::ScenarioError;

namespace
{

const char kScenario[] =
  R"(
name: test
runtime: 10
error_covariance: 0.2
seed: 4
detection_probability: 0.9
clutter:
  mean_count: 1.5
  area: [0.0, 0.0, 40.0, 20.0]
agents:
  - id: 1
    dimension: {length: 1.0, width: 0.5, height: 1.8}
    orientation: 90
    waypoints:
      - {position: [0.0, 0.0], time: 0}
      - {position: [10.0, 0.0], time: 10}
    occlusions:
      - {start: 2, end: 3}
  - id: 2
    dimension: {length: 1.0, width: 1.0, height: 1.0}
    starting: {position: [5.0, 5.0], time: 1}
    stopping: {position: [5.0, 15.0], time: 6}
)";

std::string replace(std::string text, const std::string & from, const std::string & to)
{
  return text.replace(text.find(from), from.size(), to);
}

}  // namespace

TEST(ScenarioTest, ParsesAllKeys)
{
  const auto scenario = parseScenario(kScenario);
  EXPECT_EQ(scenario.name, "test");
  EXPECT_DOUBLE_EQ(scenario.runtime, 10.0);
  EXPECT_DOUBLE_EQ(scenario.position_noise, 0.2);
  ASSERT_TRUE(scenario.seed);
  EXPECT_EQ(*scenario.seed, 4u);
  EXPECT_DOUBLE_EQ(scenario.detection_probability, 0.9);
  EXPECT_DOUBLE_EQ(scenario.clutter.mean_count, 1.5);
  EXPECT_DOUBLE_EQ(scenario.clutter.area[3], 20.0);

  ASSERT_EQ(scenario.agents.size(), 2u);
  const auto first = scenario.agents[0].state(5.0);
  ASSERT_TRUE(first);
  EXPECT_FLOAT_EQ(first->position.x(), 5.0F);
  EXPECT_NEAR(first->orientation, 1.5707963, 1e-6);
  EXPECT_FLOAT_EQ(first->dimension.y(), 0.5F);
  EXPECT_FALSE(scenario.agents[0].visible(2.5));

  EXPECT_EQ(scenario.agents[1].id(), 2);
  EXPECT_FALSE(scenario.agents[1].state(0.5));
  EXPECT_FLOAT_EQ(scenario.agents[1].state(6.0)->position.y(), 15.0F);
}

TEST(ScenarioTest, OptionalKeysHaveDefaults)
{
  const auto scenario = parseScenario(
    "name: minimal\nruntime: 1\nerror_covariance: 0.0\nagents: []\n");
  EXPECT_FALSE(scenario.seed);
  EXPECT_DOUBLE_EQ(scenario.detection_probability, 1.0);
  EXPECT_DOUBLE_EQ(scenario.clutter.mean_count, 0.0);
  EXPECT_TRUE(scenario.agents.empty());
}

TEST(ScenarioTest, RejectsInvalidScenarios)
{
  EXPECT_THROW(parseScenario("name: [unterminated"), ScenarioError);
  EXPECT_THROW(parseScenario(replace(kScenario, "runtime: 10", "")), ScenarioError);
  EXPECT_THROW(parseScenario(replace(kScenario, "runtime: 10", "runtime: 0")), ScenarioError);
  EXPECT_THROW(
    parseScenario(replace(kScenario, "error_covariance: 0.2", "error_covariance: -1")),
    ScenarioError);
  EXPECT_THROW(
    parseScenario(replace(kScenario, "detection_probability: 0.9", "detection_probability: 2")),
    ScenarioError);
  EXPECT_THROW(
    parseScenario(replace(kScenario, "[0.0, 0.0, 40.0, 20.0]", "[0.0, 0.0, 40.0]")),
    ScenarioError);
  EXPECT_THROW(parseScenario(replace(kScenario, "id: 2", "id: 1")), ScenarioError);
  EXPECT_THROW(parseScenario(replace(kScenario, "time: 10", "time: 0")), ScenarioError);
  EXPECT_THROW(parseScenario(replace(kScenario, "end: 3", "end: 1")), ScenarioError);
}

TEST(ScenarioTest, MissingFileThrows)
{
  EXPECT_THROW(loadScenarioFile("/nonexistent/scenario.yaml"), ScenarioError);
}

TEST(ScenarioTest, LoadsAllShippedScenarios)
{
  std::size_t count = 0;
  for (const auto & entry : std::filesystem::directory_iterator(SCENARIO_DIRECTORY)) {
    if (entry.path().filename() == "default.yaml") {
      continue;  // Node parameters, not a scenario.
    }
    SCOPED_TRACE(entry.path().string());
    const auto scenario = loadScenarioFile(entry.path().string());
    EXPECT_EQ(scenario.name + ".yaml", entry.path().filename().string());
    EXPECT_FALSE(scenario.agents.empty());
    ++count;
  }
  EXPECT_EQ(count, 10u);
}
