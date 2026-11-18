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

#include "ufil_examples_pedestrian_simulator/scenario.hpp"

#include <yaml-cpp/yaml.h>

#include <numbers>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ufil_examples_pedestrian_simulator
{

namespace
{

Waypoint waypointFromYaml(const YAML::Node & node)
{
  Waypoint waypoint;
  waypoint.position << node["position"][0].as<float>(), node["position"][1].as<float>();
  waypoint.time = node["time"].as<double>();
  return waypoint;
}

Agent agentFromYaml(const YAML::Node & node)
{
  const auto id = node["id"].as<std::int64_t>();
  const Eigen::Vector3f dimension(
    node["dimension"]["length"].as<float>(), node["dimension"]["width"].as<float>(),
    node["dimension"]["height"].as<float>());

  std::optional<double> orientation;
  if (node["orientation"]) {
    orientation = node["orientation"].as<double>() * (std::numbers::pi / 180.0);
  }

  std::vector<Waypoint> waypoints;
  if (node["waypoints"]) {
    for (const auto & waypoint : node["waypoints"]) {
      waypoints.push_back(waypointFromYaml(waypoint));
    }
  } else {
    waypoints.push_back(waypointFromYaml(node["starting"]));
    waypoints.push_back(waypointFromYaml(node["stopping"]));
  }

  std::vector<Occlusion> occlusions;
  if (node["occlusions"]) {
    for (const auto & occlusion : node["occlusions"]) {
      occlusions.push_back({occlusion["start"].as<double>(), occlusion["end"].as<double>()});
    }
  }
  return Agent(id, dimension, std::move(waypoints), orientation, std::move(occlusions));
}

ScenarioDescription scenarioFromYaml(const YAML::Node & node)
{
  ScenarioDescription scenario;
  scenario.name = node["name"].as<std::string>();
  scenario.runtime = node["runtime"].as<double>();
  scenario.position_noise = node["error_covariance"].as<double>();
  if (node["seed"]) {
    scenario.seed = node["seed"].as<unsigned int>();
  }
  scenario.detection_probability = node["detection_probability"].as<double>(1.0);

  if (!(scenario.runtime > 0.0)) {
    throw ScenarioError("runtime must be positive.");
  }
  if (!(scenario.position_noise >= 0.0)) {
    throw ScenarioError("error_covariance must not be negative.");
  }
  if (!(scenario.detection_probability >= 0.0 && scenario.detection_probability <= 1.0)) {
    throw ScenarioError("detection_probability must be within [0, 1].");
  }

  if (const auto clutter = node["clutter"]) {
    scenario.clutter.mean_count = clutter["mean_count"].as<double>();
    const auto area = clutter["area"].as<std::vector<double>>();
    if (!(scenario.clutter.mean_count >= 0.0) || area.size() != 4 || !(area[2] > area[0]) ||
      !(area[3] > area[1]))
    {
      throw ScenarioError(
              "clutter needs a non-negative mean_count and an area [x_min, y_min, x_max, y_max].");
    }
    std::copy(area.begin(), area.end(), scenario.clutter.area.begin());
  }

  for (const auto & agent : node["agents"]) {
    scenario.agents.push_back(agentFromYaml(agent));
  }
  for (std::size_t i = 0; i < scenario.agents.size(); ++i) {
    for (std::size_t j = i + 1; j < scenario.agents.size(); ++j) {
      if (scenario.agents[i].id() == scenario.agents[j].id()) {
        throw ScenarioError("agent id " + std::to_string(scenario.agents[i].id()) +
              " is not unique.");
      }
    }
  }
  return scenario;
}

}  // namespace

ScenarioDescription parseScenario(const std::string & yaml)
{
  try {
    return scenarioFromYaml(YAML::Load(yaml));
  } catch (const YAML::Exception & error) {
    throw ScenarioError(error.what());
  } catch (const std::invalid_argument & error) {
    throw ScenarioError(error.what());
  }
}

ScenarioDescription loadScenarioFile(const std::string & path)
{
  try {
    return scenarioFromYaml(YAML::LoadFile(path));
  } catch (const YAML::BadFile &) {
    throw ScenarioError("cannot read scenario file '" + path + "'.");
  } catch (const YAML::Exception & error) {
    throw ScenarioError("invalid scenario file '" + path + "': " + error.what());
  } catch (const std::exception & error) {
    throw ScenarioError("invalid scenario file '" + path + "': " + error.what());
  }
}

}  // namespace ufil_examples_pedestrian_simulator
