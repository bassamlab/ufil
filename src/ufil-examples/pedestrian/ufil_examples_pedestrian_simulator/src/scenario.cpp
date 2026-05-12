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

#include <iostream>

namespace ufil_examples_pedestrian_simulator
{

Scenario::Scenario()
{
}

Scenario::~Scenario()
{
}

bool Scenario::step(std::map<int64_t, ScenarioState> & ids_with_states)
{
  this->time_ += this->timestep_;
  if (this->time_ > this->runtime_) {
    return false;
  }

  // std::cout << time_ << " " << timestep_ << std::endl;

  for (auto & agent : this->agents_) {
    ScenarioState state;

    agent.step(state.ground_truth);
    state.measured = state.ground_truth;

    double error = distribution(generator);
    state.measured.position.x() += error;
    error = distribution(generator);
    state.measured.position.y() += error;
    error = distribution(generator) * 0.001;
    state.measured.orientation += error;
    ids_with_states.insert({agent.id(), state});
  }
  return true;
}

void Scenario::generateTrajectories(const double timestep)
{
  this->timestep_ = timestep;
  for (auto & agent : this->agents_) {
    agent.generateTrajectory(timestep);
  }
}

void Scenario::load(const YAML::Node & node)
{
  this->name_ = node["name"].as<std::string>();
  this->runtime_ = node["runtime"].as<double>();
  this->error_covariance_ = node["error_covariance"].as<double>();
  this->distribution = std::normal_distribution<double>(0.0, this->error_covariance_);
  for (const auto & agent_node : node["agents"]) {
    Agent agent = agentFromYaml(agent_node);
    this->agents_.push_back(agent);
  }
}

Scenario scenarioFromYaml(const YAML::Node & node)
{
  Scenario scenario;
  scenario.load(node);
  return scenario;
}

}  // namespace ufil_examples_pedestrian_simulator
