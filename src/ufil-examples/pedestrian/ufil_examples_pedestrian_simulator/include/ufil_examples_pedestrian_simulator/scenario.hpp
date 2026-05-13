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

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>

#include <memory>
#include <vector>
#include <random>
#include <string>
#include <map>

#include "ufil_examples_pedestrian_simulator/visibility_control.h"
#include "ufil_examples_pedestrian_simulator/agent.hpp"

namespace ufil_examples_pedestrian_simulator
{

struct ScenarioState
{
  AgentState ground_truth;
  AgentState measured;
};

class Scenario
{
public:
  using SharedPtr = std::shared_ptr<ufil_examples_pedestrian_simulator::Scenario>;
  using UniquePtr = std::unique_ptr<ufil_examples_pedestrian_simulator::Scenario>;

  Scenario();

  virtual ~Scenario();

  bool step(std::map<int64_t, ScenarioState> & ids_with_states);
  void generateTrajectories(const double timestep);
  void load(const YAML::Node & node);

private:
  std::string name_ = "";
  double runtime_ = 0.0;
  std::vector<Agent> agents_;

  double time_ = 0.0;
  double timestep_ = 0.0;

  double error_covariance_ = 0.0;
  std::default_random_engine generator;
  std::normal_distribution<double> distribution;
};

Scenario scenarioFromYaml(const YAML::Node & node);

}  // namespace ufil_examples_pedestrian_simulator

#endif  // UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__SCENARIO_HPP_
