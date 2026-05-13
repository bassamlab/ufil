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


#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Eigen>

#include <memory>
#include <vector>

#include "ufil_examples_pedestrian_simulator/visibility_control.h"

namespace ufil_examples_pedestrian_simulator
{

struct AgentState
{
  Eigen::Vector3f dimension = Eigen::Vector3f::Zero();
  Eigen::Vector2f position = Eigen::Vector2f::Zero();
  double orientation = 0.0;
};

class Agent
{
public:
  using SharedPtr = std::shared_ptr<ufil_examples_pedestrian_simulator::Agent>;
  using UniquePtr = std::unique_ptr<ufil_examples_pedestrian_simulator::Agent>;

  Agent();

  virtual ~Agent();

  void step(AgentState & state);
  void generateTrajectory(const double timestep);
  void load(const YAML::Node & node);

  int64_t id();

private:
  int64_t id_ = -1;
  Eigen::Vector3f dimension_ = Eigen::Vector3f::Zero();
  double orientation_ = 0;

  Eigen::Vector2f start_position_ = Eigen::Vector2f::Zero();
  double start_time_ = 0;

  Eigen::Vector2f stop_position_ = Eigen::Vector2f::Zero();
  double stop_time_ = 0;

  std::vector<Eigen::Vector2f> trajectory_{};
  std::vector<Eigen::Vector2f>::iterator trajectory_iter_;
};

Agent agentFromYaml(const YAML::Node & node);

}  // namespace ufil_examples_pedestrian_simulator

#endif  // UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__AGENT_HPP_
