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


#include "ufil_examples_pedestrian_simulator/agent.hpp"

#include <iostream>

namespace ufil_examples_pedestrian_simulator
{

Agent::Agent()
{
}

Agent::~Agent()
{
}

void Agent::step(AgentState & state)
{
  if (this->trajectory_iter_ == this->trajectory_.end()) {
    state.position = this->trajectory_.back();
  } else {
    state.position = *this->trajectory_iter_;
    this->trajectory_iter_++;
  }
  state.dimension = this->dimension_;
  state.orientation = this->orientation_;
}

void Agent::generateTrajectory(const double timestep)
{
  double time = this->start_time_;
  // std::cout << this->id_ << std::endl;

  while (time < this->stop_time_) {
    double time_factor = (time - this->start_time_) / (this->stop_time_ - this->start_time_);
    Eigen::Vector2f position = this->start_position_ + time_factor *
      (this->stop_position_ - this->start_position_);
    this->trajectory_.push_back(position);
    // std::cout << position << std::endl;

    time += timestep;
  }
  this->trajectory_iter_ = this->trajectory_.begin();
}

void Agent::load(const YAML::Node & node)
{
  this->id_ = node["id"].as<int64_t>();
  this->dimension_ << node["dimension"]["length"].as<double>(),
    node["dimension"]["width"].as<double>(),
    node["dimension"]["height"].as<double>();
  this->orientation_ = node["orientation"].as<double>() * (M_PI / 180.0);

  this->start_time_ = node["starting"]["time"].as<double>();
  this->start_position_ << node["starting"]["position"][0].as<double>(),
    node["starting"]["position"][1].as<double>();
  this->stop_time_ = node["stopping"]["time"].as<double>();
  this->stop_position_ << node["stopping"]["position"][0].as<double>(),
    node["stopping"]["position"][1].as<double>();
  // std::cout << this->id_ << std::endl;
  // std::cout << this->start_position_ << std::endl;
  // std::cout << this->stop_position_ << std::endl;
}

int64_t Agent::id()
{
  return this->id_;
}

Agent agentFromYaml(const YAML::Node & node)
{
  Agent agent;
  agent.load(node);
  return agent;
}

}  // namespace ufil_examples_pedestrian_simulator
