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


#include "ufil_examples_pedestrian_simulator/ufil_examples_pedestrian_simulator.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>

namespace ufil_examples_pedestrian_simulator
{

PedestrianSimulator::PedestrianSimulator()
{
}

PedestrianSimulator::~PedestrianSimulator()
{
}

bool PedestrianSimulator::step(std::map<int64_t, ScenarioState> & ids_with_states)
{
  return this->scenario_.step(ids_with_states);
}

void PedestrianSimulator::generateTrajectories(const double timestep)
{
  this->scenario_.generateTrajectories(timestep);
}

bool PedestrianSimulator::loadScenarioFile(std::string file_path)
{
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    this->scenario_ = scenarioFromYaml(config);
    this->initialized_ = true;
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
}

bool PedestrianSimulator::initialized() const
{
  return this->initialized_;
}

}  // namespace ufil_examples_pedestrian_simulator
