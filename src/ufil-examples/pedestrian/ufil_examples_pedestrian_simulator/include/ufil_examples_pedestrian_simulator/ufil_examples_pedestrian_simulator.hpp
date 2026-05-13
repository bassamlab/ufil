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


#ifndef UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR_HPP_
#define UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR_HPP_

#include <memory>
#include <filesystem>
#include <string>
#include <map>

#include "ufil_examples_pedestrian_simulator/visibility_control.h"
#include "ufil_examples_pedestrian_simulator/scenario.hpp"

namespace ufil_examples_pedestrian_simulator
{

class PedestrianSimulator
{
public:
  using SharedPtr = std::shared_ptr<ufil_examples_pedestrian_simulator::PedestrianSimulator>;
  using UniquePtr = std::unique_ptr<ufil_examples_pedestrian_simulator::PedestrianSimulator>;

  PedestrianSimulator();

  virtual ~PedestrianSimulator();

  void generateTrajectories(const double timestep);

  bool step(std::map<int64_t, ScenarioState> & ids_with_states);
  bool loadScenarioFile(std::string file_path);
  bool initialized() const;

private:
  bool initialized_ = false;
  Scenario scenario_;
};

}  // namespace ufil_examples_pedestrian_simulator

#endif  // UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR__UFIL_EXAMPLES_PEDESTRIAN_SIMULATOR_HPP_
