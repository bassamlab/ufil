// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWHT Aachen University
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

#ifndef UFIL_OSN_TRACKER__EXISTENCE_PROBABILITY_HPP_
#define UFIL_OSN_TRACKER__EXISTENCE_PROBABILITY_HPP_

#include <ufil_object_tracking/types/existence_probability.hpp>

namespace ufil_osn_tracker
{

class ExistenceProbability : public ufil::type::probability::ExistenceProbabilityBasicBelief
{
public:
  using Scalar = ufil::type::Scalar;

  ExistenceProbability()
  : ufil::type::probability::ExistenceProbabilityBasicBelief()
  {
    existence() = 1.0f;
    nonExistence() = 0.0f;
    uncertainty() = 0.0f;
  }

  // detection probability
  Scalar & pDetection() {return p_detection_;}
  const Scalar & pDetection() const {return p_detection_;}

  // clutter probability
  Scalar & pClutter() {return p_clutter_;}
  const Scalar & pClutter() const {return p_clutter_;}

private:
  Scalar p_detection_{0.5};
  Scalar p_clutter_{0.2};
};

}  // namespace ufil_osn_tracker
#endif  // UFIL_OSN_TRACKER__EXISTENCE_PROBABILITY_HPP_
