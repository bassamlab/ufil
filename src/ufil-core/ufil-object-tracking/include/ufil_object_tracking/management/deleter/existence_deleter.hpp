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

#ifndef UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__EXISTENCE_DELETER_HPP_
#define UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__EXISTENCE_DELETER_HPP_

#include "ufil_object_tracking/management/deleter/deleter.hpp"
#include "ufil_object_tracking/types/time.hpp"
#include "ufil_object_tracking/types/track.hpp"

namespace ufil
{
namespace management
{
template<typename T>
class ExistenceDeleter : public Deleter<T>
{
public:
  using TrackType = T;
  using ExistenceProbabilityType = TrackType::ExistenceProbabilityType;

  explicit ExistenceDeleter(const ufil::type::Scalar threshold = 0.15)
  : threshold_(threshold)
  {
  }

  void setThreshold(const ufil::type::Scalar threshold)
  {
    this->threshold_ = threshold;
  }

  bool checkForDeletion(const type::Timestamp & /*timestamp*/, TrackType & track) override
  {
    ExistenceProbabilityType existence_probability_ = track.currentExistenceProbability();
    return existence_probability_.existence() < this->threshold_;
  }

private:
  ufil::type::Scalar threshold_;
};

}  // namespace management
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__EXISTENCE_DELETER_HPP_
