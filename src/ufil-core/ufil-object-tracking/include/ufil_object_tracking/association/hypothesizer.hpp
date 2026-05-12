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

#ifndef UFIL_OBJECT_TRACKING__ASSOCIATION__HYPOTHESIZER_HPP_
#define UFIL_OBJECT_TRACKING__ASSOCIATION__HYPOTHESIZER_HPP_

#include <map>
#include <utility>

#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/scalar.hpp"

namespace ufil
{
namespace association
{

template<class T>
class Hypothesizer
{
protected:
  using MeasurementType = T::MeasurementType;

public:
  virtual ~Hypothesizer() = default;

  virtual void hypothesize(
    const std::map<type::Id, T> & tracks, const std::map<type::Id, MeasurementType> & measurements,
    type::CostMatrix & hypothesis) = 0;
};

template<typename T>
class FunctionHypothesizer : public Hypothesizer<T>
{
private:
  using MeasurementType = typename T::MeasurementType;

  // Lambda function to calculate the distance between a track and a measurement
  std::function<void(const T &, const MeasurementType &, type::Scalar &)> distance_function_;

public:
  explicit FunctionHypothesizer(
    std::function<void(const T &, const MeasurementType &,
    type::Scalar &)> distance_function)
  : distance_function_(std::move(distance_function))
  {
  }

  void hypothesize(
    const std::map<type::Id, T> & tracks, const std::map<type::Id, MeasurementType> & measurements,
    type::CostMatrix & hypothesis)
  {
    // Resize cost matrix
    hypothesis.resize(tracks.size(), measurements.size());

    Eigen::Index trackIndex = 0;
    Eigen::Index measurementIndex = 0;
    for (const auto & [track_uuid, track] : tracks) {
      for (const auto & [measurement_uuid, measurement] : measurements) {
        type::Scalar distance = 0.0;
        this->distance_function_(track, measurement, distance);
        hypothesis(trackIndex, measurementIndex) = distance;
        measurementIndex++;
      }
      measurementIndex = 0;
      trackIndex++;
    }
  }
};

}  // namespace association
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__ASSOCIATION__HYPOTHESIZER_HPP_
