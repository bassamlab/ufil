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

#ifndef UFIL_OBJECT_TRACKING__PREDICT__CLASSIFICATION__CLASSIFICATION_PREDICTOR_HPP_
#define UFIL_OBJECT_TRACKING__PREDICT__CLASSIFICATION__CLASSIFICATION_PREDICTOR_HPP_

#include <optional>

#include "ufil_object_tracking/predict/predictor.hpp"

namespace ufil
{
namespace predict
{

template<typename T>
class ClassificationPredictor : public Predictor<T>
{
protected:
  using TrackType = typename Predictor<T>::TrackType;
  using ControlType = typename Predictor<T>::ControlType;
  using ClassificationType = typename Predictor<T>::ClassificationType;

public:
  using Predictor<T>::predict;

  void predict(
    TrackType & track, const std::optional<ControlType> & control,
    const ufil::type::Timestamp & timestamp) override
  {
    this->storeControlInput(track, control);
    this->predict(track, control, timestamp, track.currentClassification());
  }

  virtual void predict(
    const TrackType & track, const std::optional<ControlType> & control,
    const ufil::type::Timestamp & timestamp, ClassificationType & prediction) = 0;
};

}  // namespace predict
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__PREDICT__CLASSIFICATION__CLASSIFICATION_PREDICTOR_HPP_
