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

#ifndef UFIL_OBJECT_TRACKING__UPDATE__UPDATER_HPP_
#define UFIL_OBJECT_TRACKING__UPDATE__UPDATER_HPP_

#include <memory>
#include <optional>
#include <map>
#include <set>

#include "ufil_object_tracking/models/measurement/measurement_model.hpp"
#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace update
{

template<class T>
class Updater
{
public:
  using TrackType = T;
  using MeasurementType = T::MeasurementType;

public:
  virtual void update_parameter(
    const std::map<type::Id, T> & /*tracks*/,
    const std::map<type::Id, MeasurementType> & /*associations*/,
    const std::set<MeasurementType> & /*unassociated_measurements*/)
  {
  }

  virtual void update(
    TrackType & track, const std::optional<MeasurementType> & measurement,
    const ufil::type::Timestamp & timestamp) = 0;
};

}  // namespace update
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__UPDATE__UPDATER_HPP_
