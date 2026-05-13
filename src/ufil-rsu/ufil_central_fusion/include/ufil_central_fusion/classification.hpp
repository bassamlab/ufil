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

#ifndef UFIL_CENTRAL_FUSION__CLASSIFICATION_HPP_
#define UFIL_CENTRAL_FUSION__CLASSIFICATION_HPP_

#include <map>

#include <ufil_object_tracking/types/classification.hpp>

namespace ufil_central_fusion
{

class BasicBeliefClassification : public ufil::type::Classification<13>
{
public:
  static constexpr int CAR = 0;
  static constexpr int TRUCK = 1;
  static constexpr int MOTORCYCLE = 2;
  static constexpr int BICYCLE = 3;
  static constexpr int PEDESTRIAN = 4;
  static constexpr int STATIONARY = 5;
  static constexpr int VEHICLES = 6;
  static constexpr int VRU = 7;
  static constexpr int TRAFFIC = 8;
  static constexpr int VEHICLE_STATIONARY = 9;
  static constexpr int VRU_STATIONARY = 10;
  static constexpr int DISCERNMENT = 11;

  using VectorType = ufil::type::Classification<13>::VectorType;

  BasicBeliefClassification();

  ufil::type::Scalar car() const;
  ufil::type::Scalar truck() const;
  ufil::type::Scalar pedestrian() const;
  ufil::type::Scalar bicycle() const;
  ufil::type::Scalar motorcycle() const;
  ufil::type::Scalar stationary() const;
  ufil::type::Scalar vehicles() const;
  ufil::type::Scalar vru() const;
  ufil::type::Scalar traffic() const;
  ufil::type::Scalar vehicleStationary() const;
  ufil::type::Scalar vruStationary() const;
  ufil::type::Scalar discernment() const;

  ufil::type::Scalar & car();
  ufil::type::Scalar & truck();
  ufil::type::Scalar & pedestrian();
  ufil::type::Scalar & bicycle();
  ufil::type::Scalar & motorcycle();
  ufil::type::Scalar & stationary();
  ufil::type::Scalar & vehicles();
  ufil::type::Scalar & vru();
  ufil::type::Scalar & traffic();
  ufil::type::Scalar & vehicleStationary();
  ufil::type::Scalar & vruStationary();
  ufil::type::Scalar & discernment();
};

class Classification : public ufil::type::classification::ObjectClassification{
private:
  BasicBeliefClassification basic_belief_classification_;

public:
  const BasicBeliefClassification & basicBeliefClassification() const
  {
    return basic_belief_classification_;
  }

  BasicBeliefClassification & basicBeliefClassification()
  {
    return basic_belief_classification_;
  }
};

}  // namespace ufil_central_fusion
#endif  // UFIL_CENTRAL_FUSION__CLASSIFICATION_HPP_
