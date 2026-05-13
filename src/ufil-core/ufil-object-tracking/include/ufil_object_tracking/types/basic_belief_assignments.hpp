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

#ifndef UFIL_OBJECT_TRACKING__TYPES__BASIC_BELIEF_ASSIGNMENTS_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__BASIC_BELIEF_ASSIGNMENTS_HPP_

#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/scalar.hpp"
#include "ufil_object_tracking/types/vector.hpp"

namespace ufil
{
namespace type
{

template<int N>
class BasicBeliefAssignments : public VectorInterface<N>
{
public:
  using VectorType = Vector<N>;

public:
  BasicBeliefAssignments()
  : VectorInterface<N>()
  {
  }
  explicit BasicBeliefAssignments(const VectorType & vector)
  : VectorInterface<N>(vector)
  {
  }

  [[nodiscard]] const auto & basicBeliefVector() const
  {
    return VectorInterface<N>::vector();
  }

  [[nodiscard]] auto & basicBeliefVector()
  {
    return VectorInterface<N>::vector();
  }
};

// namespace basic_belief_assignments
// {

// class ClassificationBasicBeliefAssignments : public ufil::type::BasicBeliefAssignments<13>
// {
// public:
//   static constexpr int CAR = 0;
//   static constexpr int TRUCK = 1;
//   static constexpr int MOTORCYCLE = 2;
//   static constexpr int BICYCLE = 3;
//   static constexpr int PEDESTRIAN = 4;
//   static constexpr int STATIONARY = 5;
//   static constexpr int VEHICLES = 6;
//   static constexpr int VRU = 7;
//   static constexpr int TRAFFIC = 8;
//   static constexpr int VEHICLE_STATIONARY = 9;
//   static constexpr int VRU_STATIONARY = 10;
//   static constexpr int DISCERNMENT = 11;

//   using VectorType = ufil::type::BasicBeliefAssignments<13>::VectorType;

//   ClassificationBasicBeliefAssignments()
//   {
//     vector().setConstant(1.0f / 12.0f);
//   }

//   ufil::type::Scalar car() const {return vector()[CAR];}
//   ufil::type::Scalar truck() const {return vector()[TRUCK];}
//   ufil::type::Scalar pedestrian() const {return vector()[PEDESTRIAN];}
//   ufil::type::Scalar bicycle() const {return vector()[BICYCLE];}
//   ufil::type::Scalar motorcycle() const {return vector()[MOTORCYCLE];}
//   ufil::type::Scalar stationary() const {return vector()[STATIONARY];}
//   ufil::type::Scalar vehicles() const {return vector()[VEHICLES];}
//   ufil::type::Scalar vru() const {return vector()[VRU];}
//   ufil::type::Scalar traffic() const {return vector()[TRAFFIC];}
//   ufil::type::Scalar vehicle_stationary() const {return vector()[VEHICLE_STATIONARY];}
//   ufil::type::Scalar vru_stationary() const {return vector()[VRU_STATIONARY];}
//   ufil::type::Scalar discernment() const {return vector()[DISCERNMENT];}


//   ufil::type::Scalar & car() {return vector()[CAR];}
//   ufil::type::Scalar & truck() {return vector()[TRUCK];}
//   ufil::type::Scalar & pedestrian() {return vector()[PEDESTRIAN];}
//   ufil::type::Scalar & bicycle() {return vector()[BICYCLE];}
//   ufil::type::Scalar & motorcycle() {return vector()[MOTORCYCLE];}
//   ufil::type::Scalar & stationary() {return vector()[STATIONARY];}
//   ufil::type::Scalar & vehicles() {return vector()[VEHICLES];}
//   ufil::type::Scalar & vru() {return vector()[VRU];}
//   ufil::type::Scalar & traffic() {return vector()[TRAFFIC];}
//   ufil::type::Scalar & vehicle_stationary() {return vector()[VEHICLE_STATIONARY];}
//   ufil::type::Scalar & vru_stationary() {return vector()[VRU_STATIONARY];}
//   ufil::type::Scalar & discernment() {return vector()[DISCERNMENT];}
// };

// }  // namespace basic_belief_assignments

}  // namespace type
}  // namespace ufil
#endif  // UFIL_OBJECT_TRACKING__TYPES__BASIC_BELIEF_ASSIGNMENTS_HPP_
