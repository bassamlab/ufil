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

#include "ufil_central_fusion/classification.hpp"

namespace ufil_central_fusion
{

BasicBeliefClassification::BasicBeliefClassification()
{
  classificationVector().setConstant(1.0f / 12.0f);
}

ufil::type::Scalar BasicBeliefClassification::car() const
{
  return classificationVector()[CAR];
}
ufil::type::Scalar BasicBeliefClassification::truck() const
{
  return classificationVector()[TRUCK];
}
ufil::type::Scalar BasicBeliefClassification::pedestrian() const
{
  return classificationVector()[PEDESTRIAN];
}
ufil::type::Scalar BasicBeliefClassification::bicycle() const
{
  return classificationVector()[BICYCLE];
}
ufil::type::Scalar BasicBeliefClassification::motorcycle() const
{
  return classificationVector()[MOTORCYCLE];
}
ufil::type::Scalar BasicBeliefClassification::stationary() const
{
  return classificationVector()[STATIONARY];
}
ufil::type::Scalar BasicBeliefClassification::vehicles() const
{
  return classificationVector()[VEHICLES];
}
ufil::type::Scalar BasicBeliefClassification::vru() const
{
  return classificationVector()[VRU];
}
ufil::type::Scalar BasicBeliefClassification::traffic() const
{
  return classificationVector()[TRAFFIC];
}
ufil::type::Scalar BasicBeliefClassification::vehicleStationary() const
{
  return classificationVector()[VEHICLE_STATIONARY];
}
ufil::type::Scalar BasicBeliefClassification::vruStationary() const
{
  return classificationVector()[VRU_STATIONARY];
}
ufil::type::Scalar BasicBeliefClassification::discernment() const
{
  return classificationVector()[DISCERNMENT];
}


ufil::type::Scalar & BasicBeliefClassification::car()
{
  return classificationVector()[CAR];
}
ufil::type::Scalar & BasicBeliefClassification::truck()
{
  return classificationVector()[TRUCK];
}
ufil::type::Scalar & BasicBeliefClassification::pedestrian()
{
  return classificationVector()[PEDESTRIAN];
}
ufil::type::Scalar & BasicBeliefClassification::bicycle()
{
  return classificationVector()[BICYCLE];
}
ufil::type::Scalar & BasicBeliefClassification::motorcycle()
{
  return classificationVector()[MOTORCYCLE];
}
ufil::type::Scalar & BasicBeliefClassification::stationary()
{
  return classificationVector()[STATIONARY];
}
ufil::type::Scalar & BasicBeliefClassification::vehicles()
{
  return classificationVector()[VEHICLES];
}
ufil::type::Scalar & BasicBeliefClassification::vru()
{
  return classificationVector()[VRU];
}
ufil::type::Scalar & BasicBeliefClassification::traffic()
{
  return classificationVector()[TRAFFIC];
}
ufil::type::Scalar & BasicBeliefClassification::vehicleStationary()
{
  return classificationVector()[VEHICLE_STATIONARY];
}
ufil::type::Scalar & BasicBeliefClassification::vruStationary()
{
  return classificationVector()[VRU_STATIONARY];
}
ufil::type::Scalar & BasicBeliefClassification::discernment()
{
  return classificationVector()[DISCERNMENT];
}


}   // namespace ufil_central_fusion
