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

#ifndef UFIL_OBJECT_TRACKING__TYPES__SENSOR_FOV_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__SENSOR_FOV_HPP_

#include <Eigen/Geometry>

#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/covariance.hpp"
#include "ufil_object_tracking/types/vector.hpp"

namespace ufil
{
namespace type
{

/**
 * @brief Cone-shaped Field of View (FOV) description for a directional sensor.
 *
 * Represents the 3D sensing limits of a sensor using a truncated cone model.
 * Angles are stored in radians and distances in meters.
 *
 * Layout (internal vector):
 *   [0] r_min
 *   [1] r_max
 *   [2] phi_min
 *   [3] phi_max
 *   [4] theta_min
 *   [5] theta_max
 */
class SensorFOV : public VectorInterface<6>, public CovarianceInterface<6>
{
public:
  using FOVVectorType = Vector<6>;
  using CovarianceMatrixType = Covariance<6>;
  using OriginVectorType = Vector<3>;

  static constexpr int R_MIN = 0;
  static constexpr int R_MAX = 1;
  static constexpr int PHI_MIN = 2;
  static constexpr int PHI_MAX = 3;
  static constexpr int THETA_MIN = 4;
  static constexpr int THETA_MAX = 5;

protected:
  // origin of the sensor (x,y,z) in the same frame as angles/distances
  OriginVectorType origin_ = OriginVectorType::Zero();

public:
  constexpr static auto Size = 6;

  SensorFOV()
  : VectorInterface<6>(), CovarianceInterface<6>()
  {
  }

  // --------------------------------------------------------------------------
  // Origin access
  // --------------------------------------------------------------------------
  [[nodiscard]] const OriginVectorType & origin() const {return origin_;}
  [[nodiscard]] OriginVectorType & origin() {return origin_;}
  [[nodiscard]] Scalar originX() const {return origin_[0];}
  [[nodiscard]] Scalar originY() const {return origin_[1];}
  [[nodiscard]] Scalar originZ() const {return origin_[2];}
  Scalar & originX() {return origin_[0];}
  Scalar & originY() {return origin_[1];}
  Scalar & originZ() {return origin_[2];}

  // --------------------------------------------------------------------------
  // FOV parameter accessors
  // --------------------------------------------------------------------------

  // --- r_min ---
  ufil::type::Scalar rMin() const {return this->vector_[R_MIN];}
  ufil::type::Scalar & rMin() {return this->vector_[R_MIN];}

  // --- r_max ---
  ufil::type::Scalar rMax() const {return this->vector_[R_MAX];}
  ufil::type::Scalar & rMax() {return this->vector_[R_MAX];}

  // --- phi_min (horizontal left) ---
  ufil::type::Scalar phiMin() const {return this->vector_[PHI_MIN];}
  ufil::type::Scalar & phiMin() {return this->vector_[PHI_MIN];}

  // --- phi_max (horizontal right) ---
  ufil::type::Scalar phiMax() const {return this->vector_[PHI_MAX];}
  ufil::type::Scalar & phiMax() {return this->vector_[PHI_MAX];}

  // --- theta_min (vertical down) ---
  ufil::type::Scalar thetaMin() const {return this->vector_[THETA_MIN];}
  ufil::type::Scalar & thetaMin() {return this->vector_[THETA_MIN];}

  // --- theta_max (vertical up) ---
  ufil::type::Scalar thetaMax() const {return this->vector_[THETA_MAX];}
  ufil::type::Scalar & thetaMax() {return this->vector_[THETA_MAX];}
};

}  // namespace type
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__SENSOR_FOV_HPP_
