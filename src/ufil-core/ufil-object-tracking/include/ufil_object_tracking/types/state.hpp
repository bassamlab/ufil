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

#ifndef UFIL_OBJECT_TRACKING__TYPES__STATE_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__STATE_HPP_

#include "ufil_object_tracking/types/covariance.hpp"
#include "ufil_object_tracking/types/matrix.hpp"
#include "ufil_object_tracking/types/time.hpp"
#include "ufil_object_tracking/types/vector.hpp"

namespace ufil
{
namespace type
{

template<int N>
class State : public VectorInterface<N>, public CovarianceInterface<N>
{
public:
  using CovarianceMatrixType = Covariance<N>;
  using StateVectorType = Vector<N>;

protected:
  ufil::type::Timestamp timestamp_;

public:
  constexpr static auto Size = N;

  explicit State(
    const ufil::type::Timestamp & timestamp = ufil::from_nanoseconds<ufil::type::Timestamp>(0))
  : VectorInterface<N>(), CovarianceInterface<N>(), timestamp_(timestamp)
  {
  }

  [[nodiscard]] const ufil::type::Timestamp & timestamp() const
  {
    return timestamp_;
  }

  [[nodiscard]] ufil::type::Timestamp & timestamp()
  {
    return timestamp_;
  }

  [[nodiscard]] const StateVectorType & stateVector() const
  {
    return this->vector_;
  }

  [[nodiscard]] StateVectorType & stateVector()
  {
    return this->vector_;
  }

  [[nodiscard]] const CovarianceMatrixType & covariance() const
  {
    return this->covariance_;
  }

  [[nodiscard]] CovarianceMatrixType & covariance()
  {
    return this->covariance_;
  }
};

namespace state
{

class NoState : public ufil::type::State<0>
{
public:
  explicit NoState(const ufil::type::Timestamp & timestamp = ufil::from_nanoseconds<Timestamp>(0))
  : ufil::type::State<0>(timestamp)
  {
  }
};

class Position2D : public ufil::type::State<2>
{
public:
  static constexpr int X = 0;
  static constexpr int Y = 1;

  explicit Position2D(const ufil::type::Timestamp & timestamp = ufil::type::Timestamp())
  : ufil::type::State<2>(timestamp)
  {
  }

  ufil::type::Vector2 position() const
  {
    return {stateVector()[X], stateVector()[Y]};
  }

  ufil::type::Scalar x() const
  {
    return stateVector()[X];
  }

  ufil::type::Scalar y() const
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar & x()
  {
    return stateVector()[X];
  }

  ufil::type::Scalar & y()
  {
    return stateVector()[Y];
  }
};

class PositionVelocity2D : public ufil::type::State<4>
{
public:
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int VX = 2;
  static constexpr int VY = 3;

  explicit PositionVelocity2D(const ufil::type::Timestamp & timestamp = ufil::type::Timestamp())
  : ufil::type::State<4>(timestamp)
  {
  }

  ufil::type::Vector2 position() const
  {
    return {stateVector()[X], stateVector()[Y]};
  }

  ufil::type::Vector2 velocity() const
  {
    return {stateVector()[VX], stateVector()[VY]};
  }

  ufil::type::Scalar x() const
  {
    return stateVector()[X];
  }

  ufil::type::Scalar y() const
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar vx() const
  {
    return stateVector()[VX];
  }

  ufil::type::Scalar vy() const
  {
    return stateVector()[VY];
  }

  ufil::type::Scalar & x()
  {
    return stateVector()[X];
  }

  ufil::type::Scalar & y()
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar & vx()
  {
    return stateVector()[VX];
  }

  ufil::type::Scalar & vy()
  {
    return stateVector()[VY];
  }
};

class PositionVelocityAcceleration2D : public ufil::type::State<6>
{
public:
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int VX = 2;
  static constexpr int VY = 3;
  static constexpr int AX = 4;
  static constexpr int AY = 5;

  explicit PositionVelocityAcceleration2D(
    const ufil::type::Timestamp & timestamp = ufil::type::Timestamp())
  : ufil::type::State<6>(timestamp)
  {
  }

  ufil::type::Vector2 position() const
  {
    return {stateVector()[X], stateVector()[Y]};
  }

  ufil::type::Vector2 velocity() const
  {
    return {stateVector()[VX], stateVector()[VY]};
  }

  ufil::type::Vector2 acceleration() const
  {
    return {stateVector()[AX], stateVector()[AY]};
  }

  ufil::type::Scalar x() const
  {
    return stateVector()[X];
  }

  ufil::type::Scalar y() const
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar vx() const
  {
    return stateVector()[VX];
  }

  ufil::type::Scalar vy() const
  {
    return stateVector()[VY];
  }

  ufil::type::Scalar ax() const
  {
    return stateVector()[AX];
  }

  ufil::type::Scalar ay() const
  {
    return stateVector()[AY];
  }

  ufil::type::Scalar & x()
  {
    return stateVector()[X];
  }

  ufil::type::Scalar & y()
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar & vx()
  {
    return stateVector()[VX];
  }

  ufil::type::Scalar & vy()
  {
    return stateVector()[VY];
  }

  ufil::type::Scalar & ax()
  {
    return stateVector()[AX];
  }

  ufil::type::Scalar & ay()
  {
    return stateVector()[AY];
  }
};

class Pose2D : public ufil::type::State<3>
{
public:
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int YAW = 2;

  explicit Pose2D(const ufil::type::Timestamp & timestamp = ufil::type::Timestamp())
  : ufil::type::State<3>(timestamp)
  {
  }

  ufil::type::Vector2 position() const
  {
    return {stateVector()[X], stateVector()[Y]};
  }

  ufil::type::Scalar x() const
  {
    return stateVector()[X];
  }

  ufil::type::Scalar y() const
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar yaw() const
  {
    return stateVector()[YAW];
  }

  ufil::type::Scalar & x()
  {
    return stateVector()[X];
  }

  ufil::type::Scalar & y()
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar & yaw()
  {
    return stateVector()[YAW];
  }
};

class PoseVelocity2D : public ufil::type::State<6>
{
public:
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int VX = 2;
  static constexpr int VY = 3;
  static constexpr int YAW = 4;
  static constexpr int YAW_RATE = 5;

  explicit PoseVelocity2D(const ufil::type::Timestamp & timestamp = ufil::type::Timestamp())
  : ufil::type::State<6>(timestamp)
  {
  }

  ufil::type::Vector2 position() const
  {
    return {stateVector()[X], stateVector()[Y]};
  }

  ufil::type::Vector2 velocity() const
  {
    return {stateVector()[VX], stateVector()[VY]};
  }

  ufil::type::Scalar x() const
  {
    return stateVector()[X];
  }

  ufil::type::Scalar y() const
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar vx() const
  {
    return stateVector()[VX];
  }

  ufil::type::Scalar vy() const
  {
    return stateVector()[VY];
  }

  ufil::type::Scalar yaw() const
  {
    return stateVector()[YAW];
  }

  ufil::type::Scalar yawRate() const
  {
    return stateVector()[YAW_RATE];
  }

  ufil::type::Scalar & x()
  {
    return stateVector()[X];
  }

  ufil::type::Scalar & y()
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar & vx()
  {
    return stateVector()[VX];
  }

  ufil::type::Scalar & vy()
  {
    return stateVector()[VY];
  }

  ufil::type::Scalar & yaw()
  {
    return stateVector()[YAW];
  }

  ufil::type::Scalar & yawRate()
  {
    return stateVector()[YAW_RATE];
  }
};

class PoseVelocityAcceleration2D : public ufil::type::State<8>
{
public:
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int VX = 2;
  static constexpr int VY = 3;
  static constexpr int AX = 4;
  static constexpr int AY = 5;
  static constexpr int YAW = 6;
  static constexpr int YAW_RATE = 7;

  explicit PoseVelocityAcceleration2D(
    const ufil::type::Timestamp & timestamp = ufil::type::Timestamp())
  : ufil::type::State<8>(timestamp)
  {
  }

  ufil::type::Vector2 position() const
  {
    return {stateVector()[X], stateVector()[Y]};
  }

  ufil::type::Vector2 velocity() const
  {
    return {stateVector()[VX], stateVector()[VY]};
  }

  ufil::type::Vector2 acceleration() const
  {
    return {stateVector()[AX], stateVector()[AY]};
  }

  ufil::type::Scalar x() const
  {
    return stateVector()[X];
  }

  ufil::type::Scalar y() const
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar vx() const
  {
    return stateVector()[VX];
  }

  ufil::type::Scalar vy() const
  {
    return stateVector()[VY];
  }

  ufil::type::Scalar ax() const
  {
    return stateVector()[AX];
  }

  ufil::type::Scalar ay() const
  {
    return stateVector()[AY];
  }

  ufil::type::Scalar yaw() const
  {
    return stateVector()[YAW];
  }

  ufil::type::Scalar yawRate() const
  {
    return stateVector()[YAW_RATE];
  }

  ufil::type::Scalar & x()
  {
    return stateVector()[X];
  }

  ufil::type::Scalar & y()
  {
    return stateVector()[Y];
  }

  ufil::type::Scalar & vx()
  {
    return stateVector()[VX];
  }

  ufil::type::Scalar & vy()
  {
    return stateVector()[VY];
  }

  ufil::type::Scalar & yaw()
  {
    return stateVector()[YAW];
  }

  ufil::type::Scalar & yawRate()
  {
    return stateVector()[YAW_RATE];
  }

  ufil::type::Scalar & ax()
  {
    return stateVector()[AX];
  }

  ufil::type::Scalar & ay()
  {
    return stateVector()[AY];
  }
};
}  // namespace state

}  // namespace type
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__STATE_HPP_
