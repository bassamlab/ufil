// Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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


#pragma once

#include <kalman_cpp/kalman_cpp.hpp>

namespace model
{

class MeasurementModelBase {
public:
  virtual ~MeasurementModelBase() = default;
};

/**
 * @brief System state vector-type for an object
 *
 * @param T Numeric scalar type
 */
template<typename T>
class ConstantVelocityState : public kalman_cpp::Vector<T, 5> {
public:
  KALMAN_VECTOR(ConstantVelocityState, T, 5)

  static constexpr size_t x_ = 0;
  static constexpr size_t y_ = 1;
  static constexpr size_t speed_ = 2;
  static constexpr size_t yaw_ = 3;
  static constexpr size_t yaw_rate_ = 4;

  [[nodiscard]] auto x() const -> T {return (*this)[x_];}

  [[nodiscard]] auto x() -> T & {return (*this)[x_];}

  [[nodiscard]] auto y() const -> T {return (*this)[y_];}

  [[nodiscard]] auto y() -> T & {return (*this)[y_];}

  [[nodiscard]] auto speed() const -> T {return (*this)[speed_];}

  [[nodiscard]] auto speed() -> T & {return (*this)[speed_];}

  [[nodiscard]] auto yaw() const -> T {return (*this)[yaw_];}

  [[nodiscard]] auto yaw() -> T & {return (*this)[yaw_];}

  [[nodiscard]] auto yawRate() const -> T {return (*this)[yaw_rate_];}

  [[nodiscard]] auto yawRate() -> T & {return (*this)[yaw_rate_];}

  friend auto operator<<(std::ostream & os, const ConstantVelocityState<T> & o) -> std::ostream &
  {
    os << "X: " << o.x() << " ";
    os << "Y: " << o.y() << " ";
    os << "SPEED: " << o.speed() << " ";
    os << "YAW: " << o.yaw() << " ";
    os << "YAW RATE: " << o.yawRate() << " ";
    return os;
  }
};

/**
 * @brief System control-input vector-type for a a IMU
 *
 * @param T Numeric scalar type
 */
template<typename T>
class TimeStepControl : public kalman_cpp::Vector<T, 1> {
public:
  KALMAN_VECTOR(TimeStepControl, T, 1)

  static constexpr size_t time_step = 0;

  [[nodiscard]] auto timeStep() const -> T {return (*this)[time_step];}

  [[nodiscard]] auto timeStep() -> T & {return (*this)[time_step];}
};

/**
 * @brief System model for a IMU
 *
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = kalman_cpp::StandardBase>
class ConstantVelocitySystemModel
  : public kalman_cpp::
  LinearizedSystemModel<ConstantVelocityState<T>, TimeStepControl<T>, CovarianceBase> {
public:
        //! State type shortcut definition
  using S = model::ConstantVelocityState<T>;

        //! Control type shortcut definition
  using C = model::TimeStepControl<T>;

        /**
         * @brief Definition of (non-linear) state transition function
         *
         * This function defines how the system state is propagated through time,
         * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
         * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
         * the system control input \f$u\f$.
         *
         * @param [in] x The system state in current time-step
         * @param [in] u The control vector input
         * @returns The (predicted) system state in the next time-step
         */
  [[nodiscard]] auto f(const S & x, const C & u) const -> S
  {
            //! Predicted state vector after transition
    S x_ = x;

    auto yaw_estimated = x.yaw();         /* + 0.5 * u.timeStep() * x.yawRate();*/
    x_.x() = x.x() + u.timeStep() * x.speed() * std::cos(yaw_estimated);
    x_.y() = x.y() + u.timeStep() * x.speed() * std::sin(yaw_estimated);

    x_.speed() = x.speed();
    x_.yaw() = phi(x.yaw() + u.timeStep() * x.yawRate());
    x_.yawRate() = x.yawRate();

            // Return transitioned state vector
    return x_;
  }

  ConstantVelocitySystemModel()
  {
    this->F.setIdentity();

    this->W.setIdentity();
    auto cov_matrix = kalman_cpp::Matrix<T, 5, 5>();
    cov_matrix.setZero();
    cov_matrix.diagonal() << 0.1f, 0.1f, 0.4f, 0.5f, 0.8f;
    this->setCovariance(cov_matrix);
  }

protected:
        /**
         * @brief Update jacobian matrices for the system state transition function using curfrent state
         *
         * This will re-compute the (state-dependent) elements of the jacobian matrices
         * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
         * current state \f$x\f$.
         *
         * @note This is only needed when implementing a LinearizedSystemModel,
         *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
         *       When using a fully non-linear filter such as the UnscentedKalmanFilter
         *       or its square-root form then this is not needed.
         *
         * @param x The current system state around which to linearize
         * @param u The current system control input
         */
  void updateJacobians(const S & x, const C & u)
  {
            // F = df/dx (Jacobian of state transition w.r.t. the state)
    this->F.setIdentity();

    this->F(S::x_, S::x_) = 1;
    this->F(S::x_,
        S::speed_) = u.timeStep() * std::cos(x.yaw() /*+ 0.5 * u.timeStep() * x.yawRate()*/);
    this->F(S::x_,
        S::yaw_) = -u.timeStep() * x.speed() *
      std::sin(x.yaw() /*+ 0.5 * u.timeStep() * x.yawRate()*/);
            // this->F(S::x_, S::yaw_rate_) =
            // -0.5 * u.timeStep() * u.timeStep() * x.speed() *
            // std::sin(x.yaw() + 0.5 * u.timeStep() * x.yawRate());

    this->F(S::y_, S::y_) = 1;
    this->F(S::y_,
        S::speed_) = u.timeStep() * std::sin(x.yaw() /*+ 0.5 * u.timeStep() * x.yawRate()*/);
    this->F(S::y_,
        S::yaw_) = u.timeStep() * x.speed() *
      std::cos(x.yaw() /*+ 0.5 * u.timeStep() * x.yawRate()*/);
            // this->F(S::y_, S::yaw_rate_) =
            // 0.5 * u.timeStep() * u.timeStep() * x.speed() *
            // std::cos(x.yaw() + 0.5 * u.timeStep() * x.yawRate());

    this->F(S::yaw_, S::yaw_) = 1;
    this->F(S::yaw_, S::yaw_rate_) = u.timeStep();

    this->F(S::speed_, S::speed_) = 1;

    this->F(S::yaw_rate_, S::yaw_rate_) = 1;

            // W = df/dw (Jacobian of state transition w.r.t. the noise)
    this->W.setIdentity();
    this->W(S::x_, S::x_) =
      0.5 * u.timeStep() * u.timeStep() * std::cos(x.yaw() /*+ 0.5 * u.timeStep() * x.yawRate()*/);
    this->W(S::y_, S::y_) =
      0.5 * u.timeStep() * u.timeStep() * std::sin(x.yaw() /*+ 0.5 * u.timeStep() * x.yawRate()*/);
    this->W(S::yaw_, S::yaw_) = 0.5 * u.timeStep() * u.timeStep();
    this->W(S::speed_, S::speed_) = u.timeStep();
    this->W(S::yaw_rate_, S::yaw_rate_) = u.timeStep();
  }

        /**
         * @brief Maps angle ranges [0,2*pi) -> [-pi,pi)
         *
         * @param theta angle to be mapped
         * @return float wrapped angle
         */
  [[nodiscard]] auto phi(float const theta) const -> float
  {
    float theta_wrapped = fmod(theta + M_PI, 2.0f * M_PI);
    if (theta_wrapped < 0) {
      theta_wrapped += 2.0f * M_PI;
    }
    return theta_wrapped - M_PI;
  }
};

inline auto angleDiff(float const x, float const y) -> float
{
  return std::atan2(std::sin(x - y), std::cos(x - y));
}

}  // namespace model
