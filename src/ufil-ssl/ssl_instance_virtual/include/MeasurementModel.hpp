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

#include <chrono>
#include <cstdint>
#include <map>
#include "SystemModel.hpp"
#include <kalman_cpp/kalman_cpp.hpp>

namespace model
{
/**
 * @brief Measurement vector measuring the vehicle by: x y speed yaw yawRate
 * @param T Numeric scalar type
 */
template<typename T>
class Measurement : public kalman_cpp::Vector<T, 5> {
public:
  KALMAN_VECTOR(Measurement, T, 5)

  static constexpr size_t x_ = 0;

  [[nodiscard]] auto x() const -> T {return (*this)[x_];}

  [[nodiscard]] auto x() -> T & {return (*this)[x_];}

  static constexpr size_t y_ = 1;

  [[nodiscard]] auto y() const -> T {return (*this)[y_];}

  [[nodiscard]] auto y() -> T & {return (*this)[y_];}

  static constexpr size_t speed_ = 2;

  [[nodiscard]] auto speed() const -> T {return (*this)[speed_];}

  [[nodiscard]] auto speed() -> T & {return (*this)[speed_];}

  static constexpr size_t yaw_ = 3;

  [[nodiscard]] auto yaw() const -> T {return (*this)[yaw_];}

  [[nodiscard]] auto yaw() -> T & {return (*this)[yaw_];}

  static constexpr size_t yaw_rate_ = 4;

  [[nodiscard]] auto yawRate() const -> T {return (*this)[yaw_rate_];}

  [[nodiscard]] auto yawRate() -> T & {return (*this)[yaw_rate_];}
};

/**
 * @brief Measurement model for measuring the dynamics of the vehicle using the onboard fusion on Measurement_x_y_speed_yaw_yawRate
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = kalman_cpp::StandardBase>
class MeasurementModel : public kalman_cpp::LinearizedMeasurementModel<ConstantVelocityState<T>,
    Measurement<T>,
    CovarianceBase>,
  public model::MeasurementModelBase {
private:
  Measurement<T> z_;
  bool measurement_valid_ = false;

public:
  void validate() {measurement_valid_ = true;}

  void invalidate() {this->measurement_valid_ = true;}

  [[nodiscard]] auto isValid() const -> bool {return this->measurement_valid_;}

  void setMeasurement(Measurement<T> const & z) {this->z_ = z;}

  [[nodiscard]] auto getMeasurement() const -> Measurement<T> {return this->z_;}

public:
  friend auto operator<<(std::ostream & os, const MeasurementModel<T> & o) -> std::ostream &
  {
    os << "X: " << o.x() << " ";
    os << "Y: " << o.y() << " ";
    os << "SPEED: " << o.speed() << " ";
    os << "YAW: " << o.yaw() << " ";
    os << "YAW RATE: " << o.yawRate() << " ";

    return os;
  }
//! State type shortcut definition
  using S = ConstantVelocityState<T>;

        //! Measurement type shortcut definition
  using M = Measurement<T>;

        /**
         * @brief Constructor
         **/
  MeasurementModel()
  {
            // Setup noise jacobian. As this one is static, we can define it once
            // and do not need to update it dynamically
    this->H.setZero();
    this->H(M::x_, S::x_) = 1;
    this->H(M::y_, S::y_) = 1;
    this->H(M::speed_, S::speed_) = 1;
    this->H(M::yaw_, S::yaw_) = 1;
    this->H(M::yaw_rate_, S::yaw_rate_) = 1;
    this->V.setIdentity();
    auto cov_matrix = kalman_cpp::Matrix<T, 5, 5>();
    cov_matrix.setZero();
    cov_matrix.diagonal() << 0.01f, 0.01f, 0.01f, 0.01f, 0.01f;
    this->setCovariance(cov_matrix);
  }

        /**
          * @brief Definition of (possibly non-linear) measurement function
          *
          * This function maps the system state to the measurement that is expected
          * to be received from the sensor assuming the system is currently in the
          * estimated state.
          *
          * @param [in] x The system state in current time-step
          * @returns The (predicted) sensor measurement for the system state
          */
  [[nodiscard]] auto h(const S & x) const -> M override
  {
            // States are identical
    M m;
    m.x() = x.x();
    m.y() = x.y();
    m.speed() = x.speed();
    m.yaw() = x.yaw();
    m.yawRate() = x.yawRate();
    return m;
  }

protected:
        /**
         * @brief Update jacobian matrices for the system state transition function using current state
         *
         * This will re-compute the (state-dependent) elements of the jacobian matrices
         * to linearize the non-linear measurement function $h(x)$ around the
         *
         * @note This is only needed when implementing a LinearizedSystemModel,
         *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
         *       When using a fully non-linear filter such as the UnscentedKalmanFilter
         *       or its square-root form then this is not needed.
         *
         * @param x The current system state around which to linearize
         * @param u The current system control input
         */
  void updateJacobians(const S & /*x*/) override
  {
            // H = dh/dx (Jacobian of measurement function w.r.t. the state)
    this->H.setZero();
    this->H(M::x_, S::x_) = 1;
    this->H(M::y_, S::y_) = 1;
    this->H(M::speed_, S::speed_) = 1;
    this->H(M::yaw_, S::yaw_) = 1;
    this->H(M::yaw_rate_, S::yaw_rate_) = 1;
  }
};
}  // namespace model
#define PRED_UPDATE_1_1_1_1_1(EKF, SYSTEM, STATE, CONTROL, BASE) \
  if(auto ptr = \
    std::dynamic_pointer_cast<model::Measurement_x_y_speed_yaw_yawRate_Model<T>>(BASE); \
    ptr != nullptr && ptr->isValid()) { \
    STATE = EKF.predict(SYSTEM, CONTROL); \
    auto z = ptr->getMeasurement(); \
    z.yaw() = STATE.yaw() + model::angleDiff(z.yaw(), STATE.yaw()); \
    STATE = EKF.update(*ptr, z); \
  }
#define MEASURE_CAST_1_1_1_1_1(BASE, X, Y, SPEED, YAW, YAWRATE) \
  if(auto ptr = \
    std::dynamic_pointer_cast<model::Measurement_x_y_speed_yaw_yawRate_Model<T>>(BASE); \
    ptr != nullptr && ptr->isValid()) { \
    auto z = ptr->getMeasurement(); \
    X = z.x(); \
    Y = z.y(); \
    SPEED = z.speed(); \
    YAW = z.yaw(); \
    YAWRATE = z.yawRate(); \
 \
  }
#define MEASURE_UPDATE_ASSOC_1_1_1_1_1(BASE, ASSOC_OBJ, TIME_STAMP) \
  if(auto ptr = \
    std::dynamic_pointer_cast<model::Measurement_x_y_speed_yaw_yawRate_Model<T>>(BASE); \
    ptr != nullptr && ptr->isValid()) { \
 \
    ASSOC_OBJ.updateHistoryFrom(TIME_STAMP, ptr); \
  }
