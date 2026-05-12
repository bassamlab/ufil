// The MIT License (MIT)
//
// Copyright (c) 2015 Markus Herb
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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef KALMAN_CPP__TYPES_HPP_
#define KALMAN_CPP__TYPES_HPP_

#include "kalman_cpp/Matrix.hpp"

namespace kalman_cpp
{
/**
 * @class kalman_cpp::SquareMatrix
 * @brief Template type representing a square matrix
 * @param T The numeric scalar type
 * @param N The dimensionality of the Matrix
 */
template<typename T, int N>
using SquareMatrix = Matrix<T, N, N>;

/**
 * @class kalman_cpp::Covariance
 * @brief Template type for covariance matrices
 * @param Type The vector type for which to generate a covariance (usually a state or measurement type)
 */
template<class Type>
using Covariance = SquareMatrix<typename Type::Scalar, Type::RowsAtCompileTime>;

/**
 * @class kalman_cpp::CovarianceSquareRoot
 * @brief Template type for covariance square roots
 * @param Type The vector type for which to generate a covariance (usually a state or measurement type)
 */
template<class Type>
using CovarianceSquareRoot = Cholesky<Covariance<Type>>;

/**
 * @class kalman_cpp::KalmanGain
 * @brief Template type of Kalman Gain
 * @param State The system state type
 * @param Measurement The measurement type
 */
template<class State, class Measurement>
using KalmanGain = Matrix<typename State::Scalar, State::RowsAtCompileTime,
    Measurement::RowsAtCompileTime>;

/**
 * @class kalman_cpp::Jacobian
 * @brief Template type of jacobian of A w.r.t. B
 */
template<class A, class B>
using Jacobian = Matrix<typename A::Scalar, A::RowsAtCompileTime, B::RowsAtCompileTime>;
}  // namespace kalman_cpp

#endif  // KALMAN_CPP__TYPES_HPP_
