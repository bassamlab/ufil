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

#ifndef COVARIANCE_HPP_
#define COVARIANCE_HPP_

// Private helpers for 2D covariances.

#include <numbers>  // cpplint treats this C++20 header as a C system header

#include <algorithm>
#include <array>
#include <cmath>

namespace ufil_etsi
{

/// Variance of the projection of a 2D quantity with covariance [[xx, xy], [xy, yy]] onto the
/// unit vector (ux, uy).
inline double projectedVariance(double xx, double xy, double yy, double ux, double uy)
{
  return ux * ux * xx + 2.0 * ux * uy * xy + uy * uy * yy;
}

/// Principal axes of a 2D covariance [[xx, xy], [xy, yy]].
struct PrincipalAxes
{
  double major_variance;
  double minor_variance;
  /// Angle of the major axis, counter-clockwise from the x axis [rad].
  double major_angle;
};

inline PrincipalAxes principalAxes(double xx, double xy, double yy)
{
  const double mean = 0.5 * (xx + yy);
  const double radius = std::hypot(0.5 * (xx - yy), xy);
  PrincipalAxes result;
  result.major_variance = mean + radius;
  result.minor_variance = std::max(0.0, mean - radius);
  // A circle has no major axis; prefer the y axis (north) as the generator always did.
  result.major_angle = radius > 0.0 ? 0.5 * std::atan2(2.0 * xy, xx - yy) : std::numbers::pi / 2;
  return result;
}

/// 2D covariance [xx, xy, yy] with the given principal axes.
inline std::array<double, 3> covarianceFromAxes(
  double major_variance, double minor_variance, double major_angle)
{
  const double c = std::cos(major_angle);
  const double s = std::sin(major_angle);
  return {
    c * c * major_variance + s * s * minor_variance,
    c * s * (major_variance - minor_variance),
    s * s * major_variance + c * c * minor_variance};
}

/// Normalizes an angle into (-pi, pi].
inline double normalizeAngle(double angle)
{
  const double result = std::remainder(angle, 2.0 * std::numbers::pi);
  return result <= -std::numbers::pi ? result + 2.0 * std::numbers::pi : result;
}

}  // namespace ufil_etsi

#endif  // COVARIANCE_HPP_
