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

#include "ufil_examples_cpm_lab_adapter/artificial_gnss.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <stdexcept>
#include <string>

#include "ufil_examples_cpm_lab_adapter/vehicle_state.hpp"

namespace ufil_examples_cpm_lab_adapter
{

namespace
{

/// Position of the GNSS receiver relative to the vehicle centre [m].
const Eigen::Vector3d kReceiverOffset{-0.075, 0.0, 0.0};

bool isValidError(double error)
{
  return std::isfinite(error) && error >= 0.0;
}

}  // namespace

std::optional<std::string> validate(const ArtificialGnssConfig & config)
{
  if (config.number_of_frequency_bands < 1) {
    return "number_of_frequency_bands must be at least 1";
  }
  if (!isValidError(config.clock_error) || !isValidError(config.ionospheric_error) ||
    !isValidError(config.ephemeris_error))
  {
    return "the GNSS errors must be finite and not negative";
  }
  return std::nullopt;
}

ArtificialGnss::ArtificialGnss(
  const ArtificialGnssConfig & config, std::optional<unsigned int> seed)
: config_(config),
  random_engine_(seed ? *seed : std::random_device{}()),
  satellite_positions_{
    Eigen::Vector3d{-1000000.0, -1000000.0, 1111111.0},
    Eigen::Vector3d{1732050.0, 2.0, 1111111.0},
    Eigen::Vector3d{-1000000.0, 1000000.0, 1111111.0},
    Eigen::Vector3d{2.25, 2.0, 1277778.0}}
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument("invalid artificial GNSS configuration: " + *error);
  }
}

double ArtificialGnss::scaledRandomError(double real_world_error)
{
  // 95 % (two standard deviations, rounded up to three) of the errors are within the real-world
  // error scaled down to the lab.
  const double standard_deviation = real_world_error / kLabScale / 3.0;
  if (standard_deviation <= 0.0) {
    return 0.0;
  }
  std::normal_distribution<double> distribution(0.0, standard_deviation);
  return distribution(random_engine_);
}

Eigen::Vector3d ArtificialGnss::generateFix(const Eigen::Vector3d & position, double yaw)
{
  const Eigen::Vector3d receiver = Eigen::Vector3d(position.x(), position.y(), 0.0) +
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * kReceiverOffset;

  // Pseudo ranges averaged over the frequency bands.
  Eigen::Vector4d ranges = Eigen::Vector4d::Zero();
  for (std::size_t i = 0; i < satellite_positions_.size(); ++i) {
    for (int band = 0; band < config_.number_of_frequency_bands; ++band) {
      const double error = scaledRandomError(config_.ionospheric_error) +
        scaledRandomError(config_.clock_error) + scaledRandomError(config_.ephemeris_error);
      ranges[static_cast<Eigen::Index>(i)] +=
        (receiver - satellite_positions_[i]).norm() + error;
    }
  }
  ranges /= static_cast<double>(config_.number_of_frequency_bands);

  // Trilateration: |p - s|^2 = r^2 is linear in (|p|^2, p): |p|^2 - 2 s^T p = r^2 - |s|^2.
  Eigen::Matrix4d a;
  Eigen::Vector4d b;
  for (std::size_t i = 0; i < satellite_positions_.size(); ++i) {
    const auto row = static_cast<Eigen::Index>(i);
    const Eigen::Vector3d & satellite = satellite_positions_[i];
    a.row(row) << 1.0, -2.0 * satellite.x(), -2.0 * satellite.y(), -2.0 * satellite.z();
    b[row] = ranges[row] * ranges[row] - satellite.squaredNorm();
  }
  const Eigen::Vector4d solution = a.colPivHouseholderQr().solve(b);
  return {solution[1], solution[2], 0.0};
}

}  // namespace ufil_examples_cpm_lab_adapter
