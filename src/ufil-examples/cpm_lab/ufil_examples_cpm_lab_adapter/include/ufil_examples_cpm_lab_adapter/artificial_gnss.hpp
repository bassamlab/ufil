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

#ifndef UFIL_EXAMPLES_CPM_LAB_ADAPTER__ARTIFICIAL_GNSS_HPP_
#define UFIL_EXAMPLES_CPM_LAB_ADAPTER__ARTIFICIAL_GNSS_HPP_

#include <Eigen/Core>

#include <array>
#include <optional>
#include <random>
#include <string>

#include "ufil_examples_cpm_lab_adapter/visibility_control.hpp"

namespace ufil_examples_cpm_lab_adapter
{

/// Real-world GNSS error magnitudes [m], scaled down to the lab by kLabScale.
struct ArtificialGnssConfig
{
  /// Number of frequency bands whose pseudo ranges are averaged.
  int number_of_frequency_bands = 3;
  double clock_error = 2.0;
  double ionospheric_error = 5.0;
  double ephemeris_error = 2.5;
};

/// @return a description of the first invalid value, or std::nullopt if the configuration is valid.
UFIL_EXAMPLES_CPM_LAB_ADAPTER_PUBLIC std::optional<std::string> validate(
  const ArtificialGnssConfig & config);

/**
 * @brief Simulates the GNSS fix of a CPM Lab vehicle.
 *
 * The receiver sits 7.5 cm behind the vehicle centre. Pseudo ranges to four virtual satellites are
 * disturbed by normally distributed clock, ionospheric and ephemeris errors (95 % within the
 * scaled real-world error), averaged over the frequency bands and trilaterated.
 */
class UFIL_EXAMPLES_CPM_LAB_ADAPTER_PUBLIC ArtificialGnss
{
public:
  /**
   * @param seed seed of the random errors; std::nullopt draws one from std::random_device.
   * @throws std::invalid_argument if the configuration is invalid.
   */
  explicit ArtificialGnss(
    const ArtificialGnssConfig & config = {}, std::optional<unsigned int> seed = std::nullopt);

  /**
   * @param position position of the vehicle centre in the lab [m], z is ignored.
   * @param yaw heading of the vehicle [rad].
   * @return the disturbed position of the receiver in the lab [m], z is 0.
   */
  Eigen::Vector3d generateFix(const Eigen::Vector3d & position, double yaw);

  const ArtificialGnssConfig & config() const {return config_;}

private:
  double scaledRandomError(double real_world_error);

  ArtificialGnssConfig config_;
  std::default_random_engine random_engine_;
  std::array<Eigen::Vector3d, 4> satellite_positions_;
};

}  // namespace ufil_examples_cpm_lab_adapter

#endif  // UFIL_EXAMPLES_CPM_LAB_ADAPTER__ARTIFICIAL_GNSS_HPP_
