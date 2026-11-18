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

#ifndef SSL_WHEEL_TRACKER__WHEEL_DETECTOR_HPP_
#define SSL_WHEEL_TRACKER__WHEEL_DETECTOR_HPP_

#include <vector>

#include "ssl_wheel_tracker/config.hpp"
#include "ssl_wheel_tracker/types.hpp"
#include "ssl_wheel_tracker/visibility_control.hpp"

namespace ssl_wheel_tracker
{

/**
 * @brief Detects wheel contacts in the pressure grid of a sensitive surface layer (SSL).
 *
 * 1. Local maxima: cells above the threshold that are the maximum within a square window of
 *    radius min_peak_distance and do not lie in the excluded border.
 * 2. Every cell within the L1 distance contact_radius of a maximum is assigned to the nearest
 *    maximum (Euclidean); all other cells form the background.
 * 3. A wheel is located at the pressure-weighted center of its cells. Its pressure is the sum
 *    of the pressure of its cells above the mean background pressure.
 *
 * The class is stateless apart from its configuration and thread-safe.
 */
class SSL_WHEEL_TRACKER_PUBLIC WheelDetector
{
public:
  /// @throws std::invalid_argument if the configuration is invalid.
  explicit WheelDetector(const DetectorConfig & config);

  /// Wheels in the frame of the grid origin, ordered by their first cell (column-major).
  std::vector<WheelDetection> detect(const PressureGrid & grid) const;

  const DetectorConfig & config() const {return config_;}

private:
  DetectorConfig config_;
};

/// Local maxima of the grid as described in WheelDetector, ordered by decreasing pressure.
SSL_WHEEL_TRACKER_PUBLIC std::vector<Cell> findPeaks(
  const PressureGrid & grid, const DetectorConfig & config);

}  // namespace ssl_wheel_tracker

#endif  // SSL_WHEEL_TRACKER__WHEEL_DETECTOR_HPP_
