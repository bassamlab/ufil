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


#ifndef UFIL_OBU_OBJECT_AGGREGATION__CONFIG_HPP_
#define UFIL_OBU_OBJECT_AGGREGATION__CONFIG_HPP_

#include <cstddef>
#include <optional>
#include <string>

#include "ufil_obu_object_aggregation/visibility_control.hpp"

namespace ufil_obu_object_aggregation
{

/// Object classes, matching the indices of ufil_msgs/Classification.
enum class ObjectClass : int
{
  kCar = 0,
  kTruck = 1,
  kMotorcycle = 2,
  kBicycle = 3,
  kPedestrian = 4,
  kStationary = 5,
  kOther = 6,
};

constexpr std::size_t kObjectClassCount = 7;

/// Selects the UTM zone automatically from the position.
constexpr int kUtmZoneAutomatic = -1;

struct AggregatorConfig
{
  /// Extent of the own vehicle [m].
  double object_length = 4.5;
  double object_width = 1.8;
  /// Class of the own vehicle, see ObjectClass.
  int object_class = static_cast<int>(ObjectClass::kCar);
  /// UTM zone the position is expressed in (1 to 60), or kUtmZoneAutomatic for the standard
  /// zone of the current position.
  int utm_zone = kUtmZoneAutomatic;
};

/**
 * @brief Check a configuration for consistency.
 * @return A human readable description of the first problem found, or std::nullopt if valid.
 */
UFIL_OBU_OBJECT_AGGREGATION_PUBLIC
std::optional<std::string> validate(const AggregatorConfig & config);

}  // namespace ufil_obu_object_aggregation

#endif  // UFIL_OBU_OBJECT_AGGREGATION__CONFIG_HPP_
