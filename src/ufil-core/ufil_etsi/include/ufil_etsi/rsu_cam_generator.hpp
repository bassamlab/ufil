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

#ifndef UFIL_ETSI__RSU_CAM_GENERATOR_HPP_
#define UFIL_ETSI__RSU_CAM_GENERATOR_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/types.hpp"
#include "ufil_etsi/visibility_control.hpp"

namespace ufil_etsi
{

/// Configuration of the RsuCamGenerator.
struct RsuCamGeneratorConfig
{
  std::uint32_t station_id = 0;
  /// Position of the road side unit.
  GeoPosition position;
  /// Altitude above the WGS84 ellipsoid [m].
  double altitude = 0.0;
  /// See VehicleCamGeneratorConfig::time_shift.
  Duration time_shift{0};
};

/// Returns a description of the first invalid value, std::nullopt if the configuration is valid.
UFIL_ETSI_PUBLIC std::optional<std::string> validate(const RsuCamGeneratorConfig & config);

/// Random station id, for stations without a configured one.
UFIL_ETSI_PUBLIC std::uint32_t randomStationId();

/**
 * @brief Generates the CAMs of a road side unit.
 *
 * The CAM is static except for the generation delta time: station type road side unit, the
 * configured position (1 cm confidence) and an RSU high-frequency container without protected
 * communication zones. ETSI EN 302 637-2 requires an interval of at least 1 s between two CAMs
 * of an RSU (kGenerationInterval).
 */
class UFIL_ETSI_PUBLIC RsuCamGenerator
{
public:
  static constexpr Duration kGenerationInterval = std::chrono::seconds(1);

  /// @throws std::invalid_argument if the configuration is invalid.
  explicit RsuCamGenerator(RsuCamGeneratorConfig config);

  /// CAM generated at the clock time now, std::nullopt if the shifted time is before 2004.
  std::optional<Cam> generate(Time now);

  const RsuCamGeneratorConfig & config() const {return config_;}
  std::uint64_t generated() const {return generated_;}

private:
  RsuCamGeneratorConfig config_;
  Cam cam_;
  std::uint64_t generated_ = 0;
};

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__RSU_CAM_GENERATOR_HPP_
