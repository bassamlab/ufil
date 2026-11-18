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

#include "ufil_etsi/rsu_cam_generator.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

#include "ufil_etsi/its_time.hpp"

namespace ufil_etsi
{

std::optional<std::string> validate(const RsuCamGeneratorConfig & config)
{
  if (!(config.position.latitude >= -90.0 && config.position.latitude <= 90.0)) {
    return "the latitude must be in [-90, 90] deg";
  }
  if (!(config.position.longitude >= -180.0 && config.position.longitude <= 180.0)) {
    return "the longitude must be in [-180, 180] deg";
  }
  if (!(config.altitude >= etsi::kAltitudeMin / 100.0 &&
    config.altitude <= etsi::kAltitudeMax / 100.0))
  {
    return "the altitude must be in [-1000, 8000] m";
  }
  return std::nullopt;
}

std::uint32_t randomStationId()
{
  std::random_device random_device;
  std::mt19937 engine(random_device());
  std::uniform_int_distribution<std::uint32_t> distribution(
    0, std::numeric_limits<std::uint32_t>::max());
  return distribution(engine);
}

RsuCamGenerator::RsuCamGenerator(RsuCamGeneratorConfig config)
: config_(std::move(config))
{
  if (const auto error = validate(config_)) {
    throw std::invalid_argument(*error);
  }

  // Everything but the generation delta time is static.
  cam_.station_id = config_.station_id;
  cam_.station_type = etsi::station_type::kRoadSideUnit;
  auto & reference = cam_.reference_position;
  reference.latitude =
    static_cast<std::int32_t>(std::llround(config_.position.latitude * 1e7));
  reference.longitude =
    static_cast<std::int32_t>(std::llround(config_.position.longitude * 1e7));
  reference.altitude = static_cast<std::int32_t>(std::llround(config_.altitude * 100.0));
  // The position of a road side unit is surveyed.
  reference.altitude_confidence = etsi::kAltitudeConfidence1Cm;
  reference.semi_major_orientation = etsi::kHeadingNorth;
  reference.semi_major_confidence = etsi::kSemiAxisLengthMin;
  reference.semi_minor_confidence = etsi::kSemiAxisLengthMin;
}

std::optional<Cam> RsuCamGenerator::generate(Time now)
{
  const auto generation_delta_time = generationDeltaTime(now + config_.time_shift);
  if (!generation_delta_time) {
    return std::nullopt;
  }
  cam_.generation_delta_time = *generation_delta_time;
  ++generated_;
  return cam_;
}

}  // namespace ufil_etsi
