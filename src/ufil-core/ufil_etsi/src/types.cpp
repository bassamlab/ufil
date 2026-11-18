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

#include "ufil_etsi/types.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>
#include <string>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/UTMUPS.hpp>

namespace ufil_etsi
{

ObjectClass mostLikelyClass(const Object & object)
{
  const auto & classes = object.classification;
  const auto best = std::max_element(classes.begin(), classes.end());
  if (*best <= 0.0) {
    return ObjectClass::kOther;
  }
  return static_cast<ObjectClass>(std::distance(classes.begin(), best));
}

std::optional<UtmZone> utmZoneFromFrame(const std::string & frame_id)
{
  const std::string prefix = "utm_";
  if (frame_id.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }
  const std::string zone_string = frame_id.substr(prefix.size());
  // Only UTM zones with an explicit hemisphere, e.g. "32n" (UPS and MGRS bands are rejected).
  if (zone_string.size() < 2 || zone_string.size() > 3) {
    return std::nullopt;
  }
  const char hemisphere = zone_string.back();
  if (hemisphere != 'n' && hemisphere != 's' && hemisphere != 'N' && hemisphere != 'S') {
    return std::nullopt;
  }
  if (!std::all_of(
      zone_string.begin(), zone_string.end() - 1, [](char c) {return c >= '0' && c <= '9';}))
  {
    return std::nullopt;
  }
  const int zone = std::stoi(zone_string.substr(0, zone_string.size() - 1));
  if (zone < GeographicLib::UTMUPS::MINUTMZONE || zone > GeographicLib::UTMUPS::MAXUTMZONE) {
    return std::nullopt;
  }
  return UtmZone{zone, hemisphere == 'n' || hemisphere == 'N'};
}

std::string frameFromUtmZone(const UtmZone & zone)
{
  return "utm_" + std::to_string(zone.zone) + (zone.northern ? "n" : "s");
}

std::optional<GeoPosition> utmToGeo(double easting, double northing, UtmZone zone)
{
  if (!std::isfinite(easting) || !std::isfinite(northing)) {
    return std::nullopt;
  }
  GeoPosition result;
  try {
    GeographicLib::UTMUPS::Reverse(
      zone.zone, zone.northern, easting, northing, result.latitude, result.longitude);
  } catch (const GeographicLib::GeographicErr &) {
    return std::nullopt;
  }
  return result;
}

std::optional<UtmPosition> geoToUtm(const GeoPosition & position)
{
  if (!std::isfinite(position.latitude) || !std::isfinite(position.longitude)) {
    return std::nullopt;
  }
  UtmPosition result;
  try {
    GeographicLib::UTMUPS::Forward(
      position.latitude, position.longitude, result.zone.zone, result.zone.northern,
      result.easting, result.northing);
  } catch (const GeographicLib::GeographicErr &) {
    return std::nullopt;
  }
  if (result.zone.zone == GeographicLib::UTMUPS::UPS) {
    // Polar regions use UPS, which has no UFIL frame.
    return std::nullopt;
  }
  return result;
}

}  // namespace ufil_etsi
