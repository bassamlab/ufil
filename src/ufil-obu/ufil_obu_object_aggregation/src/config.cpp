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


#include "ufil_obu_object_aggregation/config.hpp"

#include <cmath>
#include <optional>
#include <string>

namespace ufil_obu_object_aggregation
{

std::optional<std::string> validate(const AggregatorConfig & config)
{
  if (!std::isfinite(config.object_length) || config.object_length <= 0.0) {
    return "object length must be > 0";
  }
  if (!std::isfinite(config.object_width) || config.object_width <= 0.0) {
    return "object width must be > 0";
  }
  if (config.object_class < 0 || config.object_class >= static_cast<int>(kObjectClassCount)) {
    return "object class must be in [0, " + std::to_string(kObjectClassCount - 1) + "]";
  }
  if (config.utm_zone != kUtmZoneAutomatic && (config.utm_zone < 1 || config.utm_zone > 60)) {
    return "UTM zone must be in [1, 60] or -1 for automatic selection";
  }
  return std::nullopt;
}

}  // namespace ufil_obu_object_aggregation
