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

#ifndef UFIL_ETSI__ITS_TIME_HPP_
#define UFIL_ETSI__ITS_TIME_HPP_

// ITS time: milliseconds since 2004-01-01T00:00:00.000Z (TAI based, i.e. including the leap
// seconds since 2004). The CAM generation delta time is the ITS time modulo 65536.

#include <cstdint>
#include <optional>

#include "ufil_etsi/types.hpp"
#include "ufil_etsi/visibility_control.hpp"

namespace ufil_etsi
{

/// 2004-01-01T00:00:00.000Z in Unix time.
inline constexpr Time kItsEpoch = std::chrono::seconds(1072915200);

/// Leap seconds inserted since 2004 (the last one at the end of 2016). Same value as used by
/// etsi_its_msgs_utils, so that the generation delta time matches other ITS stations.
inline constexpr std::chrono::seconds kLeapSecondsSince2004{5};

/// ITS time [ms] of a Unix time. Returns std::nullopt for times before 2004.
UFIL_ETSI_PUBLIC std::optional<std::uint64_t> itsTimeFromUnixTime(Time unix_time);

/// Generation delta time of a Unix time. Returns std::nullopt for times before 2004.
UFIL_ETSI_PUBLIC std::optional<std::uint16_t> generationDeltaTime(Time unix_time);

/**
 * @brief Reconstructs the Unix time of a generation delta time.
 *
 * The generation delta time wraps every 65.536 s; the result is the time with this generation
 * delta time that is closest to reference_time (normally the reception time), so CAMs generated
 * up to 32.768 s before or after the reference are reconstructed correctly, also across a wrap.
 * Returns std::nullopt if reference_time is before 2004.
 */
UFIL_ETSI_PUBLIC std::optional<Time> unixTimeFromGenerationDeltaTime(
  std::uint16_t generation_delta_time, Time reference_time);

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__ITS_TIME_HPP_
