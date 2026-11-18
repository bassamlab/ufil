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

#include "ufil_etsi/its_time.hpp"

#include <chrono>
#include <cstdint>
#include <optional>

namespace ufil_etsi
{

namespace
{

constexpr std::int64_t kGenerationDeltaTimeModulus = 65536;

}  // namespace

std::optional<std::uint64_t> itsTimeFromUnixTime(Time unix_time)
{
  const auto its_time = std::chrono::floor<std::chrono::milliseconds>(
    unix_time - kItsEpoch + kLeapSecondsSince2004);
  if (its_time.count() < 0) {
    return std::nullopt;
  }
  return static_cast<std::uint64_t>(its_time.count());
}

std::optional<std::uint16_t> generationDeltaTime(Time unix_time)
{
  const auto its_time = itsTimeFromUnixTime(unix_time);
  if (!its_time) {
    return std::nullopt;
  }
  return static_cast<std::uint16_t>(*its_time % kGenerationDeltaTimeModulus);
}

std::optional<Time> unixTimeFromGenerationDeltaTime(
  std::uint16_t generation_delta_time, Time reference_time)
{
  const auto reference = itsTimeFromUnixTime(reference_time);
  if (!reference) {
    return std::nullopt;
  }
  const auto reference_ms = static_cast<std::int64_t>(*reference);
  // Candidate in the same 65.536 s period as the reference, then the closest of its neighbours.
  std::int64_t its_time =
    reference_ms - reference_ms % kGenerationDeltaTimeModulus + generation_delta_time;
  if (its_time - reference_ms > kGenerationDeltaTimeModulus / 2) {
    its_time -= kGenerationDeltaTimeModulus;
  } else if (reference_ms - its_time > kGenerationDeltaTimeModulus / 2) {
    its_time += kGenerationDeltaTimeModulus;
  }
  return kItsEpoch - kLeapSecondsSince2004 + std::chrono::milliseconds(its_time);
}

}  // namespace ufil_etsi
