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

#ifndef UFIL_OBJECT_TRACKING__TYPES__TIME_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__TIME_HPP_

#include <chrono>

namespace ufil
{

namespace type
{

using Timestamp = std::chrono::system_clock::time_point;
using Duration = std::chrono::system_clock::duration;

}  // namespace type

template<typename T>
inline T from_nanoseconds(int64_t nanoseconds) = delete;

template<typename T>
inline int64_t to_nanoseconds(const T & value) = delete;

template<>
inline type::Timestamp from_nanoseconds<type::Timestamp>(int64_t nanoseconds)
{
  return std::chrono::system_clock::time_point(std::chrono::nanoseconds(nanoseconds));
}

template<>
inline int64_t to_nanoseconds<type::Timestamp>(const type::Timestamp & timestamp)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count();
}

template<>
inline type::Duration from_nanoseconds<type::Duration>(int64_t nanoseconds)
{
  return std::chrono::nanoseconds(nanoseconds);
}

template<>
inline int64_t to_nanoseconds<type::Duration>(const type::Duration & duration)
{
  return duration.count();
}

template<typename T>
inline T from_seconds(double seconds) = delete;

template<typename T>
inline double to_seconds(const T & value) = delete;

template<>
inline type::Duration from_seconds<type::Duration>(double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(
      seconds));
}

template<>
inline type::Timestamp from_seconds<type::Timestamp>(double seconds)
{
  return std::chrono::system_clock::time_point(from_seconds<type::Duration>(seconds));
}

template<>
inline double to_seconds<type::Duration>(const type::Duration & duration)
{
  return std::chrono::duration<double>(duration).count();
}
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__TIME_HPP_
