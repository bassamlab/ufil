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

#ifndef UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__TIMED_DELETER_HPP_
#define UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__TIMED_DELETER_HPP_

#include "ufil_object_tracking/management/deleter/deleter.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace management
{

/**
 * @class TimedDeleter
 * @brief Deletes tracks that have not been updated within a specified timeout period.
 *
 * @tparam T The type of track being managed.
 */
template<typename T>
class TimedDeleter : public Deleter<T>
{
public:
  using TrackType = T;

  /**
   * @brief Constructs a TimedDeleter with a specified timeout duration.
   * @param timeout The duration after which a track is considered stale and marked for deletion.
   */
  explicit TimedDeleter(const type::Duration & timeout = std::chrono::seconds(1))
  : timeout_(timeout)
  {
  }

  /**
   * @brief Checks if a track should be deleted based on its last update timestamp.
   * @param timestamp The current timestamp.
   * @param track The track to check for deletion.
   * @return True if the track has exceeded the timeout threshold, false otherwise.
   */
  bool checkForDeletion(const type::Timestamp & timestamp, TrackType & track) override
  {
    auto oldest_allowed_time = timestamp - timeout_;
    return track.lastUpdated() <= oldest_allowed_time;
  }

private:
  type::Duration timeout_;  ///< The timeout duration for track deletion.
};

}  // namespace management
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__TIMED_DELETER_HPP_
