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

#ifndef UFIL_OBJECT_TRACKING__MANAGEMENT__PRUNER__TIMED_PRUNER_HPP_
#define UFIL_OBJECT_TRACKING__MANAGEMENT__PRUNER__TIMED_PRUNER_HPP_

#include <algorithm>
#include <execution>
#include <map>
#include <memory>

#include "ufil_object_tracking/management/pruner/pruner.hpp"
#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace management
{

/**
 * @class TimedPruner
 * @brief Prunes historical data stored in tracks based on a specified time threshold.
 *
 * @tparam T The type of track being managed.
 */
template<typename T>
class TimedPruner : public Pruner<T>
{
private:
  type::Duration prune_time_ = std::chrono::seconds::max();

public:
  using UniquePtr = std::unique_ptr<TimedPruner<T>>;
  using SharedPtr = std::shared_ptr<TimedPruner<T>>;

  /**
   * @brief Constructs a TimedPruner with a specified pruning duration.
   * @param prune_time The duration after which historical data is pruned from the track.
   */
  explicit TimedPruner(const type::Duration & prune_time = std::chrono::seconds(9999))
  : prune_time_(prune_time)
  {
  }

  /**
   * @brief Sets the pruning duration for track history.
   * @param prune_time The new pruning duration.
   */
  void setPruneTime(const type::Duration & prune_time)
  {
    this->prune_time_ = prune_time;
  }

  /**
   * @brief Prunes outdated historical data from tracks.
   * @param timestamp The current timestamp.
   * @param tracks The map of track IDs to track objects.
   */
  void pruneTrackHistory(const type::Timestamp & timestamp, std::map<type::Id, T> & tracks) override
  {
    std::for_each(std::execution::par, tracks.begin(), tracks.end(),
      [&timestamp, this](auto & track_pair) {
        auto & [id, track] = track_pair;
        auto prune_threshold = timestamp - this->prune_time_;
        track.pruneBefore(prune_threshold);
    });
  }
};

}  // namespace management
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MANAGEMENT__PRUNER__TIMED_PRUNER_HPP_
