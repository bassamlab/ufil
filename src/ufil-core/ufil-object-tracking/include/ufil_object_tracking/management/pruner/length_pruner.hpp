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

#ifndef UFIL_OBJECT_TRACKING__MANAGEMENT__PRUNER__LENGTH_PRUNER_HPP_
#define UFIL_OBJECT_TRACKING__MANAGEMENT__PRUNER__LENGTH_PRUNER_HPP_

#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>

#include "ufil_object_tracking/management/pruner/pruner.hpp"

namespace ufil
{
namespace management
{

/**
 * @class LengthPruner
 * @brief Prunes historical data stored in tracks based on a maximum number of entries.
 *
 * @tparam T The type of track being managed.
 */
template<typename T>
class LengthPruner : public Pruner<T>
{
private:
  std::size_t max_length_ = std::numeric_limits<std::size_t>::max();

public:
  using UniquePtr = std::unique_ptr<LengthPruner<T>>;
  using SharedPtr = std::shared_ptr<LengthPruner<T>>;

  /**
   * @brief Constructs a LengthPruner with a specified maximum history length.
   * @param max_length Maximum number of entries to keep in each track history.
   * @throws std::invalid_argument if max_length is zero.
   */
  explicit LengthPruner(const std::size_t max_length = std::numeric_limits<std::size_t>::max())
  {
    if (max_length == 0) {
      throw std::invalid_argument("LengthPruner max_length must be >= 1.");
    }
    this->max_length_ = max_length;
  }

  /**
   * @brief Prunes track histories to the configured maximum number of entries.
   * @param timestamp The current timestamp (unused for length-based pruning).
   * @param tracks The map of track IDs to track objects.
   */
  void pruneTrackHistory(const type::Timestamp & timestamp, std::map<type::Id, T> & tracks) override
  {
    (void)timestamp;

    for (auto & track_pair : tracks) {
      auto & track = track_pair.second;
      auto & history = track.history();

      if (history.size() > this->max_length_) {
        const auto excess_entries = history.size() - this->max_length_;
        auto first = history.begin();
        auto last = std::next(first, excess_entries);

        history.erase(first, last);
      }
    }
  }
};

}  // namespace management
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MANAGEMENT__PRUNER__LENGTH_PRUNER_HPP_
