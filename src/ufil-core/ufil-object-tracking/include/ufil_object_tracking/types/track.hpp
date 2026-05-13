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

#ifndef UFIL_OBJECT_TRACKING__TYPES__TRACK_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__TRACK_HPP_

#include <map>

#include "ufil_object_tracking/types/existence_probability.hpp"
#include "ufil_object_tracking/types/history_entry.hpp"
#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/time.hpp"


namespace ufil
{
namespace type
{

template<typename S, typename D, typename C, typename E, typename I, typename M>
class Track : public IdInterface
{
public:
  using StateType = S;
  using DimensionType = D;
  using ClassificationType = C;
  using ExistenceProbabilityType = E;
  using ControlType = I;
  using MeasurementType = M;

  using HistoryEntryType = HistoryEntry<S, D, C, E, I, M>;
  using HistoryType = std::map<type::Timestamp, HistoryEntryType>;

protected:
  HistoryType history_{};

  type::Timestamp creation_time_ = type::Timestamp::min();

public:
  Track()
  : IdInterface()
  {
  }

  [[nodiscard]] virtual bool valid() const
  {
    return !history_.empty();
  }

  [[nodiscard]] HistoryType & history()
  {
    return history_;
  }

  [[nodiscard]] const HistoryType & history() const
  {
    return history_;
  }

  [[nodiscard]] HistoryEntryType & currentHistoryEntry()
  {
    return this->history_.rbegin()->second;
  }

  [[nodiscard]] const HistoryEntryType & currentHistoryEntry() const
  {
    return this->history_.crbegin()->second;
  }

  [[nodiscard]] HistoryEntryType & previousHistoryEntry()
  {
    return (++this->history_.rbegin())->second;
  }

  [[nodiscard]] const HistoryEntryType & previousHistoryEntry() const
  {
    return (++history_.crbegin())->second;
  }

  [[nodiscard]] StateType & currentState()
  {
    return this->currentHistoryEntry().state();
  }

  [[nodiscard]] const StateType & currentState() const
  {
    return this->currentHistoryEntry().state();
  }

  [[nodiscard]] StateType & previousState()
  {
    return this->previousHistoryEntry().state();
  }

  [[nodiscard]] const StateType & previousState() const
  {
    return this->previousHistoryEntry().state();
  }

  [[nodiscard]] DimensionType & currentDimension()
  {
    return this->currentHistoryEntry().dimension();
  }

  [[nodiscard]] const DimensionType & currentDimension() const
  {
    return this->currentHistoryEntry().dimension();
  }

  [[nodiscard]] DimensionType & previousDimension()
  {
    return this->previousHistoryEntry().dimension();
  }

  [[nodiscard]] const DimensionType & previousDimension() const
  {
    return this->previousHistoryEntry().dimension();
  }

  [[nodiscard]] ClassificationType & currentClassification()
  {
    return this->currentHistoryEntry().classification();
  }

  [[nodiscard]] const ClassificationType & currentClassification() const
  {
    return this->currentHistoryEntry().classification();
  }

  [[nodiscard]] ClassificationType & previousClassification()
  {
    return this->previousHistoryEntry().classification();
  }

  [[nodiscard]] const ClassificationType & previousClassification() const
  {
    return this->previousHistoryEntry().classification();
  }

  [[nodiscard]] ExistenceProbabilityType & currentExistenceProbability()
  {
    return this->currentHistoryEntry().existenceProbability();
  }

  [[nodiscard]] const ExistenceProbabilityType & currentExistenceProbability() const
  {
    return this->currentHistoryEntry().existenceProbability();
  }

  [[nodiscard]] ExistenceProbabilityType & previousExistenceProbability()
  {
    return this->previousHistoryEntry().existenceProbability();
  }

  [[nodiscard]] const ExistenceProbabilityType & previousExistenceProbability() const
  {
    return this->previousHistoryEntry().existenceProbability();
  }

  [[nodiscard]] type::Timestamp lastUpdated() const
  {
    if (history_.empty()) {
      return type::Timestamp::min();
    }
    for (auto iter = history_.crbegin(); iter != history_.crend(); iter++) {
      auto & [time, entry] = *iter;
      if (!entry.hasAssociatedMeasurement()) {
        continue;
      }
      return time;
    }
    return type::Timestamp::min();
  }

  [[nodiscard]] type::Timestamp creationTime() const
  {
    return this->creation_time_;
  }

  [[nodiscard]] bool contains(const type::Timestamp & timestamp)
  {
    return history_.contains(timestamp);
  }

  [[nodiscard]] HistoryEntryType & at(const type::Timestamp & timestamp)
  {
    auto it = history_.find(timestamp);
    return it->second;
  }

  void pruneBefore(const type::Timestamp & timestamp)
  {
    std::erase_if(history_, [&timestamp](const auto & pair) {return pair.first <= timestamp;});
  }

  void pruneAfter(const type::Timestamp & timestamp)
  {
    if (history_.empty()) {
      return;
    }

    auto it = history_.lower_bound(timestamp);

    // Keep the first element if we'd erase everything
    if (it == history_.begin()) {
      ++it;
    }

    history_.erase(it, history_.end());
  }


  auto insert(HistoryEntryType && history_entry)
  {
    if (this->creation_time_ <= type::Timestamp::min()) {
      this->creation_time_ = history_entry.state().timestamp();
    }

    return history_.emplace(history_entry.state().timestamp(), history_entry);
  }

  auto erase(const type::Timestamp & timestamp)
  {
    return history_.erase(timestamp) > 0;
  }
};

}  // namespace type
}  // namespace ufil
#endif  // UFIL_OBJECT_TRACKING__TYPES__TRACK_HPP_
