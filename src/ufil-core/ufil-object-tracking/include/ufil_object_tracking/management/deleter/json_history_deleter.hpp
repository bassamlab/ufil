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

#ifndef UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__JSON_HISTORY_DELETER_HPP_
#define UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__JSON_HISTORY_DELETER_HPP_

#include <map>
#include <ostream>
#include <utility>

#include "ufil_object_tracking/management/deleter/deleter.hpp"
#include "ufil_object_tracking/management/deleter/existence_deleter.hpp"
#include "ufil_object_tracking/management/deleter/timed_deleter.hpp"
#include "ufil_object_tracking/serialization/json.hpp"

namespace ufil
{
namespace management
{

template<typename T, typename BaseDeleter = ExistenceDeleter<T>>
class JsonHistoryDeleter : public Deleter<T>
{
public:
  using TrackType = T;
  using BaseDeleterType = BaseDeleter;

  explicit JsonHistoryDeleter(
    std::ostream & output,
    BaseDeleterType base_deleter = BaseDeleterType(),
    bool include_history = true)
  : base_deleter_(std::move(base_deleter))
    , output_(output)
    , include_history_(include_history)
  {
  }

  bool checkForDeletion(const type::Timestamp & timestamp, TrackType & track) override
  {
    return base_deleter_.checkForDeletion(timestamp, track);
  }

protected:
  void beginDeletionBatch(
    const type::Timestamp & timestamp,
    const std::map<type::Id, TrackType> & tracks) override
  {
    (void)tracks;
    batch_timestamp_ = timestamp;
    started_ = false;
  }

  void beforeDeletion(const type::Timestamp &, const TrackType & track) override
  {
    if (!started_) {
      output_ << "{\n";
      output_ << "  \"deleted_at_ns\": " << ufil::to_nanoseconds(batch_timestamp_) << ",\n";
      output_ << "  \"deleted_tracks\": [\n";
      started_ = true;
    } else {
      output_ << ",\n";
    }

    ufil::serialization::write_json(track, output_, include_history_, 4);
  }

  void endDeletionBatch(const type::Timestamp &) override
  {
    if (!started_) {
      return;
    }

    output_ << "\n  ]\n";
    output_ << "}\n";
    output_.flush();
  }

private:
  BaseDeleterType base_deleter_;
  std::ostream & output_;
  bool include_history_{true};
  type::Timestamp batch_timestamp_{};
  bool started_{false};
};

template<typename T>
using JsonHistoryExistenceDeleter = JsonHistoryDeleter<T, ExistenceDeleter<T>>;

template<typename T>
using JsonHistoryTimedDeleter = JsonHistoryDeleter<T, TimedDeleter<T>>;

}  // namespace management
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__JSON_HISTORY_DELETER_HPP_
