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

#ifndef UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__DELETER_HPP_
#define UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__DELETER_HPP_

#include <cstddef>
#include <map>

#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace management
{

template<typename T>
class Deleter
{
public:
  virtual ~Deleter() = default;

  using TrackType = T;

  virtual bool checkForDeletion(const type::Timestamp & timestamp, TrackType & track) = 0;

protected:
  virtual void beginDeletionBatch(
    const type::Timestamp & timestamp,
    const std::map<type::Id, TrackType> & tracks)
  {
    (void)timestamp;
    (void)tracks;
  }

  virtual void beforeDeletion(const type::Timestamp & timestamp, const TrackType & track)
  {
    (void)timestamp;
    (void)track;
  }

  virtual void endDeletionBatch(const type::Timestamp & timestamp)
  {
    (void)timestamp;
  }

public:
  void deleteTracks(const type::Timestamp & timestamp, std::map<type::Id, TrackType> & tracks)
  {
    this->beginDeletionBatch(timestamp, tracks);

    for (auto it = tracks.begin(); it != tracks.end(); ) {
      auto & track = it->second;
      if (this->checkForDeletion(timestamp, track)) {
        this->beforeDeletion(timestamp, track);
        it = tracks.erase(it);
      } else {
        ++it;
      }
    }

    this->endDeletionBatch(timestamp);
  }
};

}  // namespace management
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__MANAGEMENT__DELETER__DELETER_HPP_
