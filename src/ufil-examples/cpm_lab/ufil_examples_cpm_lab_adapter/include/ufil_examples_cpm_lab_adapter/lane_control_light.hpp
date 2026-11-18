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

#ifndef UFIL_EXAMPLES_CPM_LAB_ADAPTER__LANE_CONTROL_LIGHT_HPP_
#define UFIL_EXAMPLES_CPM_LAB_ADAPTER__LANE_CONTROL_LIGHT_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "ufil_examples_cpm_lab_adapter/visibility_control.hpp"

namespace ufil_examples_cpm_lab_adapter
{

struct LaneControlLightSegment
{
  std::int32_t segment_index = 0;
  /// Image shown on the segment, empty if the segment is switched off.
  std::string image_name;
};

/// State of the lane control lights of a gantry, one entry per lane segment.
struct LaneControlLight
{
  /// Time stamp [s].
  double stamp = 0.0;
  std::string frame_id;
  std::vector<LaneControlLightSegment> segments;
};

/**
 * @brief Encodes the lane control lights as JSON.
 *
 * Format (without white space): {"lane_control_light":{"header":{"seconds":12.500000,
 * "frame_id":"map"},"segments":[{"segment_index":0,"image_name":"282-black"}, ...]}}.
 * Segments that are switched off have the image name "-". Strings are escaped as JSON strings.
 */
UFIL_EXAMPLES_CPM_LAB_ADAPTER_PUBLIC std::string toJson(const LaneControlLight & lights);

}  // namespace ufil_examples_cpm_lab_adapter

#endif  // UFIL_EXAMPLES_CPM_LAB_ADAPTER__LANE_CONTROL_LIGHT_HPP_
