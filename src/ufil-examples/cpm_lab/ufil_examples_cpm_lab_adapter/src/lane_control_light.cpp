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

#include "ufil_examples_cpm_lab_adapter/lane_control_light.hpp"

#include <cstdio>
#include <sstream>
#include <string>

namespace ufil_examples_cpm_lab_adapter
{

namespace
{

/// Writes value as a quoted JSON string.
void writeString(std::ostream & out, const std::string & value)
{
  out << '"';
  for (const char c : value) {
    switch (c) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char escaped[7];
          std::snprintf(escaped, sizeof(escaped), "\\u%04x", static_cast<unsigned char>(c));
          out << escaped;
        } else {
          out << c;
        }
    }
  }
  out << '"';
}

}  // namespace

std::string toJson(const LaneControlLight & lights)
{
  std::ostringstream out;
  out << R"({"lane_control_light":{"header":{"seconds":)" << std::to_string(lights.stamp)
      << R"(,"frame_id":)";
  writeString(out, lights.frame_id);
  out << R"(},"segments":[)";
  for (std::size_t i = 0; i < lights.segments.size(); ++i) {
    const LaneControlLightSegment & segment = lights.segments[i];
    if (i > 0) {
      out << ',';
    }
    out << R"({"segment_index":)" << segment.segment_index << R"(,"image_name":)";
    writeString(out, segment.image_name.empty() ? "-" : segment.image_name);
    out << '}';
  }
  out << "]}}";
  return out.str();
}

}  // namespace ufil_examples_cpm_lab_adapter
