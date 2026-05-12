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

#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>

#include "ufil_examples_cpm_lab_adapter/message_converter.hpp"

constexpr char QUOTE = '"';
constexpr char COMMA = ',';
constexpr char EQUAL = ':';

namespace ufil_examples_cpm_lab_adapter
{

std::string format_header(const std_msgs::msg::Header & header)
{
  std::stringstream ss;
  rclcpp::Time time = header.stamp;
  ss << QUOTE << "header" << QUOTE << EQUAL << "{";
  ss << QUOTE << "seconds" << QUOTE << EQUAL << std::to_string(time.seconds()) << COMMA;
  ss << QUOTE << "frame_id" << QUOTE << EQUAL << QUOTE << header.frame_id << QUOTE;
  ss << "}";
  return ss.str();
}

std::string format_lane_control_light_segment(
  const cpm_lab_map_msgs::msg::LaneControlLightSegment & segment)
{
  bool image_name_exist = segment.image_name != "";
  std::string empty_sign = "-";
  std::stringstream ss;
  ss << "{";
  ss << QUOTE << "segment_index" << QUOTE << EQUAL
     << std::to_string(segment.segment_index) << COMMA;
  ss << QUOTE << "image_name" << QUOTE << EQUAL
     << QUOTE << (image_name_exist ? segment.image_name : empty_sign) << QUOTE;
  ss << "}";
  return ss.str();
}

std::string to_json(const cpm_lab_map_msgs::msg::LaneControlLight & msg)
{
  std::stringstream ss;

  ss << "{";
  ss << QUOTE << "lane_control_light" << QUOTE << EQUAL << "{";

  // Header
  ss << format_header(msg.header) << COMMA;

  // Segments
  ss << QUOTE << "segments" << QUOTE << EQUAL << "[";

  for (size_t i = 0; i < msg.segments.size(); i++) {
    ss << format_lane_control_light_segment(msg.segments[i]);
    if (i < msg.segments.size() - 1) {
      ss << COMMA;
    }
  }

  ss << "]";

  ss << "}";  // lane_control_light
  ss << "}";

  return ss.str();
}

}  // namespace ufil_examples_cpm_lab_adapter
