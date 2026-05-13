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


#ifndef UFIL_EXAMPLES_CPM_LAB_ADAPTER__MESSAGE_CONVERTER_HPP_
#define UFIL_EXAMPLES_CPM_LAB_ADAPTER__MESSAGE_CONVERTER_HPP_

#include <string>
#include <cpm_lab_map_msgs/msg/lane_control_light.hpp>
#include <cpm_lab_map_msgs/msg/lane_control_light_segment.hpp>

namespace ufil_examples_cpm_lab_adapter
{

std::string to_json(const cpm_lab_map_msgs::msg::LaneControlLight & msg);


}  // namespace ufil_examples_cpm_lab_adapter

#endif  // UFIL_EXAMPLES_CPM_LAB_ADAPTER__MESSAGE_CONVERTER_HPP_
