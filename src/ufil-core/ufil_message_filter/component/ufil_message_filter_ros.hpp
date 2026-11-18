// Copyright 2026 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#ifndef UFIL_MESSAGE_FILTER__COMPONENT__UFIL_MESSAGE_FILTER_ROS_HPP_
#define UFIL_MESSAGE_FILTER__COMPONENT__UFIL_MESSAGE_FILTER_ROS_HPP_

#include <cstddef>
#include <vector>

#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_message_filter/position_filter.hpp"

/// Conversions between ufil_msgs and the types of the core library.
namespace ufil_message_filter::conversions
{

inline Position positionFromMsg(const ufil_msgs::msg::Object & object)
{
  return Position{object.state.state.x, object.state.state.y};
}

/// @return the positions of the objects, in the order of msg.objects.
inline std::vector<Position> positionsFromMsg(const ufil_msgs::msg::ObjectList & msg)
{
  std::vector<Position> positions;
  positions.reserve(msg.objects.size());
  for (const auto & object : msg.objects) {
    positions.push_back(positionFromMsg(object));
  }
  return positions;
}

/**
 * @brief Builds the filtered object list.
 * @param indices indices into msg.objects, as returned by PositionFilter::filter().
 * @return the header of msg and the selected objects, unchanged.
 */
inline ufil_msgs::msg::ObjectList selectObjects(
  const ufil_msgs::msg::ObjectList & msg, const std::vector<std::size_t> & indices)
{
  ufil_msgs::msg::ObjectList result;
  result.header = msg.header;
  result.objects.reserve(indices.size());
  for (const auto index : indices) {
    result.objects.push_back(msg.objects.at(index));
  }
  return result;
}

}  // namespace ufil_message_filter::conversions

#endif  // UFIL_MESSAGE_FILTER__COMPONENT__UFIL_MESSAGE_FILTER_ROS_HPP_
