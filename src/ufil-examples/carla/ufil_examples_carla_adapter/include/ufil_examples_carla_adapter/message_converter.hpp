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

#ifndef UFIL_EXAMPLES_CARLA_ADAPTER__MESSAGE_CONVERTER_HPP_
#define UFIL_EXAMPLES_CARLA_ADAPTER__MESSAGE_CONVERTER_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <carla_msgs/msg/carla_actor_list.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <derived_object_msgs/msg/object_array.hpp>
#include <ufil_msgs/msg/object_list.hpp>

namespace ufil_examples_carla_adapter
{

ufil_msgs::msg::ObjectList objectsFromCarla(
  const derived_object_msgs::msg::ObjectArray & object_array,
  const carla_msgs::msg::CarlaActorList & carla_actor_list,
  const std::vector<int64_t> & vehicle_ids,
  int n_vehicles,
  const std::map<std::string, std::vector<ufil_msgs::msg::Axle>> & axle_geometries);

ufil_msgs::msg::ObjectList transformObjectList(
  const ufil_msgs::msg::ObjectList & object_list,
  const geometry_msgs::msg::TransformStamped & transform_msg);

}  // namespace ufil_examples_carla_adapter

#endif  // UFIL_EXAMPLES_CARLA_ADAPTER__MESSAGE_CONVERTER_HPP_
