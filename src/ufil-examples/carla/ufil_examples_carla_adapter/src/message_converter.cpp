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

#include "ufil_examples_carla_adapter/message_converter.hpp"

#include <cmath>
#include <algorithm>
#include <cctype>

namespace ufil_examples_carla_adapter
{
namespace
{

static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

static uint8_t getClassification(const carla_msgs::msg::CarlaActorInfo::_type_type & type)
{
  if (type == "vehicle.tesla.model3") {
    return ufil_msgs::msg::Classification::CAR;
  } else if (type == "vehicle.kawasaki.ninja") {
    return ufil_msgs::msg::Classification::MOTORCYCLE;
  } else if (type == "vehicle.carlamotors.european_hgv") {
    return ufil_msgs::msg::Classification::TRUCK;
  }
  return ufil_msgs::msg::Classification::OTHER;
}

static int64_t getVehicleIdFromObjectId(
  const carla_msgs::msg::CarlaActorList & carla_actor_list,
  int64_t object_id)
{
  const auto actor_info = std::find_if(
    carla_actor_list.actors.begin(), carla_actor_list.actors.end(),
    [object_id](const carla_msgs::msg::CarlaActorInfo & actor_info) {
      return actor_info.id == object_id;
    });

  if (actor_info == carla_actor_list.actors.end()) {
    return -1;
  }

  const std::string prefix = "vehicle_";
  if (!actor_info->rolename.starts_with(prefix)) {
    return -1;
  }

  std::string id_str = actor_info->rolename.substr(prefix.size());
  if (id_str.empty()) {
    return -1;
  }

  if (!std::all_of(id_str.begin(), id_str.end(), [](unsigned char c) {return std::isdigit(c);})) {
    return -1;
  }

  return std::stoll(id_str);
}

}  // namespace

ufil_msgs::msg::ObjectList objectsFromCarla(
  const derived_object_msgs::msg::ObjectArray & object_array,
  const carla_msgs::msg::CarlaActorList & carla_actor_list,
  const std::vector<int64_t> & vehicle_ids,
  int n_vehicles,
  const std::map<std::string, std::vector<ufil_msgs::msg::Axle>> & axle_geometries)
{
  ufil_msgs::msg::ObjectList object_list;
  object_list.header = object_array.header;

  for (const auto & derived_object : object_array.objects) {
    const auto vehicle_id = getVehicleIdFromObjectId(carla_actor_list, derived_object.id);

    if (n_vehicles != -1 && (vehicle_id == -1 ||
      std::find(vehicle_ids.begin(), vehicle_ids.end(), vehicle_id) == vehicle_ids.end()))
    {
      continue;
    }

    ufil_msgs::msg::Object object;

    const auto actor_info_it = std::find_if(
      carla_actor_list.actors.begin(), carla_actor_list.actors.end(),
      [id = derived_object.id](const carla_msgs::msg::CarlaActorInfo & actor_info) {
        return actor_info.id == id;
      });

    object.id = derived_object.id;

    const auto yaw = yawFromQuaternion(derived_object.pose.orientation);

    object.state.state.x = derived_object.pose.position.x;
    object.state.state.y = derived_object.pose.position.y;
    object.state.state.v_x = derived_object.twist.linear.x;
    object.state.state.v_y = derived_object.twist.linear.y;
    object.state.state.a_x = derived_object.accel.linear.x;
    object.state.state.a_y = derived_object.accel.linear.y;
    object.state.state.yaw = yaw;
    object.state.state.yaw_rate = derived_object.twist.angular.z;

    object.state.covariance[0] = 0.9;
    object.state.covariance[9] = 0.9;
    object.state.covariance[18] = 1.4;
    object.state.covariance[27] = 1.4;
    object.state.covariance[36] = 2.0;
    object.state.covariance[45] = 2.0;
    object.state.covariance[54] = 0.05;
    object.state.covariance[63] = 0.05;

    object.dimension.dimension.length = derived_object.shape.dimensions[0];
    object.dimension.dimension.width = derived_object.shape.dimensions[1];
    object.dimension.dimension.height = derived_object.shape.dimensions[2];

    object.dimension.covariance[0] = 0.01;
    object.dimension.covariance[4] = 0.01;
    object.dimension.covariance[8] = 0.01;

    object.existence_probability = 1.0;

    if (actor_info_it == carla_actor_list.actors.end()) {
      object.classification.classification[ufil_msgs::msg::Classification::OTHER] = 1;
    } else {
      object.classification.classification[getClassification(actor_info_it->type)] = 1;
    }

    object.features.fl = true;
    object.features.fr = true;
    object.features.bl = true;
    object.features.br = true;
    object.features.f = true;
    object.features.b = true;
    object.features.l = true;
    object.features.r = true;
    object.features.c = true;

    if (actor_info_it != carla_actor_list.actors.end()) {
      const auto & type = actor_info_it->type;
      if (auto axle_it = axle_geometries.find(type); axle_it != axle_geometries.end()) {
        object.axles = axle_it->second;
      }
    }

    object_list.objects.push_back(object);
  }

  return object_list;
}

ufil_msgs::msg::ObjectList transformObjectList(
  const ufil_msgs::msg::ObjectList & object_list,
  const geometry_msgs::msg::TransformStamped & transform_msg)
{
  ufil_msgs::msg::ObjectList transformed_object_list;
  transformed_object_list.header = object_list.header;
  transformed_object_list.header.frame_id = transform_msg.header.frame_id;

  const double yaw = yawFromQuaternion(transform_msg.transform.rotation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  for (const auto & object_in : object_list.objects) {
    auto object_out = object_in;

    const double x = object_in.state.state.x;
    const double y = object_in.state.state.y;
    object_out.state.state.x = c * x - s * y + transform_msg.transform.translation.x;
    object_out.state.state.y = s * x + c * y + transform_msg.transform.translation.y;

    const double v_x = object_in.state.state.v_x;
    const double v_y = object_in.state.state.v_y;
    object_out.state.state.v_x = c * v_x - s * v_y;
    object_out.state.state.v_y = s * v_x + c * v_y;

    const double a_x = object_in.state.state.a_x;
    const double a_y = object_in.state.state.a_y;
    object_out.state.state.a_x = c * a_x - s * a_y;
    object_out.state.state.a_y = s * a_x + c * a_y;

    object_out.state.state.yaw = object_in.state.state.yaw + yaw;

    transformed_object_list.objects.push_back(object_out);
  }

  return transformed_object_list;
}

}  // namespace ufil_examples_carla_adapter
