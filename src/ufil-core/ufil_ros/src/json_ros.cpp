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

#include "ufil_ros/json_ros.hpp"

namespace ufil_ros
{

constexpr char QUOTE = '"';
constexpr char COMMA = ',';
constexpr char EQUAL = ':';

template<int N>
std::string format_covariance(const std::array<double, N> & covariance, size_t rows, size_t cols)
{
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < rows; i++) {
    ss << "[";
    for (size_t j = 0; j < cols; j++) {
      ss << std::to_string(covariance[i * cols + j]);
      if (j < cols - 1) {
        ss << COMMA;
      }
    }
    ss << "]";
    if (i < rows - 1) {
      ss << COMMA;
    }
  }
  ss << "]";
  return ss.str();
}

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

std::string format_state(const ufil_msgs::msg::StateWithCovariance & state)
{
  std::stringstream ss;
  ss << QUOTE << "state" << QUOTE << EQUAL << "{";
  ss << QUOTE << "x" << QUOTE << EQUAL << std::to_string(state.state.x) << COMMA;
  ss << QUOTE << "y" << QUOTE << EQUAL << std::to_string(state.state.y) << COMMA;
  ss << QUOTE << "v_x" << QUOTE << EQUAL << std::to_string(state.state.v_x) << COMMA;
  ss << QUOTE << "v_y" << QUOTE << EQUAL << std::to_string(state.state.v_y) << COMMA;
  ss << QUOTE << "a_x" << QUOTE << EQUAL << std::to_string(state.state.a_x) << COMMA;
  ss << QUOTE << "a_y" << QUOTE << EQUAL << std::to_string(state.state.a_y) << COMMA;
  ss << QUOTE << "yaw" << QUOTE << EQUAL << std::to_string(state.state.yaw) << COMMA;
  ss << QUOTE << "yaw_rate" << QUOTE << EQUAL << std::to_string(state.state.yaw_rate) << COMMA;
  ss << QUOTE << "covariance" << QUOTE << EQUAL << format_covariance<64>(state.covariance, 8, 8);
  ss << "}";

  return ss.str();
}

std::string format_dimension(const ufil_msgs::msg::DimensionWithCovariance & dimension)
{
  std::stringstream ss;
  ss << QUOTE << "dimension" << QUOTE << EQUAL << "{";
  ss << QUOTE << "length" << QUOTE << EQUAL << std::to_string(dimension.dimension.length) << COMMA;
  ss << QUOTE << "width" << QUOTE << EQUAL << std::to_string(dimension.dimension.width) << COMMA;
  ss << QUOTE << "height" << QUOTE << EQUAL << std::to_string(dimension.dimension.height) << COMMA;
  ss << QUOTE << "covariance" << QUOTE << EQUAL << format_covariance<9>(dimension.covariance, 3, 3);
  ss << "}";

  return ss.str();
}

std::string format_classification(const ufil_msgs::msg::Classification & classification)
{
  std::stringstream ss;
  ss << QUOTE << "classification" << QUOTE << EQUAL << "{";
  ss << QUOTE << "car" << QUOTE << EQUAL
     << std::to_string(classification.classification[ufil_msgs::msg::Classification::CAR]) << COMMA;
  ss << QUOTE << "truck" << QUOTE << EQUAL
     << std::to_string(classification.classification[ufil_msgs::msg::Classification::TRUCK]) <<
    COMMA;
  ss << QUOTE << "motorcycle" << QUOTE << EQUAL
     << std::to_string(classification.classification[ufil_msgs::msg::Classification::MOTORCYCLE]) <<
    COMMA;
  ss << QUOTE << "bicycle" << QUOTE << EQUAL
     << std::to_string(classification.classification[ufil_msgs::msg::Classification::BICYCLE]) <<
    COMMA;
  ss << QUOTE << "pedestrian" << QUOTE << EQUAL
     << std::to_string(classification.classification[ufil_msgs::msg::Classification::PEDESTRIAN]) <<
    COMMA;
  ss << QUOTE << "stationary" << QUOTE << EQUAL
     << std::to_string(classification.classification[ufil_msgs::msg::Classification::STATIONARY]) <<
    COMMA;
  ss << QUOTE << "other" << QUOTE << EQUAL
     << std::to_string(classification.classification[ufil_msgs::msg::Classification::OTHER]);
  ss << "}";
  return ss.str();
}

std::string format_features(const ufil_msgs::msg::Object & object)
{
  std::stringstream ss;
  ss << QUOTE << "features" << QUOTE << EQUAL << "{";
  ss << QUOTE << "front_right" << QUOTE << EQUAL << (object.features.fr ? "true" : "false") <<
    COMMA;
  ss << QUOTE << "front_left" << QUOTE << EQUAL << (object.features.fl ? "true" : "false") << COMMA;
  ss << QUOTE << "rear_right" << QUOTE << EQUAL << (object.features.br ? "true" : "false") << COMMA;
  ss << QUOTE << "rear_left" << QUOTE << EQUAL << (object.features.bl ? "true" : "false") << COMMA;
  ss << QUOTE << "center" << QUOTE << EQUAL << (object.features.c ? "true" : "false") <<
    COMMA;
  ss << QUOTE << "front_center" << QUOTE << EQUAL << (object.features.f ? "true" : "false") <<
    COMMA;
  ss << QUOTE << "rear_center" << QUOTE << EQUAL << (object.features.b ? "true" : "false") <<
    COMMA;
  ss << QUOTE << "right_center" << QUOTE << EQUAL << (object.features.r ? "true" : "false") <<
    COMMA;
  ss << QUOTE << "left_center" << QUOTE << EQUAL << (object.features.l ? "true" : "false");
  ss << "}";
  return ss.str();
}

std::string format_axles(const std::vector<ufil_msgs::msg::Axle> & axles)
{
  std::stringstream ss;
  ss << QUOTE << "axles" << QUOTE << EQUAL << "[";
  for (size_t i = 0; i < axles.size(); i++) {
    const auto & axle = axles[i];
    ss << "{";
    ss << QUOTE << "track_width" << QUOTE << EQUAL << std::to_string(axle.track_width) << COMMA;
    ss << QUOTE << "track_width_variance" << QUOTE <<
      EQUAL + std::to_string(axle.track_width_variance) << COMMA;
    ss << QUOTE << "center_to_axle" << QUOTE << EQUAL << std::to_string(axle.center_to_axle) <<
      COMMA;
    ss << QUOTE << "center_to_axle_variance" << QUOTE << EQUAL <<
      std::to_string(axle.center_to_axle_variance) << COMMA;
    ss << QUOTE << "wheel_detected" << QUOTE << EQUAL << "[";

    for (size_t j = 0; j < axle.wheel_detected.size(); j++) {
      ss << (axle.wheel_detected[i] ? "true" : "false");
      if (j < axle.wheel_detected.size() - 1) {
        ss << COMMA;
      }
    }
    ss << "]";

    ss << "}";
    if (i < axles.size() - 1) {
      ss << COMMA;
    }
  }
  ss << "]";

  return ss.str();
}

std::string format_object(const ufil_msgs::msg::Object & object)
{
  std::stringstream ss;
  ss << "{";
  ss << QUOTE << "id" << QUOTE << EQUAL << std::to_string(object.id) << COMMA;
  ss << QUOTE << "existence_probability" << QUOTE << EQUAL <<
    std::to_string(object.existence_probability) << COMMA;
  ss << format_state(object.state) << COMMA;
  ss << format_dimension(object.dimension) << COMMA;
  ss << format_classification(object.classification) << COMMA;
  ss << format_features(object) << COMMA;
  ss << format_axles(object.axles);
  ss << "}";
  return ss.str();
}

std::string to_json(const ufil_msgs::msg::ObjectList & msg)
{
  std::stringstream ss;
  ss << "{";
  ss << QUOTE << "object_list" << QUOTE << EQUAL << "{";
  ss << format_header(msg.header) << COMMA;
  ss << QUOTE << "objects" << QUOTE << EQUAL << "[";
  for (size_t i = 0; i < msg.objects.size(); i++) {
    ss << format_object(msg.objects[i]);
    if (i < msg.objects.size() - 1) {
      ss << COMMA;
    }
  }
  ss << "]";
  ss << "}";
  ss << "}";
  return ss.str();
}

std::string to_json(const nav_msgs::msg::OccupancyGrid & msg)
{
  std::stringstream ss;

  ss << "{";
  ss << QUOTE << "occupancy_grid" << QUOTE << EQUAL << "{";

  // Header
  ss << format_header(msg.header) << COMMA;

  // Info
  ss << QUOTE << "info" << QUOTE << EQUAL << "{";
  ss << QUOTE << "resolution" << QUOTE << EQUAL << std::to_string(msg.info.resolution) << COMMA;
  ss << QUOTE << "width" << QUOTE << EQUAL << std::to_string(msg.info.width) << COMMA;
  ss << QUOTE << "height" << QUOTE << EQUAL << std::to_string(msg.info.height) << COMMA;

  ss << QUOTE << "origin" << QUOTE << EQUAL << "{";
  ss << QUOTE << "position" << QUOTE << EQUAL << "{";
  ss << QUOTE << "x" << QUOTE << EQUAL << std::to_string(msg.info.origin.position.x) << COMMA;
  ss << QUOTE << "y" << QUOTE << EQUAL << std::to_string(msg.info.origin.position.y) << COMMA;
  ss << QUOTE << "z" << QUOTE << EQUAL << std::to_string(msg.info.origin.position.z);
  ss << "}" << COMMA;

  ss << QUOTE << "orientation" << QUOTE << EQUAL << "{";
  ss << QUOTE << "x" << QUOTE << EQUAL << std::to_string(msg.info.origin.orientation.x) << COMMA;
  ss << QUOTE << "y" << QUOTE << EQUAL << std::to_string(msg.info.origin.orientation.y) << COMMA;
  ss << QUOTE << "z" << QUOTE << EQUAL << std::to_string(msg.info.origin.orientation.z) << COMMA;
  ss << QUOTE << "w" << QUOTE << EQUAL << std::to_string(msg.info.origin.orientation.w);
  ss << "}";

  ss << "}";  // origin
  ss << "}" << COMMA;  // info

  // Data
  ss << QUOTE << "data" << QUOTE << EQUAL << "[";
  for (size_t i = 0; i < msg.data.size(); i++) {
    ss << std::to_string(msg.data[i]);
    if (i < msg.data.size() - 1) {
      ss << COMMA;
    }
  }
  ss << "]";

  ss << "}";  // occupancy_grid
  ss << "}";

  return ss.str();
}

}  // namespace ufil_ros
