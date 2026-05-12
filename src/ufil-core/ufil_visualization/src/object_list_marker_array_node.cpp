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

#include <tf2/LinearMath/Quaternion.h>

#include <numbers>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>


#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ufil_msgs/msg/classification.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_visualization/object_list_marker_array_parameters.hpp"

namespace
{

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using ufil_visualization::ObjectListMarkerArrayParameters;

struct Ellipse2D
{
  double axis_a{0.0};
  double axis_b{0.0};
  double angle{0.0};
};

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion result;
  result.x = q.x();
  result.y = q.y();
  result.z = q.z();
  result.w = q.w();
  return result;
}

std::string classificationToString(const ufil_msgs::msg::Classification & classification)
{
  std::array<std::pair<uint8_t, const char *>, 7> labels{{
    {ufil_msgs::msg::Classification::CAR, "CAR"},
    {ufil_msgs::msg::Classification::TRUCK, "TRUCK"},
    {ufil_msgs::msg::Classification::MOTORCYCLE, "MOTORCYCLE"},
    {ufil_msgs::msg::Classification::BICYCLE, "BICYCLE"},
    {ufil_msgs::msg::Classification::PEDESTRIAN, "PEDESTRIAN"},
    {ufil_msgs::msg::Classification::STATIONARY, "STATIONARY"},
    {ufil_msgs::msg::Classification::OTHER, "OTHER"},
  }};

  uint8_t best_index = ufil_msgs::msg::Classification::OTHER;
  float best_probability = -1.0F;
  for (const auto & [index, label] : labels) {
    (void)label;
    if (classification.classification[index] > best_probability) {
      best_probability = classification.classification[index];
      best_index = index;
    }
  }

  switch (best_index) {
    case ufil_msgs::msg::Classification::CAR:
      return "CAR";
    case ufil_msgs::msg::Classification::TRUCK:
      return "TRUCK";
    case ufil_msgs::msg::Classification::MOTORCYCLE:
      return "MOTORCYCLE";
    case ufil_msgs::msg::Classification::BICYCLE:
      return "BICYCLE";
    case ufil_msgs::msg::Classification::PEDESTRIAN:
      return "PEDESTRIAN";
    case ufil_msgs::msg::Classification::STATIONARY:
      return "STATIONARY";
    case ufil_msgs::msg::Classification::OTHER:
    default:
      return "OTHER";
  }
}

Ellipse2D ellipseFromCovariance(double c00, double c01, double c10, double c11)
{
  const double sym = 0.5 * (c01 + c10);
  const double trace = c00 + c11;
  const double determinant = c00 * c11 - sym * sym;
  const double discriminant = std::max(0.0, trace * trace * 0.25 - determinant);
  const double root = std::sqrt(discriminant);
  const double lambda_1 = std::max(0.0, trace * 0.5 + root);
  const double lambda_2 = std::max(0.0, trace * 0.5 - root);
  const double angle = 0.5 * std::atan2(2.0 * sym, c00 - c11);

  Ellipse2D ellipse;
  ellipse.axis_a = 2.0 * std::sqrt(lambda_1);
  ellipse.axis_b = 2.0 * std::sqrt(lambda_2);
  ellipse.angle = angle;
  return ellipse;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

Marker makeBaseMarker(
  const std_msgs::msg::Header & header, const std::string & ns, int32_t id, uint32_t type)
{
  Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 1.0F;
  return marker;
}

Marker makeDeleteMarker(
  const std_msgs::msg::Header & header, const std::string & ns, int32_t id, uint32_t type)
{
  Marker marker = makeBaseMarker(header, ns, id, type);
  marker.action = Marker::DELETE;
  return marker;
}

void appendEllipsePoints(
  Marker & marker, double cx, double cy, double cz, const Ellipse2D & ellipse,
  const std_msgs::msg::ColorRGBA & color, double line_width, double angle_offset = 0.0,
  int segments = 64)
{
  marker.type = Marker::LINE_STRIP;
  marker.scale.x = static_cast<float>(line_width);
  marker.color = color;

  for (int i = 0; i <= segments; ++i) {
    const double t = (2.0 * std::numbers::pi * static_cast<double>(i)) /
      static_cast<double>(segments);
    const double x = ellipse.axis_a * std::cos(t);
    const double y = ellipse.axis_b * std::sin(t);
    const double theta = ellipse.angle + angle_offset;
    const double rx = x * std::cos(theta) - y * std::sin(theta);
    const double ry = x * std::sin(theta) + y * std::cos(theta);
    marker.points.push_back(makePoint(cx + rx, cy + ry, cz));
  }
}

Marker makeArrowMarker(
  const std_msgs::msg::Header & header, const std::string & ns, int32_t id,
  const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & end,
  const std_msgs::msg::ColorRGBA & color, double shaft_diameter, double head_diameter,
  double head_length)
{
  Marker marker = makeBaseMarker(header, ns, id, Marker::ARROW);
  marker.points.push_back(start);
  marker.points.push_back(end);
  marker.scale.x = shaft_diameter;
  marker.scale.y = head_diameter;
  marker.scale.z = head_length;
  marker.color = color;
  return marker;
}

Marker makeTextMarker(
  const std_msgs::msg::Header & header, const std::string & ns, int32_t id,
  const geometry_msgs::msg::Point & position, const std::string & text,
  const std_msgs::msg::ColorRGBA & color, double size)
{
  Marker marker = makeBaseMarker(header, ns, id, Marker::TEXT_VIEW_FACING);
  marker.pose.position = position;
  marker.text = text;
  marker.scale.z = size;
  marker.color = color;
  return marker;
}

geometry_msgs::msg::Point transformLocalPoint(
  const tf2::Quaternion & orientation, const geometry_msgs::msg::Point & center,
  double x, double y, double z)
{
  const tf2::Vector3 local(x, y, z);
  const tf2::Vector3 world = tf2::quatRotate(orientation, local);
  return makePoint(center.x + world.x(), center.y + world.y(), center.z + world.z());
}

}  // namespace

class ObjectListMarkerArrayNode : public rclcpp::Node
{
public:
  ObjectListMarkerArrayNode()
  : Node("object_list_marker_array_node"), params_(*this)
  {
    marker_publisher_ = this->create_publisher<MarkerArray>(params_.output_topic, rclcpp::QoS(10));
    object_subscription_ = this->create_subscription<ufil_msgs::msg::ObjectList>(
      params_.input_topic, rclcpp::QoS(10),
      std::bind(&ObjectListMarkerArrayNode::onObjectList, this, std::placeholders::_1));

    purge_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ObjectListMarkerArrayNode::purgeExpiredObjects, this));
  }

private:
  void onObjectList(const ufil_msgs::msg::ObjectList::ConstSharedPtr msg)
  {
    MarkerArray array;
    const auto now = this->now();
    last_header_ = msg->header;

    for (const auto & object : msg->objects) {
      last_seen_[object.id] = now;

      const auto header = msg->header;
      const auto center = makePoint(
        object.state.state.x, object.state.state.y, 0.5 * object.dimension.dimension.height);
      const auto orientation = yawToQuaternion(object.state.state.yaw);
      tf2::Quaternion tf_orientation(
        orientation.x, orientation.y, orientation.z, orientation.w);

      // Body
      Marker body = makeBaseMarker(header, "body", static_cast<int32_t>(object.id), Marker::CUBE);
      body.pose.position = center;
      body.pose.orientation = orientation;
      body.scale.x = static_cast<float>(object.dimension.dimension.length);
      body.scale.y = static_cast<float>(object.dimension.dimension.width);
      body.scale.z = static_cast<float>(object.dimension.dimension.height);
      body.color = params_.body_color;
      array.markers.push_back(body);

      // Velocity arrows
      const geometry_msgs::msg::Point velocity_start = makePoint(
        center.x, center.y, center.z - 0.5 * object.dimension.dimension.height);
      const geometry_msgs::msg::Point velocity_end = makePoint(
        center.x + object.state.state.v_x,
        center.y + object.state.state.v_y,
        center.z - 0.5 * object.dimension.dimension.height);
      array.markers.push_back(makeArrowMarker(
        header, "velocity", static_cast<int32_t>(object.id), velocity_start, velocity_end,
        params_.velocity_color, params_.velocity_arrow_shaft_diameter,
        params_.velocity_arrow_head_diameter, params_.velocity_arrow_head_length));

      // Body axes
      const geometry_msgs::msg::Point axis_x_end = transformLocalPoint(
        tf_orientation, center, 0.5 * object.dimension.dimension.length + 1.0, 0.0, 0.0);
      const geometry_msgs::msg::Point axis_y_end = transformLocalPoint(
        tf_orientation, center, 0.0, 0.5 * object.dimension.dimension.width + 1.0, 0.0);
      const geometry_msgs::msg::Point axis_z_end = transformLocalPoint(
        tf_orientation, center, 0.0, 0.0, 0.5 * object.dimension.dimension.height + 1.0);
      array.markers.push_back(makeArrowMarker(
        header, "body_axis_x", static_cast<int32_t>(object.id), center, axis_x_end,
        params_.axis_x_color, params_.body_axis_arrow_shaft_diameter,
        params_.body_axis_arrow_head_diameter, params_.body_axis_arrow_head_length));
      array.markers.push_back(makeArrowMarker(
        header, "body_axis_y", static_cast<int32_t>(object.id) + 1, center, axis_y_end,
        params_.axis_y_color, params_.body_axis_arrow_shaft_diameter,
        params_.body_axis_arrow_head_diameter, params_.body_axis_arrow_head_length));
      array.markers.push_back(makeArrowMarker(
        header, "body_axis_z", static_cast<int32_t>(object.id) + 2, center, axis_z_end,
        params_.axis_z_color, params_.body_axis_arrow_shaft_diameter,
        params_.body_axis_arrow_head_diameter, params_.body_axis_arrow_head_length));

      // Covariance ellipses
      const double c00 = object.state.covariance[0 * 8 + 0];
      const double c01 = object.state.covariance[0 * 8 + 1];
      const double c10 = object.state.covariance[1 * 8 + 0];
      const double c11 = object.state.covariance[1 * 8 + 1];
      const Ellipse2D pos_ellipse = ellipseFromCovariance(c00, c01, c10, c11);

      Marker pos_cov = makeBaseMarker(
        header, "covariance_position", static_cast<int32_t>(object.id), Marker::LINE_STRIP);
      appendEllipsePoints(
        pos_cov, center.x, center.y, center.z, pos_ellipse, params_.covariance_position_color,
        params_.covariance_line_width, 0.0, params_.ellipse_segments);
      array.markers.push_back(pos_cov);

      const double yaw_variance = std::max(0.0, object.state.covariance[6 * 8 + 6]);
      Ellipse2D yaw_ellipse;
      yaw_ellipse.axis_a = std::max(0.1, 2.0 * std::sqrt(yaw_variance) * 0.8);
      yaw_ellipse.axis_b = yaw_ellipse.axis_a * 0.5;
      yaw_ellipse.angle = 0.0;
      Marker yaw_cov = makeBaseMarker(
        header, "covariance_orientation", static_cast<int32_t>(object.id), Marker::LINE_STRIP);
      appendEllipsePoints(
        yaw_cov, center.x, center.y, center.z + object.dimension.dimension.height + 0.1,
        yaw_ellipse, params_.covariance_orientation_color, params_.covariance_line_width, 0.0,
        params_.ellipse_segments);
      array.markers.push_back(yaw_cov);

      // Object text label
      std::ostringstream text;
      text << object.id << " | " << classificationToString(object.classification) << " | "
           << object.existence_probability;
      Marker label = makeTextMarker(
        header, "label", static_cast<int32_t>(object.id),
        makePoint(center.x, center.y, center.z + object.dimension.dimension.height + 0.3),
        text.str(), params_.text_color, params_.text_size);
      array.markers.push_back(label);
    }

    marker_publisher_->publish(array);
  }

  void purgeExpiredObjects()
  {
    if (!last_header_) {
      return;
    }

    const auto now = this->now();
    MarkerArray deletions;
    std::vector<uint32_t> expired_ids;

    for (const auto & [id, stamp] : last_seen_) {
      const double age = (now - stamp).seconds();
      if (age > params_.object_timeout) {
        expired_ids.push_back(id);
      }
    }

    for (const auto id : expired_ids) {
      deletions.markers.push_back(makeDeleteMarker(*last_header_, "body", static_cast<int32_t>(id),
        Marker::CUBE));
      deletions.markers.push_back(makeDeleteMarker(*last_header_, "velocity",
        static_cast<int32_t>(id), Marker::ARROW));
      deletions.markers.push_back(makeDeleteMarker(*last_header_, "body_axis_x",
        static_cast<int32_t>(id), Marker::ARROW));
      deletions.markers.push_back(makeDeleteMarker(*last_header_, "body_axis_y",
        static_cast<int32_t>(id) + 1, Marker::ARROW));
      deletions.markers.push_back(makeDeleteMarker(*last_header_, "body_axis_z",
        static_cast<int32_t>(id) + 2, Marker::ARROW));
      deletions.markers.push_back(makeDeleteMarker(*last_header_, "covariance_position",
        static_cast<int32_t>(id), Marker::LINE_STRIP));
      deletions.markers.push_back(makeDeleteMarker(*last_header_, "covariance_orientation",
        static_cast<int32_t>(id), Marker::LINE_STRIP));
      deletions.markers.push_back(makeDeleteMarker(*last_header_, "label", static_cast<int32_t>(id),
        Marker::TEXT_VIEW_FACING));
      last_seen_.erase(id);
    }

    if (!deletions.markers.empty()) {
      marker_publisher_->publish(deletions);
    }
  }

  ObjectListMarkerArrayParameters params_;
  rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr object_subscription_;
  rclcpp::TimerBase::SharedPtr purge_timer_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_publisher_;
  std::optional<std_msgs::msg::Header> last_header_;
  std::unordered_map<uint32_t, rclcpp::Time> last_seen_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectListMarkerArrayNode>());
  rclcpp::shutdown();
  return 0;
}
