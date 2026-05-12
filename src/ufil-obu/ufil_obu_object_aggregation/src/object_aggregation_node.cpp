// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWHT Aachen University
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

#include <tf2/utils.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <initializer_list>
#include <algorithm>
#include <stdexcept>
#include <type_traits>

#include <GeographicLib/UTMUPS.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ufil_msgs/msg/object_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/functional/hash.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

template<typename HashType>
HashType generateHashedId()
{
  static_assert(std::is_integral_v<HashType>, "HashType must be an integral type");

  auto uuid = boost::uuids::random_generator()();
  boost::hash<boost::uuids::uuid> uuid_hasher;
  auto hash_value = uuid_hasher(uuid);

  return static_cast<HashType>(hash_value);
}

class ObjectAggregationNode : public rclcpp::Node
{
private:
  using ApproximateSyncPolicy =
    message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::AccelWithCovarianceStamped,
      nav_msgs::msg::Odometry, sensor_msgs::msg::NavSatFix>;

  message_filters::Subscriber<geometry_msgs::msg::AccelWithCovarianceStamped> accel_subscriber_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> nav_sat_fix_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;

  rclcpp::Publisher<ufil_msgs::msg::ObjectStamped>::SharedPtr object_publisher_;

  ufil_msgs::msg::Dimension::_length_type object_length_{};
  ufil_msgs::msg::Dimension::_width_type object_width_{};
  ufil_msgs::msg::Object::_id_type id_{};
  ufil_msgs::msg::Classification::_classification_type::size_type object_classification_{};

  static builtin_interfaces::msg::Time
  getBiggestStamp(const std::initializer_list<builtin_interfaces::msg::Time> & stamps)
  {
    return *std::max_element(stamps.begin(), stamps.end(), [](const auto & a, const auto & b) {
               return (a.sec == b.sec) ? (a.nanosec < b.nanosec) : (a.sec < b.sec);
    });
  }

  void syncCallback(
    const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr & accel_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr & nav_msg)
  {
    ufil_msgs::msg::ObjectStamped object;
    object.header.stamp = getBiggestStamp({accel_msg->header.stamp, odom_msg->header.stamp,
          nav_msg->header.stamp});
    object.header.frame_id = "utm_32n";
    object.object.id = this->id_;

    // Transform latitude/longitude to UTM
    int zone;
    bool north_hp;
    GeographicLib::Math::real x, y;
    GeographicLib::UTMUPS::Forward(nav_msg->latitude, nav_msg->longitude, zone, north_hp, x, y);

    object.object.state.state.x = x;
    object.object.state.state.y = y;
    object.object.state.state.v_x = odom_msg->twist.twist.linear.x;
    object.object.state.state.v_y = odom_msg->twist.twist.linear.y;
    object.object.state.state.a_x = accel_msg->accel.accel.linear.x;
    object.object.state.state.a_y = accel_msg->accel.accel.linear.y;
    object.object.state.state.yaw = tf2::getYaw(odom_msg->pose.pose.orientation);
    object.object.state.state.yaw_rate = odom_msg->twist.twist.angular.z;

    fillCovariance(odom_msg, accel_msg, object);

    object.object.dimension.dimension.length = this->object_length_;
    object.object.dimension.dimension.width = this->object_width_;
    object.object.dimension.covariance[0] = 1e-9;
    object.object.dimension.covariance[3] = 1e-9;

    object.object.existence_probability = 1;
    object.object.classification.classification[this->object_classification_] = 1.0;

    fillFeatures(object);

    this->object_publisher_->publish(object);
  }

  void fillCovariance(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
    const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr & accel_msg,
    ufil_msgs::msg::ObjectStamped & object)
  {
    const auto & pose_cov = odom_msg->pose.covariance;
    const auto & twist_cov = odom_msg->twist.covariance;
    const auto & accel_cov = accel_msg->accel.covariance;
    auto & covariance = object.object.state.covariance;

    // Assign covariance values
    covariance[0] = pose_cov[0];     // Cov(x, x)
    covariance[1] = pose_cov[1];     // Cov(x, y)
    covariance[6] = pose_cov[5];     // Cov(x, yaw)
    covariance[8] = pose_cov[6];     // Cov(y, x)
    covariance[9] = pose_cov[7];     // Cov(y, y)
    covariance[14] = pose_cov[11];   // Cov(y, yaw)
    covariance[18] = twist_cov[0];   // Cov(v_x, v_x)
    covariance[19] = twist_cov[1];   // Cov(v_x, v_y)
    covariance[23] = twist_cov[5];   // Cov(v_x, yaw_rate)
    covariance[26] = twist_cov[6];   // Cov(v_y, v_x)
    covariance[27] = twist_cov[7];   // Cov(v_y, v_y)
    covariance[31] = twist_cov[11];  // Cov(v_y, yaw_rate)
    covariance[36] = accel_cov[0];   // Cov(a_x, a_x)
    covariance[37] = accel_cov[1];   // Cov(a_x, a_y)
    covariance[44] = accel_cov[6];   // Cov(a_y, a_x)
    covariance[45] = accel_cov[7];   // Cov(a_y, a_y)
    covariance[48] = pose_cov[30];   // Cov(yaw, x)
    covariance[49] = pose_cov[31];   // Cov(yaw, y)
    covariance[54] = pose_cov[35];   // Cov(yaw, yaw)
    covariance[58] = twist_cov[30];  // Cov(yaw_rate, v_x)
    covariance[59] = twist_cov[31];  // Cov(yaw_rate, v_y)
    covariance[63] = twist_cov[35];  // Cov(yaw_rate, yaw_rate)
  }

  void fillFeatures(ufil_msgs::msg::ObjectStamped & object)
  {
    object.object.features.fl = true;
    object.object.features.fr = true;
    object.object.features.bl = true;
    object.object.features.br = true;
    object.object.features.f = true;
    object.object.features.b = true;
    object.object.features.l = true;
    object.object.features.r = true;
    object.object.features.c = true;
  }

public:
  ObjectAggregationNode()
  : Node("object_fusion_node")
  {
    declareParameters();

    // Generate ID
    this->id_ = generateHashedId<ufil_msgs::msg::Object::_id_type>();

    // Subscribe to messages
    this->accel_subscriber_.subscribe(this, "accel");
    this->odometry_subscriber_.subscribe(this, "odometry");
    this->nav_sat_fix_subscriber_.subscribe(this, "nav_sat_fix");

    // Define synchronizer
    uint32_t queue_size = 10;
    this->sync_ = std::make_unique<message_filters::Synchronizer<ApproximateSyncPolicy>>(
        ApproximateSyncPolicy(queue_size), this->accel_subscriber_, this->odometry_subscriber_,
        this->nav_sat_fix_subscriber_);
    this->sync_->setAgePenalty(1);  // 100ms
    this->sync_->registerCallback(std::bind(&ObjectAggregationNode::syncCallback, this, _1, _2,
      _3));

    // Publisher
    this->object_publisher_ =
      this->create_publisher<ufil_msgs::msg::ObjectStamped>("object", rclcpp::SystemDefaultsQoS());
  }

  void declareParameters()
  {
    this->object_length_ = this->declare_parameter("object_length", 0.0);
    if (this->object_length_ <= 0) {
      throw std::runtime_error("Invalid object length!");
    }

    this->object_width_ = this->declare_parameter("object_width", 0.0);
    if (this->object_width_ <= 0) {
      throw std::runtime_error("Invalid object width!");
    }

    this->object_classification_ =
      this->declare_parameter("object_classification", ufil_msgs::msg::Classification::OTHER);
    if (!(ufil_msgs::msg::Classification::CAR <= this->object_classification_ &&
      this->object_classification_ <= ufil_msgs::msg::Classification::OTHER))
    {
      throw std::runtime_error("Invalid classification!");
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectAggregationNode>();

  RCLCPP_INFO(node->get_logger(), "Started object aggregation node.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Stopped object aggregation node.");

  return 0;
}
