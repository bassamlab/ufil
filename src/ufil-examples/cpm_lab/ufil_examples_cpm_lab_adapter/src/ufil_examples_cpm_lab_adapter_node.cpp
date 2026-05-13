// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen
// University
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

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>

#include <cpm_lab_lab_msgs/msg/vehicle_state.hpp>
#include <ufil_msgs/msg/object.hpp>
#include <ufil_msgs/msg/object_stamped.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include <ufil_object_tracking/types/dimension.hpp>
#include <ufil_object_tracking/types/state.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include <GeographicLib/UTMUPS.hpp>

//------------------------------------------------------------------------------
// Utility: Convert from ROS quaternion to yaw angle
//------------------------------------------------------------------------------
inline double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_quat;
  tf2::fromMsg(q, tf_quat);
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Matrix3x3(tf_quat).getEulerYPR(yaw, pitch, roll);
  return yaw;
}

//------------------------------------------------------------------------------
// Main Node
//------------------------------------------------------------------------------
class CpmLabAdapterNode : public rclcpp::Node
{
public:
  CpmLabAdapterNode()
  : Node("ufil_example_cpm_lab_adapter")
  {
    RCLCPP_INFO(this->get_logger(), "Starting ufil_example_cpm_lab_adapter...");

    // Declare parameters
    this->declare_parameter("id", 0);
    this->declare_parameter("target_frame", "map");
    id_ = std::to_string(this->get_parameter("id").as_int());

    // QoS: Best effort, depth 10
    auto qos = rclcpp::QoS(10).reliability(
        rmw_qos_reliability_policy_from_str("best_effort"));

    // Subscriptions
    state_sub_ = this->create_subscription<cpm_lab_lab_msgs::msg::VehicleState>(
        "vehicle_state", qos,
        std::bind(&CpmLabAdapterNode::onVehicleState, this, std::placeholders::_1));

    // Publishers
    object_list_pub_ = this->create_publisher<ufil_msgs::msg::ObjectList>("object_list", 10);

    // TF listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "ufil_example_cpm_lab_adapter started.");
  }

private:
  //------------------------------------------------------------------------------
  // Callback for incoming VehicleState messages
  //------------------------------------------------------------------------------
  void onVehicleState(const cpm_lab_lab_msgs::msg::VehicleState & msg)
  {
    std::string target_frame = this->get_parameter("target_frame").as_string();

    // Build the object message
    ufil_msgs::msg::Object object = buildObjectFromState(msg);

    // Create buffer
    ufil_msgs::msg::ObjectList ufil_msg;
    ufil_msg.header = msg.header;
    ufil_msg.header.frame_id = "lab";
    ufil_msg.objects.push_back(object);

    ufil_msgs::msg::ObjectList transformed_ufil_msg;
    try {
      transformed_ufil_msg = this->tf_buffer_->transform(ufil_msg, target_frame);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    // Publish ObjectList
    object_list_pub_->publish(transformed_ufil_msg);
  }

  //------------------------------------------------------------------------------
  // Build UFIL object from vehicle state
  //------------------------------------------------------------------------------
  ufil_msgs::msg::Object buildObjectFromState(
    const cpm_lab_lab_msgs::msg::VehicleState & msg)
  {
    ufil_msgs::msg::Object object;
    object.id = std::stoi(id_);
    object.existence_probability = 0.99;
    object.classification.classification[ufil_msgs::msg::Classification::CAR] = 1.0;
    ufil_msgs::msg::Features features;
    features.fl = true;
    features.fr = true;
    features.bl = true;
    features.br = true;
    features.f = true;
    features.b = true;
    features.l = true;
    features.r = true;
    features.c = true;
    object.features = features;

    // Position
    Eigen::Vector3d pos{msg.pose.position.x, msg.pose.position.y, 0};
    pos = (pos * SCALING_FACTOR);

    // Orientation
    double yaw = quaternionToYaw(msg.pose.orientation);

    // Velocity
    Eigen::Rotation2D<double> rotation(yaw);
    Eigen::Vector2d vel{msg.twist.linear.x, msg.twist.linear.y};
    vel = rotation * (vel * SCALING_FACTOR);

    // State
    ufil::type::state::PoseVelocity2D state;
    state.x() = pos.x();
    state.y() = pos.y();
    state.vx() = vel.x();
    state.vy() = vel.y();
    state.yaw() = yaw;

    state.covariance().setZero();
    state.covariance()(0, 0) = 0.9;
    state.covariance()(1, 1) = 0.9;
    state.covariance()(2, 2) = 1.4;
    state.covariance()(3, 3) = 1.4;
    state.covariance()(4, 4) = 0.05;
    state.covariance()(5, 5) = -1;  // yaw rate not estimated

    object.state = ufil_ros::stateToMsg<ufil_msgs::msg::StateWithCovariance>(state);

    // Dimension
    // 22cm legnth, 10.5cm width, 6cm height scaled by SCALING_FACTOR
    ufil::type::dimension::Dimension3D dim;
    dim.length() = 0.22 * SCALING_FACTOR;
    dim.width() = 0.105 * SCALING_FACTOR;
    dim.height() = 0.06 * SCALING_FACTOR;
    // 1mm measurement error scaled by SCALING_FACTOR
    dim.covariance().diagonal() << 0.081 * SCALING_FACTOR, 0.001 * SCALING_FACTOR,
      0.001 * SCALING_FACTOR;

    object.dimension =
      ufil_ros::dimensionToMsg<ufil_msgs::msg::DimensionWithCovariance>(dim);

    return object;
  }

  //------------------------------------------------------------------------------
  // Constants & Members
  //------------------------------------------------------------------------------
  static constexpr float SCALING_FACTOR = 18.0;

  std::string id_;

  rclcpp::Subscription<cpm_lab_lab_msgs::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr object_list_pub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

//------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CpmLabAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
