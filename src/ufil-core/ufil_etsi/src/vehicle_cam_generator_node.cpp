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


#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <random>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ufil_ros/ufil_ros.hpp>

#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <ufil_etsi/CamBuilder.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_msgs/msg/object_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ufil_etsi/Utilities.hpp"

using namespace std::chrono_literals;
using etsi_its_cam_msgs::msg::DeltaAltitude;
using etsi_its_cam_msgs::msg::DeltaLatitude;
using etsi_its_cam_msgs::msg::DeltaLongitude;
using etsi_its_cam_msgs::msg::PathDeltaTime;
using etsi_its_cam_msgs::msg::PathPoint;
using etsi_its_cam_msgs::msg::ReferencePosition;

class VehicleCamGeneratorNode : public rclcpp::Node
{
private:
  // The following values are taken directly from ETSI EN 302 637-2 Clause 6.1.3

  // The CAM generation interval shall not be inferior to T_GenCamMin
  static constexpr auto t_gen_cam_min_ = 100ms;

  // The CAM generation interval shall not be superior to T_GenCamMax = 1 000 ms
  static constexpr auto t_gen_cam_max_ = 1000ms;

  // The conditions for triggering the CAM generation
  // shall be checked repeatedly every T_CheckCamGen.
  // T_CheckCamGen shall be equal to or less than T_GenCamMin
  // We check every T_GenCamMin / 2 = 50ms.
  static constexpr auto t_check_gen_cam_ = VehicleCamGeneratorNode::t_gen_cam_min_ / 2;

  // The value of the parameter N_GenCam can be dynamically
  // adjusted according to some environmental conditions.
  // The default and maximum value of N_GenCam shall be 3.
  static constexpr auto n_gen_cam_ = 3;

  // The parameter T_GenCam represents the currently valid upper limit
  // of the CAM generation interval.
  // The default value of T_GenCam shall be T_GenCamMax.
  rclcpp::Duration t_gen_cam_ = rclcpp::Duration(VehicleCamGeneratorNode::t_gen_cam_max_);

  // Count the number of triggered CAMs to set T_GenCam accordingly.
  int triggered_cams_2_ = 0;

  // Time of the last sent CAM
  rclcpp::Time time_of_last_sent_cam_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Time of the last sent low frequency container.
  rclcpp::Time time_of_last_sent_low_frequency_container_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Path History
  etsi_its_cam_msgs::msg::PathHistory path_history_;
  etsi_its_cam_msgs::msg::ReferencePosition::SharedPtr previous_reference_position;

  // CAM Builder
  std::shared_ptr<etsi_message_converter::CamBuilder> cam_builder_;

  // Subscribers
  rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr object_subscriber_;

  // Publishers
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr cam_publisher_;

  // TF Buffer
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Timers
  // Implements the T_CheckCamGen timer as specified in ETSI EN 302 637-2.
  rclcpp::TimerBase::SharedPtr check_cam_gen_timer_;

  // Last received messages
  std::string target_frame_ = "utm_32n";
  std::unique_ptr<ufil_msgs::msg::ObjectStamped> last_object_;

  // Important values of last sent CAM
  // Required for checking the conditions to send a new CAM
  double yaw_value_of_last_cam_ = 0;
  geometry_msgs::msg::Point position_of_last_cam_ = geometry_msgs::msg::Point();
  double speed_of_last_cam_ = 0;

  /*
   * SAE J2735 says about the generation of path points:
   * The initial anchor point (our reference position) is used to create the offset values of the set.
   * All Path History Points are older in time than the anchor point used.
   * Each Path History Point is subtracted from the initial anchor point to create the offset values.
   * The first point set in the message is the closest in time to the anchor point; older points follow in the order in
   * which they were determined. Note that this methodology produces offsets where positive is in the South, West and
   * Down directions. The sign of these offsets is inverted from conventions used elsewhere in this standard.
   */
  void updatePathHistory(
    ReferencePosition reference_position,
    const rclcpp::Time & measurement_time)
  {
    if (!this->previous_reference_position) {
      // We don't have a previous position to build a path
      // Thus, we just save the current position and end
      this->previous_reference_position = std::make_shared<ReferencePosition>(reference_position);
      return;
    }

    // Create path point based on last
    PathPoint path_point;
    path_point.path_position.delta_latitude.value =
      reference_position.latitude.value - this->previous_reference_position->latitude.value;
    path_point.path_position.delta_longitude.value =
      reference_position.longitude.value - this->previous_reference_position->longitude.value;
    path_point.path_position.delta_altitude.value =
      static_cast<DeltaAltitude::_value_type>(reference_position.altitude.altitude_value.value -
      this->previous_reference_position->altitude.altitude_value.value);

    path_point.path_delta_time.value = static_cast<PathDeltaTime::_value_type>(
      (measurement_time - this->time_of_last_sent_cam_).seconds() * 100);    // In 10ms steps
    path_point.path_delta_time_is_present = true;

    // Insert point into history
    this->path_history_.array.insert(this->path_history_.array.begin(), path_point);

    // Maximum length of path history is 23
    if (this->path_history_.array.size() == 24) {
      this->path_history_.array.pop_back();
    }

    // Update all following points to account for the new distance
    for (auto it = this->path_history_.array.begin() + 1; it != this->path_history_.array.end();
      ++it)
    {
      auto next_point = *it;
      next_point.path_position.delta_latitude.value +=
        path_point.path_position.delta_latitude.value;
      next_point.path_position.delta_longitude.value +=
        path_point.path_position.delta_longitude.value;
      next_point.path_position.delta_altitude.value +=
        path_point.path_position.delta_altitude.value;

      assert(DeltaLatitude::MIN <= next_point.path_position.delta_latitude.value &&
             next_point.path_position.delta_latitude.value <= DeltaLatitude::MAX - 1);
      assert(DeltaLongitude::MIN <= next_point.path_position.delta_longitude.value &&
             next_point.path_position.delta_longitude.value <= DeltaLongitude::MAX - 1);
      assert(DeltaAltitude::MIN <= next_point.path_position.delta_altitude.value &&
             next_point.path_position.delta_altitude.value <= DeltaAltitude::MAX - 1);
    }
  }

  inline void onObject(const ufil_msgs::msg::ObjectList & msg)
  {
    int64_t object_array_index = this->get_parameter("object_array_index").as_int();

    if(msg.objects.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Received empty object list");
      this->last_object_.reset();
    }

    if(object_array_index >= static_cast<int64_t>(msg.objects.size())) {
      RCLCPP_WARN(this->get_logger(), "Received object list with to little elements.");
      this->last_object_.reset();
      return;
    }

    ufil_msgs::msg::ObjectList transformed_msg;
    try {
      transformed_msg = this->tf_buffer_->transform(msg, this->target_frame_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
      this->last_object_.reset();
      return;
    }


    ufil_msgs::msg::ObjectStamped stored_msg;
    stored_msg.header = transformed_msg.header;
    stored_msg.object = transformed_msg.objects.at(object_array_index);

    this->last_object_ = std::make_unique<ufil_msgs::msg::ObjectStamped>(stored_msg);
  }

  void publishCAM()
  {
    etsi_its_cam_msgs::msg::CAM::UniquePtr cam_msg;

    auto measurement_time = this->last_object_->header.stamp;
    measurement_time = measurement_time +
      rclcpp::Duration(std::chrono::nanoseconds(this->get_parameter(
      "time_shift_nanosec").as_int()));

    // Check if low frequency container should be sent
    if (this->now() - this->time_of_last_sent_low_frequency_container_ > rclcpp::Duration(500ms)) {
      // Send low frequency container
      cam_msg = this->cam_builder_->generationDeltaTime(measurement_time)
        .basicContainer(this->last_object_)
        .highFrequencyContainer(this->last_object_)
        .lowFrequencyContainer(this->path_history_.array)
        .get();
      time_of_last_sent_low_frequency_container_ = this->now();
    } else {
      // Don't send low frequency container
      cam_msg = this->cam_builder_->generationDeltaTime(measurement_time)
        .basicContainer(this->last_object_)
        .highFrequencyContainer(this->last_object_)
        .get();
    }

    // Update path history
    this->updatePathHistory(cam_msg->cam.cam_parameters.basic_container.reference_position,
      measurement_time);

    // TODO(dobby): Very hacky, but current structure of this lib doesn't facilitate it otherwise
    cam_msg->cam.cam_parameters.low_frequency_container.basic_vehicle_container_low_frequency.
    path_history = this->path_history_;
    // Save important values of sent CAM
    this->yaw_value_of_last_cam_ = this->last_object_->object.state.state.yaw;
    const float speed = std::sqrt(std::pow(this->last_object_->object.state.state.v_x,
      2.0f) + std::pow(this->last_object_->object.state.state.v_y, 2.0f));
    this->speed_of_last_cam_ = speed;
    this->position_of_last_cam_.x = this->last_object_->object.state.state.x;
    this->position_of_last_cam_.y = this->last_object_->object.state.state.y;
    // Publish message
    cam_msg->header.station_id.value = this->last_object_->object.id;
    this->cam_publisher_->publish(*cam_msg);
  }

  bool checkCamGenCondition1() const
  {
    // Condition 1: Since we ignore whether we use ITS-G5, T_GenCam_Dcc doesn't apply.
    // Thus, we directly check the given ITS-S dynamics related conditions.
    // We do these checks in our local reference system,
    // since it is easier (concerning the reference position) and
    // requires fewer calculations

    // The absolute difference between the current heading of the originating ITS-S
    // and the heading included in the CAM previously transmitted
    // by the originating ITS-S exceeds 4°.
    auto yaw_value_of_last_cam_deg = this->yaw_value_of_last_cam_ * 180.0 / std::numbers::pi;
    auto current_yaw_deg = this->last_object_->object.state.state.yaw * 180.0 / std::numbers::pi;

    auto gen_cam = (std::abs(yaw_value_of_last_cam_deg - current_yaw_deg) > 4);

    // The distance between the current position of the originating ITS-S
    // and the position included in the CAM previously transmitted
    // by the originating ITS-S exceeds 4 m.
    auto dx = this->position_of_last_cam_.x - this->last_object_->object.state.state.x;
    auto dy = this->position_of_last_cam_.y - this->last_object_->object.state.state.y;

    gen_cam = gen_cam || (std::hypot(dx, dy) > 4);

    // The absolute difference between the current speed of the originating ITS-S
    // and the speed included in the CAM previously transmitted
    // by the originating ITS-S exceeds 0,5 m/s.
    const float speed = std::sqrt(std::pow(this->last_object_->object.state.state.v_x,
      2.0f) + std::pow(this->last_object_->object.state.state.v_y, 2.0f));

    gen_cam = gen_cam ||
      (std::abs(this->speed_of_last_cam_ - speed) > 0.5);

    return gen_cam;
  }

  inline bool checkCamGenCondition2() const
  {
    // Condition 2: The time elapsed since the last CAM generation is equal to or greater than
    // T_GenCam
    rclcpp::Time now_time = this->now();
    now_time = now_time + rclcpp::Duration(std::chrono::nanoseconds(
                                              this->get_parameter("time_shift_nanosec").as_int()));
    return (now_time - this->time_of_last_sent_cam_) >= this->t_gen_cam_;
  }

  void onCheckCamGenTimer()
  {
    if (!this->last_object_) {
      return;  // We haven't received a message yet
    }

    // ETSI EN 302 637-2 specifies the conditions for CAM generation frequency
    // Special rules apply if we're operating in ITS-G5 - we don't consider those here
    // We also don't pay attention to the channel congestion status, just our dynamics

    rclcpp::Time measurement_time = this->last_object_->header.stamp;
    measurement_time = measurement_time + rclcpp::Duration(std::chrono::nanoseconds(
                                              this->get_parameter("time_shift_nanosec").as_int()));

    // Check CAM generation trigger conditions
    if (this->checkCamGenCondition1()) {
      // Update T_GenCam
      this->t_gen_cam_ = measurement_time - this->time_of_last_sent_cam_;


         // Generate and publish a new CAM
      this->publishCAM();

      // Update time of last sent CAM
      this->time_of_last_sent_cam_ = measurement_time;

      // Reset condition 2 counter
      this->triggered_cams_2_ = 0;
    } else if (this->checkCamGenCondition2()) {
      // Generate and publish a new CAM
      this->publishCAM();

      // Update time of last sent CAM
      this->time_of_last_sent_cam_ = measurement_time;


      // Manage condition 2 counter
      if (this->triggered_cams_2_ < VehicleCamGeneratorNode::n_gen_cam_ - 1) {
        // Just increase counter
        this->triggered_cams_2_++;
      } else if (this->triggered_cams_2_ == VehicleCamGeneratorNode::n_gen_cam_ - 1) {
        // We triggered the number of n_gen_cam_ consecutive
        // CAMs due to condition 2, T_GenCam shall be set to
        // T_GenCamMax.
        this->t_gen_cam_ = VehicleCamGeneratorNode::t_gen_cam_max_;
      }
      this->last_object_.reset();
    }
  }

public:
  VehicleCamGeneratorNode()
  : Node("vehicle_cam_generator_node")
  {
    std::random_device random_device;      // Obtain a random seed from the hardware
    std::mt19937 engine(random_device());  // Seed the generator
    std::uniform_int_distribution<etsi_its_cam_msgs::msg::StationID::_value_type> distribution(
      etsi_its_cam_msgs::msg::StationID::MIN,
      etsi_its_cam_msgs::msg::StationID::MAX);    // Define the distribution for allow range

    // Generate Station ID
    auto station_id = distribution(engine);
    RCLCPP_INFO(this->get_logger(), "Station ID: %u", station_id);

    // Vehicle Role Parameter
    auto vehicle_role_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    vehicle_role_param_desc.description =
      "Vehicle Role. See ETSI TS 102 894-2 A.94 DE_VehicleRole. Default for cars is 0.";
    vehicle_role_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("vehicle_role", etsi_its_cam_msgs::msg::VehicleRole::DEFAULT,
      vehicle_role_param_desc);
    auto vehicle_role =
      static_cast<VehicleRole::_value_type>(this->get_parameter("vehicle_role").as_int());
    assert(0 <= vehicle_role && vehicle_role <= 15);
    RCLCPP_INFO(this->get_logger(), "Vehicle Role: %hhu", vehicle_role);

    this->declare_parameter("target_frame", this->target_frame_);
    this->target_frame_ = this->get_parameter("target_frame").as_string();
    RCLCPP_INFO(this->get_logger(), "Transform input to target frane: %s",
      this->target_frame_.c_str());

    this->declare_parameter("time_shift_nanosec", 0);
    int64_t shift_ns = this->get_parameter("time_shift_nanosec").as_int();
    RCLCPP_INFO(this->get_logger(), "Time shift in nanoseconds: %lu", shift_ns);

    this->declare_parameter("object_array_index", 0);
    int64_t object_array_index = this->get_parameter("object_array_index").as_int();
    RCLCPP_INFO(this->get_logger(), "Select object array index: %lu", object_array_index);

    // Init CAM Builder
    this->cam_builder_ = std::make_shared<etsi_message_converter::CamBuilder>(station_id,
      vehicle_role);

    // Subscribers
    this->object_subscriber_ = this->create_subscription<ufil_msgs::msg::ObjectList>(
        "object", rclcpp::SystemDefaultsQoS(),
        std::bind(&VehicleCamGeneratorNode::onObject, this, std::placeholders::_1));

    this->cam_publisher_ = this->create_publisher<etsi_its_cam_msgs::msg::CAM>("cam",
      rclcpp::SystemDefaultsQoS());

    // Timers
    this->check_cam_gen_timer_ =
      this->create_wall_timer(VehicleCamGeneratorNode::t_check_gen_cam_, [this] {
          onCheckCamGenTimer();
      });

    // TF listener
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleCamGeneratorNode>();

  RCLCPP_INFO(node->get_logger(), "Started vehicle CAM generator.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Stopped vehicle CAM generator.");

  return 0;
}
