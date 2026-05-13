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


/**
 * @file ssl_node.cpp
 * @author Marius Molz (marius.molz@rwth-aachen.de)
 * @brief Virtual SSL Node
 * @version 1.0
 * @date 2024-01-12
 *
 */
#include "ssl_node.hpp"

#include <fmt/format.h>
#include <chrono>
#include <cstdint>
#include <sstream>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geom.hpp"
#include "simple_vehicle_model.hpp"
#include "ssl_single_virtual.hpp"
#include "vehicle_model_interface.hpp"

namespace ros_ssl
{
SSLNode::SSLNode()
: rclcpp::Node("ssl_publisher")
{
  using cppdriver::ACTIVE_POINTS_MODE;
  using cppdriver::ADC_RESOLUTION;
  using cppdriver::AUTO_ZERO;
  using cppdriver::LINE_FILTER;
  using cppdriver::NOISE_FILTER;
  using cppdriver::VOLTAGE_SHARED_COMPENSATION;
  using cppdriver::impl::SingleSSLVirtual;
  using cppdriver::impl::configuration::Configuration;

        // Declare parameters
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "Specify threshold of active points. Defaults to 10. [1, 4095]";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  declare_parameter("threshold", 10, param_desc);
  auto threshold = this->get_parameter("threshold").as_int();

  param_desc.description = "Node index. Defaults to 1.";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  declare_parameter("node_id", 1, param_desc);
  auto node_id = this->get_parameter("node_id").as_int();

  param_desc.description =
    "Number of seconds elapsed from the last received message after which an "
    "object will be declared stale, and cleaned up.";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  this->declare_parameter("stale_seconds", 0.5, param_desc);

  param_desc.description = "Width of the SSL (direction of heading) in meters. Defaults to 10.";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  this->declare_parameter("width", 10.0, param_desc);
  auto width = this->get_parameter("width").as_double();

  param_desc.description = "Height of the SSL in meters. Defaults to 5.";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  this->declare_parameter("height", 5.0, param_desc);
  auto height = this->get_parameter("height").as_double();

  param_desc.description = "Resolution of the SSL in meters/cell. Defaults to 0.2.";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  this->declare_parameter("resolution", 0.2, param_desc);
  auto resolution = static_cast<float>(this->get_parameter("resolution").as_double());

  param_desc.description = "Mat placement [x]. Defaults to 0.";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  this->declare_parameter("x", 0.0, param_desc);
  auto x = static_cast<float>(this->get_parameter("x").as_double());

  param_desc.description = "Mat placement [y]. Defaults to 0.";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  this->declare_parameter("y", 0.0, param_desc);
  auto y = static_cast<float>(this->get_parameter("y").as_double());

  param_desc.description = "Mat placement [z]. Defaults to 0.011.";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  this->declare_parameter("z", 0.011f, param_desc);
  auto z = static_cast<float>(this->get_parameter("z").as_double());

  param_desc.description = "Mat placement [heading]. Defaults to 0.";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  this->declare_parameter("h", 0.0, param_desc);
  auto heading = static_cast<float>(this->get_parameter("h").as_double());

  param_desc.description =
    "Scale of the environment. Only for internal values, does not have an "
    "influence on the other parameters.";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  this->declare_parameter("microscale", false, param_desc);
  auto microscale = this->get_parameter("microscale").as_bool();

  auto cells_width = static_cast<int>(std::ceil(width / resolution));
  auto cells_height = static_cast<int>(std::ceil(height / resolution));

  Configuration configuration{
    x,
    y,
    z,
    heading,
    resolution,
    cells_width,
    cells_height,
    microscale
  };

  RCLCPP_INFO(
    this->get_logger(),
    "Configuration Node %li: X: %f, Y: %f, Z: %f, Heading: %f, Resolution: %f, "
    "Cells: %i x %i, Microscale: %d, Safety Margin: %f",
    node_id, x, y, z, heading, resolution, cells_width, cells_height, microscale,
    configuration.getSimulationSafetyMargin());

        // Simulate virtual ssl
  ssl_instance_ = std::make_unique<SingleSSLVirtual>(node_id, configuration);
  auto const instance_ptr = std::static_pointer_cast<SingleSSLVirtual>(ssl_instance_);

  this->subscription_ = this->create_subscription<ufil_msgs::msg::ObjectList>(
                "object_list",
                rclcpp::SensorDataQoS(),
                std::bind(&SSLNode::object_list_callback, this, std::placeholders::_1));

  instance_ptr->registerClock(get_clock());

  auto own_frame_id = "ssl_frame_" + std::to_string(node_id);

        // Prepare grid message
  grid_.header.frame_id = "map";
  grid_.info.width = cells_width;
  grid_.info.height = cells_height;
  grid_.info.origin.position.x = x;
  grid_.info.origin.position.y = y;
  grid_.info.origin.position.z = z;
  grid_.info.resolution = resolution;

  tf2::Quaternion quaternionGrid;
  quaternionGrid.setRPY(0.f, 0.f, heading);
  grid_.info.origin.orientation = tf2::toMsg(quaternionGrid);

  grid_.data.resize(cells_width * cells_height);

  ssl_instance_->commandActivePointMode(ACTIVE_POINTS_MODE::ONLY_ACTIVE_POINTS);
  ssl_instance_->commandADCResolution(ADC_RESOLUTION::LOW);
  ssl_instance_->commandActivePointThreshold(threshold);
  ssl_instance_->commandAutoZero(AUTO_ZERO::DISABLED);
  ssl_instance_->commandLineFilter(LINE_FILTER::ENABLED);
  ssl_instance_->commandNoiseFilter(NOISE_FILTER(2));
  ssl_instance_->commandVoltageSharedCompensation(VOLTAGE_SHARED_COMPENSATION::ENABLED);

  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Prepare positioning
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id = own_frame_id;

  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;

  tf2::Quaternion quaternionFrame;

        // Rotation of the grid frame
  quaternionFrame.setRPY(0.f, 0.f, heading);
  t.transform.rotation.x = quaternionFrame.x();
  t.transform.rotation.y = quaternionFrame.y();
  t.transform.rotation.z = quaternionFrame.z();
  t.transform.rotation.w = quaternionFrame.w();

  tf_static_broadcaster_->sendTransform(t);

  RCLCPP_INFO(this->get_logger(),
                    "Initialized %s node %li at X: %f, Y: %f, H: %f",
                    "virtual",
                    node_id,
                    x,
                    y,
                    heading);

  publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("ssl_" +
      std::to_string(node_id) + "/map",
                                                                          rclcpp::QoS(
      10).best_effort().durability_volatile());

  ssl_instance_->commandStart();
  // Mat @ 23ms
  publisher_timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(10), [&] {
      publisher_timer_callback();
    });

  // Reset clock if necessary
  if (get_clock()->ros_time_is_active()) {
    RCLCPP_INFO(this->get_logger(), "Using simulated time source.");
    jump_handler_ = get_clock()->create_jump_callback(
      [&] {ssl_instance_->commandStop();},
      [&](auto && rcl_time_jump) {
        resetClock(std::forward<decltype(rcl_time_jump)>(rcl_time_jump));
                    },
                    rcl_jump_threshold_t{
        false,
        rcl_duration_t{0},
        rcl_duration_t{-10},
                    });
  }

  start = std::chrono::high_resolution_clock::now();
}

void
SSLNode::resetClock(rcl_time_jump_t const & jump)
{
  if (jump.delta.nanoseconds <= 0U) {
    auto const instance_ptr =
      std::static_pointer_cast<cppdriver::impl::SingleSSLVirtual>(ssl_instance_);
    std::lock_guard<std::mutex> lock(instance_ptr->clock_used);
    instance_ptr->registerClock(get_clock());
    RCLCPP_WARN(this->get_logger(), "Detected backward time jump. Resetting threads.");
  }
  ssl_instance_->commandStart();
}

void
SSLNode::object_list_callback(const ufil_msgs::msg::ObjectList::SharedPtr msg)
{
  using cppdriver::impl::SingleSSLVirtual;
  auto instance_ptr = std::static_pointer_cast<SingleSSLVirtual>(ssl_instance_);

  rclcpp::Time now = this->now();
  rclcpp::Time tp = msg->header.stamp;

  std::lock_guard<std::mutex> lock(instance_ptr->map_mutex);
  instance_ptr->object_map_.clear();

  for (const auto & object : msg->objects) {
    Axles axles{};
    for (const auto & axle : object.axles) {
      axles.emplace_back(Axle{
          .single_track = axle.single_track,
          .track_width = axle.track_width,
          .center_to_axle = axle.center_to_axle
                });
    }


    cppdriver::vehicle::Measurement m{};
    m.x() = static_cast<float>(object.state.state.x);
    m.y() = static_cast<float>(object.state.state.y);
    m.speed() = static_cast<float>(object.state.state.v_x);
    m.yaw() = static_cast<float>(object.state.state.yaw);
    m.yawRate() = static_cast<float>(object.state.state.yaw_rate);

    cppdriver::vehicle::MeasurementModel mm{};
    auto covariance = Eigen::MatrixX<cppdriver::vehicle::T>(5, 5);
    covariance.setZero();         // TODO(simon.schaefer): this is not supposed to be zero
    mm.setCovariance(covariance);
    mm.setMeasurement(m);
    mm.validate();

    auto model = std::make_shared<cppdriver::vehicle::SimpleVehicleModel>(axles);
    model->update(mm, tp);

    instance_ptr->object_map_[object.id] = model;
    this->last_seen_times_[object.id] = now;
  }
}

void SSLNode::publisher_timer_callback()
{
  using cppdriver::impl::SingleSSLVirtual;
  auto instance_ptr = std::static_pointer_cast<SingleSSLVirtual>(ssl_instance_);
  auto now = this->get_clock()->now();

  std::unique_lock<std::mutex> lock(instance_ptr->map_mutex);

  // Removes vehicles not seen for more than stale_seconds
  auto stale_seconds = this->get_parameter("stale_seconds").as_double();
  for (auto it = instance_ptr->object_map_.begin(); it != instance_ptr->object_map_.end(); ) {
    int64_t id = it->first;

    if (last_seen_times_.find(id) == last_seen_times_.end() ||
      (now - last_seen_times_[id]).seconds() > stale_seconds)
    {
      last_seen_times_.erase(id);
      it = instance_ptr->object_map_.erase(it);
    } else {
      ++it;
    }
  }

  grid_.header.stamp = ssl_instance_->lastUpdate();
  auto const points_n = ssl_instance_->mapPtr().size();

  if (points_n > 0 || this->last_points_ > 0) {
    this->last_points_ = points_n;
    std::fill(grid_.data.begin(), grid_.data.end(), 0U);

    for (auto const &[coordinate, pressure] : ssl_instance_->mapPtr()) {
      auto const x = (+std::get<0>(coordinate));
      auto const y = (+std::get<1>(coordinate));
      grid_.data[y * grid_.info.width + x] = static_cast<int8_t>(pressure);
    }

    grid_.info.map_load_time = grid_.header.stamp;

    publisher_->publish(grid_);
  }
}
}  // namespace ros_ssl

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros_ssl::SSLNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
