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
 * @file ssl_node.hpp
 * @author Marius Molz (marius.molz@rwth-aachen.de)
 * @brief Virtual SSL Node
 * @version 1.0
 * @date 2024-01-12
 *
 */
#pragma once

#include <tf2_ros/static_transform_broadcaster.h>

#include <limits>
#include <map>
#include <memory>
#include <utility>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/node.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include <configuration.hpp>
#include "geom.hpp"

namespace ros_ssl
{
    /// @brief SSL Node, virtual instance
class SSLNode : public rclcpp::Node {
private:
        /// @brief ROS2 Publisher publishing pressure matrix data
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_{nullptr};
  rclcpp::TimerBase::SharedPtr publisher_timer_{nullptr};
  rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr subscription_{nullptr};

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_{nullptr};
  std::shared_ptr<rclcpp::JumpHandler> jump_handler_{nullptr};

  std::map<int64_t, rclcpp::Time> last_seen_times_;

  void resetClock(const rcl_time_jump_t & jump);

  std::chrono::time_point<std::chrono::system_clock> start;

  uint16_t last_points_ = std::numeric_limits<uint16_t>::max();

        /// @brief Specific driver instance used
  std::shared_ptr<cppdriver::IDriver> ssl_instance_{nullptr};

        /// @brief Internal pressure matrix to be published
  nav_msgs::msg::OccupancyGrid grid_{};

  void object_list_callback(ufil_msgs::msg::ObjectList::SharedPtr msg);

  void publisher_timer_callback();

public:
  SSLNode();
};
}  // namespace ros_ssl
