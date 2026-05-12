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


#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <ufil_msgs/msg/object_list.hpp>
#include <ufil_ros/ufil_ros.hpp>

#include <ufil_examples_pedestrian_tracker/ufil_examples_pedestrian_tracker.hpp>

/**
 * @class PedestrianTrackerNode
 * @brief ROS2 Node for tracking pedestrians using UFIL framework.
 */
class PedestrianTrackerNode : public rclcpp::Node
{
public:
  PedestrianTrackerNode()
  : Node("pedestrian_tracker_node")
  {
    // Subscribe to the topic that provides detected objects
    subscription_ = this->create_subscription<ufil_msgs::msg::ObjectList>(
        "measured_objects", 10,
      std::bind(&PedestrianTrackerNode::processDetections, this, std::placeholders::_1));

    // Publisher for broadcasting tracked pedestrian data
    publisher_ = this->create_publisher<ufil_msgs::msg::ObjectList>("tracked_objects", 10);

    // Initialize the tracker
    initializeTracker();
  }

private:
  /**
   * @brief Initializes the pedestrian tracker.
   */
  void initializeTracker()
  {
    tracker_ = std::make_unique<ufil_examples_pedestrian_tracker::PedestrianTracker>();
    last_update_time_ = this->now();
  }

  /**
   * @brief Callback function for processing incoming object detections.
   * @param msg The received ObjectList message containing detected objects.
   */
  void processDetections(const ufil_msgs::msg::ObjectList & msg)
  {
    auto current_time = this->now();
    double time_delta = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;

    RCLCPP_WARN(this->get_logger(), "Received update with delta time %f seconds.", time_delta);


    // Check if the time gap is too large, indicating a possible jump in timestamps
    if (time_delta < 0.0 || time_delta > 1.0) {
      RCLCPP_WARN(this->get_logger(), "Time jump detected, resetting tracker.");
      initializeTracker();
    }

    // Convert ROS message to UFIL-compatible format
    auto ufil_detections =
      ufil_ros::measurementsFromMsg<ufil_examples_pedestrian_tracker::Detection>(msg);
    auto ufil_time = ufil::from_nanoseconds<ufil::type::Timestamp>(current_time.nanoseconds());

    // Update the tracker with new detections
    tracker_->update(std::move(ufil_detections), std::move(ufil_time));

    // Convert tracked objects back to ROS message format
    ufil_msgs::msg::ObjectList tracked_objects = ufil_ros::tracksToMsg(tracker_->tracks());
    tracked_objects.header = msg.header;

    // Publish the tracked objects
    publisher_->publish(tracked_objects);
  }

  // Publisher for tracked objects
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_;
  // Subscriber for detected objects
  rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr subscription_;
  // Unique pointer to pedestrian tracker
  ufil_examples_pedestrian_tracker::PedestrianTracker::UniquePtr tracker_;

  rclcpp::Time last_update_time_;  ///< Last update time for detecting time jumps
};

/**
 * @brief Main function to start the pedestrian tracker node.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PedestrianTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
