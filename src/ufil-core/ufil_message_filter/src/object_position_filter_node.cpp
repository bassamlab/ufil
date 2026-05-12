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


#include <ranges>
#include <vector>
#include <ufil_msgs/msg/object_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "position_filter.hpp"

class ObjectPositionFilter : public PositionFilter<ufil_msgs::msg::ObjectStamped, double>
{
protected:
  void toPoint(const ufil_msgs::msg::ObjectStamped & message, Point & point) override
  {
    point.x(message.object.state.state.x);
    point.y(message.object.state.state.y);
  }

public:
  explicit ObjectPositionFilter(std::vector<Point> && polygon_points)
  : PositionFilter(std::move(polygon_points))
  {
  }
};

class ObjectPositionFilterNode : public rclcpp::Node
{
private:
  using Point = ObjectPositionFilter::Point;

  std::unique_ptr<ObjectPositionFilter> object_position_filter_;

  // Subscriptions
  rclcpp::Subscription<ufil_msgs::msg::ObjectStamped>::SharedPtr object_subscription_{};

  // Publishers
  rclcpp::Publisher<ufil_msgs::msg::ObjectStamped>::SharedPtr object_publisher_{};

  void onObject(ufil_msgs::msg::ObjectStamped::UniquePtr msg)
  {
    if(this->object_position_filter_->filter(*msg)) {
      this->object_publisher_->publish(*msg);
    }
  }

public:
  ObjectPositionFilterNode()
  : Node("object_position_filter_node")
  {
    // Load parameter
    auto area_of_interest_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    area_of_interest_param_desc.description =
      "Area of interest. Double array of even size (x1, y1, x2, y2, ...)";
    area_of_interest_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    auto area_of_interest_vector = this->declare_parameter(
      "area_of_interest", std::vector<double>{}, area_of_interest_param_desc);

    if (area_of_interest_vector.empty() || area_of_interest_vector.size() % 2 != 0) {
      throw std::runtime_error(
        "The area_of_interest parameter must contain an even number of elements and not be empty.");
    }

    std::vector<Point> area_of_interest;

    for (size_t i = 0; i < area_of_interest_vector.size(); i += 2) {
      Point point;
      point.x(area_of_interest_vector[i]);
      point.y(area_of_interest_vector[i + 1]);
      area_of_interest.push_back(point);
    }

    RCLCPP_INFO(this->get_logger(), "Area of interest:");
    // You can now use area_of_interest as needed in your application.
    for (const auto & point : area_of_interest) {
      RCLCPP_INFO(this->get_logger(), "Point(x: %.2f, y: %.2f)", point.x(), point.y());
    }

    // Repeat first point to close polygon
    Point point;
    point.x(area_of_interest_vector[0]);
    point.y(area_of_interest_vector[1]);
    area_of_interest.push_back(point);

    this->object_position_filter_ =
      std::make_unique<ObjectPositionFilter>(std::move(area_of_interest));

    auto qos = rclcpp::SystemDefaultsQoS();

    // Subscription
    this->object_subscription_ = this->create_subscription<ufil_msgs::msg::ObjectStamped>(
      "object", qos,
      [this](ufil_msgs::msg::ObjectStamped::UniquePtr msg) {this->onObject(std::move(msg));});

    // Publishers
    this->object_publisher_ =
      this->create_publisher<ufil_msgs::msg::ObjectStamped>("object_filtered", qos);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectPositionFilterNode>();

  RCLCPP_INFO(node->get_logger(), "Started object position filter.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Stopped object position filter.");

  return 0;
}
