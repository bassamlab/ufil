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

#include <memory>
#include <stdexcept>

#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "ufil_etsi/rsu_cam_generator.hpp"
#include "RsuCamGeneratorParameterRos.hpp"
#include "ufil_etsi_ros.hpp"

namespace ufil_etsi
{

/**
 * @brief ROS 2 component sending the CAMs of a road side unit every second.
 *
 * Publishes:   cam (etsi_its_cam_msgs/CAM)
 */
class RsuCamGeneratorNode : public rclcpp::Node
{
public:
  explicit RsuCamGeneratorNode(const rclcpp::NodeOptions & options)
  : Node("rsu_cam_generator_node", options),
    parameters_(loadParameters(*this)),
    generator_(parameters_.config())
  {
    publisher_ = create_publisher<etsi_its_cam_msgs::msg::CAM>("cam", 10);
    timer_ = rclcpp::create_timer(
      this, get_clock(), RsuCamGenerator::kGenerationInterval, [this] {onTimer();});

    const auto & config = parameters_.config();
    RCLCPP_INFO(
      get_logger(), "Sending CAMs of road side unit station %u%s at %.7f deg, %.7f deg, %.2f m.",
      config.station_id, parameters_.randomStationIdUsed() ? " (random id)" : "",
      config.position.latitude, config.position.longitude, config.altitude);
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static RsuCamGeneratorParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return RsuCamGeneratorParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  void onTimer()
  {
    const auto cam = generator_.generate(conversions::timeFromRos(now()));
    if (!cam) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Not sending CAMs: the ROS time plus time_shift_nanosec is before 2004, the generation "
        "delta time is undefined. Increase time_shift_nanosec.");
      return;
    }
    publisher_->publish(conversions::camToMsg(*cam));
  }

  RsuCamGeneratorParameterRos parameters_;
  RsuCamGenerator generator_;
  rclcpp::Publisher<etsi_its_cam_msgs::msg::CAM>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ufil_etsi

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_etsi::RsuCamGeneratorNode)
