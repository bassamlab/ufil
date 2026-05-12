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

#include <oscpack/ip/UdpSocket.h>
#include <oscpack/osc/OscOutboundPacketStream.h>

#include <rclcpp/rclcpp.hpp>

#include "ufil_examples_cpm_lab_adapter/message_converter.hpp"

#define OUTPUT_BUFFER_SIZE 65536

void to_osc(
  const cpm_lab_map_msgs::msg::LaneControlLight & msg,
  osc::OutboundPacketStream & stream)
{
// Reuse existing JSON-like serialization
  std::string json_payload = ufil_examples_cpm_lab_adapter::to_json(msg);

  // Add as single OSC string argument
  stream << json_payload.c_str();
}

class OscPublisherNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<cpm_lab_map_msgs::msg::LaneControlLight>::SharedPtr subscriber_;

  std::string host_;
  int port_;
  std::string source_topic_;
  std::string osc_address_;

  std::unique_ptr<UdpTransmitSocket> socket_;

  void publish_to_osc(const cpm_lab_map_msgs::msg::LaneControlLight & msg)
  {
    if (!socket_) {
      return;
    }

    try {
      char buffer[OUTPUT_BUFFER_SIZE];
      osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

      p << osc::BeginMessage(osc_address_.c_str());

      to_osc(msg, p);  // resolves via overload

      p << osc::EndMessage;

      socket_->Send(p.Data(), p.Size());

      RCLCPP_INFO(this->get_logger(),
                  "Published OSC message to %s:%d (address: %s)",
                  host_.c_str(), port_, osc_address_.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to send OSC message: %s", e.what());
    }
  }

public:
  OscPublisherNode()
  : Node("osc_publisher_node")
  {
    this->declare_parameter("host", "127.0.0.1");
    this->declare_parameter("port", 9000);
    this->declare_parameter("source_topic", "lab/groundtruth");
    this->declare_parameter("osc_address", "/objects");

    this->get_parameter("host", host_);
    this->get_parameter("port", port_);
    this->get_parameter("source_topic", source_topic_);
    this->get_parameter("osc_address", osc_address_);

    RCLCPP_INFO(this->get_logger(), "OSC Publisher Node Parameters:");
    RCLCPP_INFO(this->get_logger(), "Host: %s", host_.c_str());
    RCLCPP_INFO(this->get_logger(), "Port: %i", port_);
    RCLCPP_INFO(this->get_logger(), "Source Topic: %s", source_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "OSC Address: %s", osc_address_.c_str());

    try {
      IpEndpointName endpoint(host_.c_str(), port_);
      socket_ = std::make_unique<UdpTransmitSocket>(endpoint);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to create OSC socket: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    subscriber_ = this->create_subscription<cpm_lab_map_msgs::msg::LaneControlLight>(
      source_topic_, 10,
      [this](const cpm_lab_map_msgs::msg::LaneControlLight & msg)
      {
        publish_to_osc(msg);
      });
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OscPublisherNode>();

  if (!rclcpp::ok()) {
    RCLCPP_ERROR(node->get_logger(),
      "Failed to start OSC publisher node due to initialization error.");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Started OSC publisher.");

  rclcpp::spin(node);

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Stopped OSC publisher.");

  return 0;
}
