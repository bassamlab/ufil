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


#include <mqtt/async_client.h>
#include <rclcpp/rclcpp.hpp>
#include <ufil_msgs/msg/object_list.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <ufil_ros/json_ros.hpp>

class MqttPublisherNode : public rclcpp::Node,
  public virtual mqtt::callback,
  public virtual mqtt::iaction_listener
{
private:
  bool connected_ = false;
  std::shared_ptr<mqtt::async_client> client_;

  rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr object_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_subscriber_;

  std::string protocol_;
  std::string host_;
  int port_;
  std::string id_;
  std::string source_topic_;
  std::string mqtt_topic_;
  std::string message_type_;

  // --------------------------------------------------------------------------
  // Generic MQTT publish backend
  // --------------------------------------------------------------------------

  template<typename MsgT>
  void publish_to_mqtt(const MsgT & msg)
  {
    if (!client_ || !connected_) {
      return;
    }

    try {
      std::string payload = ufil_ros::to_json(msg);
      auto mqtt_msg = mqtt::make_message(mqtt_topic_, payload);

      client_->publish(mqtt_msg);

      RCLCPP_INFO(this->get_logger(),
                  "Published MQTT message to topic: %s",
                  mqtt_topic_.c_str());
    } catch (const mqtt::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to publish MQTT message: %s",
                   e.what());
    }
  }

public:
  MqttPublisherNode()
  : Node("mqtt_publisher_node")
  {
    // ----------------------------------------------------------------------
    // Parameters
    // ----------------------------------------------------------------------

    this->declare_parameter("protocol", "tcp");
    this->declare_parameter("host", "localhost");
    this->declare_parameter("port", 1883);
    this->declare_parameter("id", "mqtt_publisher_node");
    this->declare_parameter("source_topic", "lab/groundtruth");
    this->declare_parameter("mqtt_topic", "test");
    this->declare_parameter("message_type", "object_list");

    this->get_parameter("protocol", protocol_);
    this->get_parameter("host", host_);
    this->get_parameter("port", port_);
    this->get_parameter("id", id_);
    this->get_parameter("source_topic", source_topic_);
    this->get_parameter("mqtt_topic", mqtt_topic_);
    this->get_parameter("message_type", message_type_);

    RCLCPP_INFO(this->get_logger(), "MQTT Publisher Node Parameters:");
    RCLCPP_INFO(this->get_logger(), "Protocol: %s", protocol_.c_str());
    RCLCPP_INFO(this->get_logger(), "Host: %s", host_.c_str());
    RCLCPP_INFO(this->get_logger(), "Port: %i", port_);
    RCLCPP_INFO(this->get_logger(), "Client ID: %s", id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Source Topic: %s", source_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "MQTT Topic: %s", mqtt_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Message Type: %s", message_type_.c_str());

    // ----------------------------------------------------------------------
    // Create correct subscription
    // ----------------------------------------------------------------------

    if (message_type_ == "object_list") {
      object_subscriber_ =
        this->create_subscription<ufil_msgs::msg::ObjectList>(
          source_topic_,
          10,
        [this](const ufil_msgs::msg::ObjectList & msg)
        {
          publish_to_mqtt(msg);
          });
    } else if (message_type_ == "occupancy_grid") {
      grid_subscriber_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          source_topic_,
          10,
        [this](const nav_msgs::msg::OccupancyGrid & msg)
        {
          publish_to_mqtt(msg);
          });

    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Unsupported message_type: %s",
                   message_type_.c_str());
      rclcpp::shutdown();
      return;
    }

    // ----------------------------------------------------------------------
    // MQTT Setup
    // ----------------------------------------------------------------------

    std::string uri = protocol_ + "://" + host_ + ":" + std::to_string(port_);

    auto connect_opts =
      mqtt::connect_options_builder()
      .ssl(mqtt::ssl_options())
      .automatic_reconnect(true)
      .finalize();

    try {
      client_ = std::make_shared<mqtt::async_client>(
        uri, id_, mqtt::create_options());

      client_->set_callback(*this);
      client_->connect(connect_opts);
    } catch (const mqtt::exception & e) {
      RCLCPP_ERROR(this->get_logger(),
                   "MQTT connection failed: %s",
                   e.what());
      rclcpp::shutdown();
    }
  }

  // --------------------------------------------------------------------------
  // MQTT Callbacks
  // --------------------------------------------------------------------------

  void connected(const std::string & cause) override
  {
    connected_ = true;
    RCLCPP_INFO(this->get_logger(),
                "MQTT connected (cause: %s).",
                cause.c_str());
  }

  void connection_lost(const std::string & cause) override
  {
    connected_ = false;
    RCLCPP_ERROR(this->get_logger(),
                 "MQTT connection lost (cause: %s).",
                 cause.c_str());
  }

  void message_arrived(mqtt::const_message_ptr) override
  {
    RCLCPP_INFO(this->get_logger(), "MQTT message received.");
  }

  void delivery_complete(mqtt::delivery_token_ptr token) override
  {
    RCLCPP_INFO(this->get_logger(),
                "MQTT message delivery complete (return code: %d).",
                token->get_return_code());
  }

  void on_success(const mqtt::token & token) override
  {
    RCLCPP_INFO(this->get_logger(),
                "MQTT connection success (return code: %d).",
                token.get_return_code());
  }

  void on_failure(const mqtt::token & token) override
  {
    RCLCPP_ERROR(this->get_logger(),
                 "MQTT connection failure (return code: %d).",
                 token.get_return_code());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MqttPublisherNode>();

  if (!rclcpp::ok()) {
    RCLCPP_ERROR(node->get_logger(),
      "Failed to start MQTT publisher node due to initialization error.");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Started MQTT publisher.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Stopped MQTT publisher.");

  return 0;
}
