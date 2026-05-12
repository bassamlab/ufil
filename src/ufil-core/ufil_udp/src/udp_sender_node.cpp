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

#include <chrono>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#include <boost/asio.hpp>

class UdpSender : public rclcpp::Node
{
public:
  UdpSender()
  : Node("udp_sender"), socket_(io_context_), packet_count_(0)
  {
    declare_parameter<std::string>("target_ip", "127.0.0.1");
    declare_parameter<int>("target_port", 5000);
    declare_parameter<bool>("enable_stats", false);

    target_ip_ = get_parameter("target_ip").as_string();
    target_port_ = get_parameter("target_port").as_int();
    enable_stats_ = get_parameter("enable_stats").as_bool();

    endpoint_ = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(target_ip_),
      target_port_);

    socket_.open(boost::asio::ip::udp::v4());
    RCLCPP_INFO(this->get_logger(), "UDP forwarder initialized. Sending to %s:%d",
      target_ip_.c_str(), target_port_);

    subscription_ = create_subscription<udp_msgs::msg::UdpPacket>(
      "udp_input", rclcpp::QoS(10),
      std::bind(&UdpSender::udp_callback, this, std::placeholders::_1));

    if (enable_stats_) {
      stats_timer_ = create_wall_timer(std::chrono::seconds(10),
        std::bind(&UdpSender::print_stats, this));
    }
  }

private:
  void udp_callback(const udp_msgs::msg::UdpPacket::SharedPtr msg)
  {
    try {
      socket_.send_to(boost::asio::buffer(msg->data), endpoint_);
      packet_count_.fetch_add(1, std::memory_order_relaxed);
    } catch (std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "UDP send error: %s", e.what());
    }
  }

  void print_stats()
  {
    RCLCPP_INFO(this->get_logger(), "Forwarded %d UDP packets in the last 10 seconds",
      packet_count_.exchange(0, std::memory_order_relaxed));
  }

  rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr subscription_;
  boost::asio::io_context io_context_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::string target_ip_;
  int target_port_;
  bool enable_stats_;
  std::atomic<int> packet_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpSender>());
  rclcpp::shutdown();
  return 0;
}
