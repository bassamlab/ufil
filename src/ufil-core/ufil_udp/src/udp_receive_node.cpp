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

#include <array>
#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <udp_msgs/msg/udp_packet.hpp>
#include <boost/asio.hpp>

class UdpReceiver : public rclcpp::Node
{
public:
  UdpReceiver()
  : Node("udp_receiver"),
    io_context_(),
    socket_(io_context_),
    packet_count_(0)
  {
    declare_parameter<int>("listen_port", 5000);
    declare_parameter<bool>("enable_stats", false);

    listen_port_ = get_parameter("listen_port").as_int();
    enable_stats_ = get_parameter("enable_stats").as_bool();

    endpoint_ = boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), listen_port_);
    socket_.open(endpoint_.protocol());
    socket_.bind(endpoint_);

    RCLCPP_INFO(this->get_logger(), "UDP listener initialized on port %d", listen_port_);

    publisher_ = create_publisher<udp_msgs::msg::UdpPacket>("udp_output", rclcpp::QoS(10));

    // Start async receive loop
    start_receive();

    // Run io_context in a background thread
    io_thread_ = std::thread([this]() {io_context_.run();});

    if (enable_stats_) {
      stats_timer_ = create_wall_timer(std::chrono::seconds(10),
        std::bind(&UdpReceiver::print_stats, this));
    }
  }

  ~UdpReceiver()
  {
    // Stop the io_context to break async operations
    socket_.cancel();
    io_context_.stop();
    if (io_thread_.joinable()) {
      io_thread_.join();
    }
    socket_.close();
  }

private:
  void start_receive()
  {
    socket_.async_receive_from(
      boost::asio::buffer(buffer_), sender_endpoint_,
      [this](boost::system::error_code ec, std::size_t length) {
        if (!ec && length > 0) {
          auto msg = std::make_shared<udp_msgs::msg::UdpPacket>();
          msg->data.assign(buffer_.begin(), buffer_.begin() + length);
          publisher_->publish(*msg);
          packet_count_.fetch_add(1, std::memory_order_relaxed);
        } else if (ec != boost::asio::error::operation_aborted) {
          RCLCPP_ERROR(this->get_logger(), "UDP receive error: %s", ec.message().c_str());
        }

        // Keep listening unless shutting down
        if (rclcpp::ok()) {
          start_receive();
        }
      });
  }

  void print_stats()
  {
    RCLCPP_INFO(this->get_logger(), "Received %d UDP packets in the last 10 seconds",
      packet_count_.exchange(0, std::memory_order_relaxed));
  }

  rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr publisher_;
  boost::asio::io_context io_context_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::thread io_thread_;
  std::array<uint8_t, 4096> buffer_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  int listen_port_;
  bool enable_stats_;
  std::atomic<int> packet_count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpReceiver>());
  rclcpp::shutdown();
  return 0;
}
