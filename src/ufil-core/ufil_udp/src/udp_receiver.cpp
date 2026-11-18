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

#include "ufil_udp/udp_receiver.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/udp.hpp>

namespace ufil_udp
{

class UdpReceiver::Impl
{
public:
  Impl(const ReceiverConfig & config, PacketCallback on_packet, ErrorCallback on_error)
  : on_packet_(std::move(on_packet)), on_error_(std::move(on_error)), socket_(io_)
  {
    if (!on_packet_) {
      throw std::invalid_argument("the packet callback must be set");
    }
    const boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::udp::v4(), config.listen_port);
    boost::system::error_code error;
    socket_.open(endpoint.protocol(), error);
    if (!error) {
      socket_.bind(endpoint, error);
    }
    if (error) {
      throw std::runtime_error(
              "cannot bind UDP port " + std::to_string(config.listen_port) + ": " +
              error.message());
    }
    status_.local_port = socket_.local_endpoint().port();
    status_.receiving = true;

    startReceive();
    thread_ = std::thread([this] {io_.run();});
  }

  ~Impl() {stop();}

  Impl(const Impl &) = delete;
  Impl & operator=(const Impl &) = delete;

  void stop()
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    if (!thread_.joinable()) {
      return;
    }
    // Pending handlers are not called after run() returned; the one currently running finishes.
    io_.stop();
    thread_.join();
    boost::system::error_code ignored;
    socket_.close(ignored);
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    status_.receiving = false;
  }

  ReceiverStatus status() const
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
  }

private:
  void startReceive()
  {
    socket_.async_receive_from(
      boost::asio::buffer(buffer_), source_,
      [this](const boost::system::error_code & error, std::size_t length) {
        onReceive(error, length);
      });
  }

  void onReceive(const boost::system::error_code & error, std::size_t length)
  {
    if (error == boost::asio::error::operation_aborted) {
      return;
    }
    if (error) {
      reportError("receive failed: " + error.message());
    } else {
      Packet packet;
      packet.payload.assign(buffer_.begin(), buffer_.begin() + length);
      packet.source_address = source_.address().to_string();
      packet.source_port = source_.port();
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        ++status_.packets;
        status_.bytes += length;
        status_.last_packet_time = std::chrono::steady_clock::now();
      }
      try {
        on_packet_(std::move(packet));
      } catch (const std::exception & exception) {
        reportError(std::string("packet callback failed: ") + exception.what());
      } catch (...) {
        reportError("packet callback failed with an unknown exception");
      }
    }
    startReceive();
  }

  void reportError(const std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      ++status_.errors;
      status_.last_error = message;
    }
    if (on_error_) {
      try {
        on_error_(message);
      } catch (...) {
        // Already counted; the error callback must not stop the reception.
      }
    }
  }

  PacketCallback on_packet_;
  ErrorCallback on_error_;

  boost::asio::io_context io_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint source_;
  /// Large enough for the maximum UDP payload, so datagrams are never truncated.
  std::array<std::uint8_t, 65536> buffer_{};
  std::thread thread_;
  std::mutex stop_mutex_;

  mutable std::mutex status_mutex_;
  ReceiverStatus status_;
};

UdpReceiver::UdpReceiver(
  const ReceiverConfig & config, PacketCallback on_packet, ErrorCallback on_error)
: impl_(std::make_unique<Impl>(config, std::move(on_packet), std::move(on_error)))
{
}

UdpReceiver::~UdpReceiver() = default;

void UdpReceiver::stop() {impl_->stop();}

std::uint16_t UdpReceiver::localPort() const {return impl_->status().local_port;}

ReceiverStatus UdpReceiver::status() const {return impl_->status();}

}  // namespace ufil_udp
