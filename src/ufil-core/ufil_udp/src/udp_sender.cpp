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

#include "ufil_udp/udp_sender.hpp"

#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/asio/ip/udp.hpp>

namespace ufil_udp
{

class UdpSender::Impl
{
public:
  explicit Impl(const SenderConfig & config)
  : config_(config), socket_(io_)
  {
    if (const auto error = validate(config)) {
      throw std::invalid_argument(*error);
    }
    target_ = boost::asio::ip::udp::endpoint(
      boost::asio::ip::make_address(config.target_address), config.target_port);

    boost::system::error_code error;
    socket_.open(target_.protocol(), error);
    if (!error && target_.address().is_v4()) {
      // Allows sending to a broadcast address.
      socket_.set_option(boost::asio::socket_base::broadcast(true), error);
    }
    if (error) {
      throw std::runtime_error("cannot open UDP socket: " + error.message());
    }
  }

  std::optional<std::string> send(std::span<const std::uint8_t> payload)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    boost::system::error_code error;
    socket_.send_to(boost::asio::buffer(payload.data(), payload.size()), target_, 0, error);
    if (error) {
      ++status_.errors;
      status_.last_error = error.message();
      return status_.last_error;
    }
    ++status_.packets;
    status_.bytes += payload.size();
    status_.last_packet_time = std::chrono::steady_clock::now();
    return std::nullopt;
  }

  const SenderConfig & config() const {return config_;}

  SenderStatus status() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }

private:
  const SenderConfig config_;
  boost::asio::io_context io_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint target_;

  /// Protects the socket and the status.
  mutable std::mutex mutex_;
  SenderStatus status_;
};

UdpSender::UdpSender(const SenderConfig & config)
: impl_(std::make_unique<Impl>(config))
{
}

UdpSender::~UdpSender() = default;

std::optional<std::string> UdpSender::send(std::span<const std::uint8_t> payload)
{
  return impl_->send(payload);
}

const SenderConfig & UdpSender::config() const {return impl_->config();}

SenderStatus UdpSender::status() const {return impl_->status();}

}  // namespace ufil_udp
