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

#include "ufil_osc/osc_sender.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ufil_osc/osc_message.hpp"

namespace ufil_osc
{

namespace
{

/// Numeric "host:port" of a resolved address.
std::string toString(const sockaddr * address, socklen_t length)
{
  char host[NI_MAXHOST] = {};
  char port[NI_MAXSERV] = {};
  if (::getnameinfo(
      address, length, host, sizeof(host), port, sizeof(port),
      NI_NUMERICHOST | NI_NUMERICSERV) != 0)
  {
    return "?";
  }
  return std::string(host) + ":" + port;
}

}  // namespace

class OscSender::Impl
{
public:
  explicit Impl(OscConfig config)
  : config_(std::move(config))
  {
    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid OSC configuration: " + *error);
    }

    // IPv4 is preferred, e.g. "localhost" is sent to 127.0.0.1 rather than ::1.
    addrinfo * result = nullptr;
    int error = EAI_FAMILY;
    for (const int family : {AF_INET, AF_UNSPEC}) {
      addrinfo hints{};
      hints.ai_family = family;
      hints.ai_socktype = SOCK_DGRAM;
      error = ::getaddrinfo(
        config_.host.c_str(), std::to_string(config_.port).c_str(), &hints, &result);
      if (error == 0) {
        break;
      }
    }
    if (error != 0) {
      throw std::runtime_error(
              "cannot resolve OSC receiver '" + config_.host + "': " + ::gai_strerror(error));
    }
    std::memcpy(&address_, result->ai_addr, result->ai_addrlen);
    address_length_ = result->ai_addrlen;
    socket_ = ::socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    const int socket_error = errno;
    ::freeaddrinfo(result);
    if (socket_ < 0) {
      throw std::runtime_error(
              std::string("cannot open the OSC socket: ") + std::strerror(socket_error));
    }
    endpoint_ = toString(reinterpret_cast<const sockaddr *>(&address_), address_length_);
  }

  ~Impl()
  {
    ::close(socket_);
  }

  SendResult send(const std::string & payload)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t size = stringMessageSize(config_.address, payload.size());
    status_.last_packet_size = size;
    if (size > kMaxPacketSize) {
      return fail(
        SendResult::kTooLarge, "OSC packet of " + std::to_string(size) + " bytes exceeds " +
        std::to_string(kMaxPacketSize) + " bytes");
    }

    if (payload.find('\0') != std::string::npos) {
      return fail(SendResult::kFailed, "the payload contains a NUL character");
    }

    const std::vector<char> packet = encodeStringMessage(config_.address, payload);
    const ssize_t sent = ::sendto(
      socket_, packet.data(), packet.size(), 0, reinterpret_cast<const sockaddr *>(&address_),
      address_length_);
    if (sent < 0) {
      return fail(SendResult::kFailed, std::string("sendto failed: ") + std::strerror(errno));
    }
    ++status_.sent;
    status_.last_result = SendResult::kSent;
    return SendResult::kSent;
  }

  SenderStatus status() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }

  const OscConfig & config() const {return config_;}
  const std::string & endpoint() const {return endpoint_;}

private:
  /// Records a failed send; mutex_ must be held.
  SendResult fail(SendResult result, std::string error)
  {
    ++(result == SendResult::kTooLarge ? status_.too_large : status_.failed);
    status_.last_result = result;
    status_.last_error = std::move(error);
    return result;
  }

  const OscConfig config_;
  sockaddr_storage address_{};
  socklen_t address_length_ = 0;
  std::string endpoint_;
  int socket_ = -1;

  mutable std::mutex mutex_;
  SenderStatus status_;
};

OscSender::OscSender(OscConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
}

OscSender::~OscSender() = default;

SendResult OscSender::send(const std::string & payload) {return impl_->send(payload);}

SenderStatus OscSender::status() const {return impl_->status();}

const OscConfig & OscSender::config() const {return impl_->config();}

const std::string & OscSender::endpoint() const {return impl_->endpoint();}

}  // namespace ufil_osc
