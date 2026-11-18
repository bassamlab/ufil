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

#ifndef UDP_RECEIVER_HPP_
#define UDP_RECEIVER_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace ufil_osc::test
{

/// Decoded OSC message with a single string argument.
struct OscMessage
{
  std::string address;
  std::string argument;
};

/// UDP socket on 127.0.0.1 receiving OSC messages.
class UdpReceiver
{
public:
  UdpReceiver()
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t length = sizeof(address);
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
      ::getsockname(fd_, reinterpret_cast<sockaddr *>(&address), &length) != 0)
    {
      throw std::runtime_error("cannot open the UDP receiver socket");
    }
    port_ = ntohs(address.sin_port);
  }

  ~UdpReceiver() {::close(fd_);}

  int port() const {return port_;}

  std::optional<OscMessage> receive(std::chrono::milliseconds timeout)
  {
    pollfd entry{fd_, POLLIN, 0};
    if (::poll(&entry, 1, static_cast<int>(timeout.count())) <= 0) {
      return std::nullopt;
    }
    std::vector<char> buffer(65536);
    const ssize_t size = ::recv(fd_, buffer.data(), buffer.size(), 0);
    if (size <= 0) {
      return std::nullopt;
    }
    buffer.resize(static_cast<std::size_t>(size));
    std::size_t offset = 0;
    OscMessage message;
    message.address = readString(buffer, offset);
    if (readString(buffer, offset) != ",s") {
      throw std::runtime_error("expected an OSC message with one string argument");
    }
    message.argument = readString(buffer, offset);
    if (offset != buffer.size()) {
      throw std::runtime_error("trailing bytes after the OSC message");
    }
    return message;
  }

private:
  /// Reads a NUL terminated string padded to four bytes.
  static std::string readString(const std::vector<char> & buffer, std::size_t & offset)
  {
    std::size_t end = offset;
    while (end < buffer.size() && buffer[end] != '\0') {
      ++end;
    }
    const std::size_t next = (end + 4) & ~std::size_t{3};
    if (end == buffer.size() || next > buffer.size()) {
      throw std::runtime_error("malformed OSC string");
    }
    for (std::size_t i = end; i < next; ++i) {
      if (buffer[i] != '\0') {
        throw std::runtime_error("OSC string padding is not NUL");
      }
    }
    std::string value(buffer.begin() + offset, buffer.begin() + end);
    offset = next;
    return value;
  }

  int fd_ = -1;
  int port_ = 0;
};

}  // namespace ufil_osc::test

#endif  // UDP_RECEIVER_HPP_
