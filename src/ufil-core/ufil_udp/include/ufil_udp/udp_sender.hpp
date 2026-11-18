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

#ifndef UFIL_UDP__UDP_SENDER_HPP_
#define UFIL_UDP__UDP_SENDER_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>  // NOLINT(build/include_order): cpplint does not know C++20 headers
#include <string>

#include "ufil_udp/config.hpp"
#include "ufil_udp/visibility_control.hpp"

namespace ufil_udp
{

/// Counters and state of a UdpSender.
struct SenderStatus
{
  std::uint64_t packets = 0;
  /// Payload bytes of all sent packets.
  std::uint64_t bytes = 0;
  /// Packets that could not be sent.
  std::uint64_t errors = 0;
  /// Time the last packet was sent, empty if none was sent yet.
  std::optional<std::chrono::steady_clock::time_point> last_packet_time;
  /// Description of the last error, empty if none occurred.
  std::string last_error;
};

/// Sends UDP datagrams to a fixed target. send() may be called from several threads.
class UFIL_UDP_PUBLIC UdpSender
{
public:
  /**
   * @brief Opens the socket.
   * @throws std::invalid_argument if the configuration is invalid, see validate().
   * @throws std::runtime_error if the socket cannot be opened.
   */
  explicit UdpSender(const SenderConfig & config);
  ~UdpSender();

  UdpSender(const UdpSender &) = delete;
  UdpSender & operator=(const UdpSender &) = delete;

  /**
   * @brief Sends the payload as one datagram.
   * @return std::nullopt on success, otherwise a description of the error (e.g. message too long,
   *         network unreachable). The error is also counted in status().
   */
  [[nodiscard]] std::optional<std::string> send(std::span<const std::uint8_t> payload);

  const SenderConfig & config() const;

  SenderStatus status() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ufil_udp

#endif  // UFIL_UDP__UDP_SENDER_HPP_
