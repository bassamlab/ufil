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

#ifndef UFIL_UDP__UDP_RECEIVER_HPP_
#define UFIL_UDP__UDP_RECEIVER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "ufil_udp/config.hpp"
#include "ufil_udp/packet.hpp"
#include "ufil_udp/visibility_control.hpp"

namespace ufil_udp
{

/// Counters and state of a UdpReceiver.
struct ReceiverStatus
{
  /// Port the socket is bound to.
  std::uint16_t local_port = 0;
  /// False after stop().
  bool receiving = false;
  std::uint64_t packets = 0;
  /// Payload bytes of all received packets.
  std::uint64_t bytes = 0;
  /// Failed receive operations, including exceptions thrown by the packet callback.
  std::uint64_t errors = 0;
  /// Time the last packet was received, empty if none was received yet.
  std::optional<std::chrono::steady_clock::time_point> last_packet_time;
  /// Description of the last error, empty if none occurred.
  std::string last_error;
};

/**
 * @brief Receives UDP datagrams on a port in a background thread.
 *
 * Every received datagram is passed to the packet callback, every failed receive operation to the
 * error callback. Both callbacks are called from the receive thread, one at a time, and must not
 * call stop() or destroy the receiver. Exceptions thrown by a callback are counted as errors and
 * do not stop the reception.
 */
class UFIL_UDP_PUBLIC UdpReceiver
{
public:
  using PacketCallback = std::function<void (Packet && packet)>;
  using ErrorCallback = std::function<void (const std::string & error)>;

  /**
   * @brief Binds the socket and starts receiving.
   * @throws std::invalid_argument if on_packet is empty.
   * @throws std::runtime_error if the socket cannot be opened or bound (e.g. port in use).
   */
  UdpReceiver(
    const ReceiverConfig & config, PacketCallback on_packet, ErrorCallback on_error = {});

  /// Stops receiving, see stop().
  ~UdpReceiver();

  UdpReceiver(const UdpReceiver &) = delete;
  UdpReceiver & operator=(const UdpReceiver &) = delete;

  /// Stops receiving and closes the socket. When stop() returns, no callback is running and none
  /// will be called anymore. Calling it again has no effect.
  void stop();

  /// Port the socket is bound to; differs from the configured port if that was 0.
  std::uint16_t localPort() const;

  ReceiverStatus status() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ufil_udp

#endif  // UFIL_UDP__UDP_RECEIVER_HPP_
