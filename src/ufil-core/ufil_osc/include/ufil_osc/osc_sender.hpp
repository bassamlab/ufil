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

#ifndef UFIL_OSC__OSC_SENDER_HPP_
#define UFIL_OSC__OSC_SENDER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "ufil_osc/config.hpp"
#include "ufil_osc/visibility_control.hpp"

namespace ufil_osc
{

enum class SendResult
{
  kSent,
  /// The OSC packet does not fit into one UDP datagram and was not sent.
  kTooLarge,
  /// The socket reported an error.
  kFailed,
};

struct SenderStatus
{
  std::uint64_t sent = 0;
  std::uint64_t too_large = 0;
  std::uint64_t failed = 0;
  /// Result of the last send(), std::nullopt if nothing was sent yet.
  std::optional<SendResult> last_result;
  /// Size of the last OSC packet [bytes].
  std::size_t last_packet_size = 0;
  /// Last error, empty if there was none.
  std::string last_error;
};

/**
 * @brief Sends payloads as OSC 1.0 messages with a single string argument over UDP.
 *
 * Each message is one UDP datagram: payloads whose OSC packet exceeds kMaxPacketSize are rejected.
 * All methods are thread-safe.
 */
class UFIL_OSC_PUBLIC OscSender
{
public:
  /// Largest UDP payload over IPv4 [bytes].
  static constexpr std::size_t kMaxPacketSize = 65507;

  /**
   * @throws std::invalid_argument if the configuration is invalid.
   * @throws std::runtime_error if the host cannot be resolved or the socket cannot be opened.
   */
  explicit OscSender(OscConfig config);
  ~OscSender();

  OscSender(const OscSender &) = delete;
  OscSender & operator=(const OscSender &) = delete;

  /// Sends payload, which must not contain NUL characters, as the string argument of a message.
  SendResult send(const std::string & payload);

  SenderStatus status() const;
  const OscConfig & config() const;
  /// Resolved receiver, e.g. "127.0.0.1:9000".
  const std::string & endpoint() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ufil_osc

#endif  // UFIL_OSC__OSC_SENDER_HPP_
