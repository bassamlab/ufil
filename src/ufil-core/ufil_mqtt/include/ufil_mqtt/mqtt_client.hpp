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

#ifndef UFIL_MQTT__MQTT_CLIENT_HPP_
#define UFIL_MQTT__MQTT_CLIENT_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "ufil_mqtt/config.hpp"
#include "ufil_mqtt/visibility_control.hpp"

namespace ufil_mqtt
{

enum class ConnectionState
{
  kDisconnected,
  kConnecting,
  kConnected,
};

/// "disconnected", "connecting" or "connected".
UFIL_MQTT_PUBLIC const char * toString(ConnectionState state);

enum class PublishResult
{
  /// Handed to the client library, the delivery is reported asynchronously.
  kQueued,
  /// Dropped, the client is not connected to the broker.
  kNotConnected,
  /// The client library rejected the message.
  kFailed,
};

/// Something that happened in the client, reported through the event callback.
struct MqttEvent
{
  enum class Type
  {
    kConnected,
    kConnectFailed,
    kConnectionLost,
    kPublishFailed,
  };

  Type type;
  /// Reason reported by the client library, empty if there is none.
  std::string reason;
  /// kConnectFailed: number of consecutive failed connection attempts, starting at 1.
  std::uint64_t attempt = 0;
};

struct MqttStatus
{
  ConnectionState state = ConnectionState::kDisconnected;
  /// Connection attempts since the client was created.
  std::uint64_t connect_attempts = 0;
  /// Failed connection attempts since the last successful connection.
  std::uint64_t consecutive_connect_failures = 0;
  std::uint64_t connection_losses = 0;
  /// Messages written to the broker connection.
  std::uint64_t sent = 0;
  /// Messages dropped because the client was not connected.
  std::uint64_t dropped = 0;
  /// Messages that could not be published or delivered.
  std::uint64_t failed = 0;
  /// Last connection or publication error, empty if there was none.
  std::string last_error;
};

/**
 * @brief Publishes payloads on one MQTT topic (QoS 0, not retained).
 *
 * The client never connects by itself: connect() starts an asynchronous connection attempt and
 * must be called again after a failed attempt or a lost connection, e.g. periodically. Messages
 * published while not connected are dropped.
 *
 * All methods are thread-safe. The event callback is called from the thread of the client library
 * or from the calling thread, without internal locks held; it must be thread-safe and must not
 * destroy the client.
 */
class UFIL_MQTT_PUBLIC MqttClient
{
public:
  using EventCallback = std::function<void (const MqttEvent &)>;

  /// @throws std::invalid_argument if the configuration is invalid.
  explicit MqttClient(MqttConfig config, EventCallback on_event = {});
  /// Disconnects from the broker.
  ~MqttClient();

  MqttClient(const MqttClient &) = delete;
  MqttClient & operator=(const MqttClient &) = delete;

  /// Starts a connection attempt if the client is disconnected. @return true if it was started.
  bool connect();

  PublishResult publish(const std::string & payload);

  MqttStatus status() const;
  const MqttConfig & config() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ufil_mqtt

#endif  // UFIL_MQTT__MQTT_CLIENT_HPP_
