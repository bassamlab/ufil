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

#include "ufil_mqtt/mqtt_client.hpp"

#include <mqtt/async_client.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace ufil_mqtt
{

namespace
{

constexpr auto kConnectTimeout = std::chrono::seconds(5);
constexpr auto kKeepAliveInterval = std::chrono::seconds(30);
constexpr auto kDisconnectTimeout = std::chrono::seconds(1);

/// The client library only reports a return code, e.g. "Failure" for a refused connection.
std::string tokenError(const std::string & action, const mqtt::token & token)
{
  return action + " failed: " + mqtt::exception::error_str(token.get_return_code()) + " (" +
         std::to_string(token.get_return_code()) + ")";
}

}  // namespace

const char * toString(ConnectionState state)
{
  switch (state) {
    case ConnectionState::kDisconnected:
      return "disconnected";
    case ConnectionState::kConnecting:
      return "connecting";
    case ConnectionState::kConnected:
      return "connected";
  }
  return "unknown";
}

class MqttClient::Impl
{
public:
  Impl(MqttConfig config, EventCallback on_event)
  : config_(std::move(config)),
    on_event_(std::move(on_event)),
    connect_listener_(*this),
    delivery_listener_(*this)
  {
    if (const auto error = validate(config_)) {
      throw std::invalid_argument("invalid MQTT configuration: " + *error);
    }
    // No persistence: messages are not stored on disk while the client is disconnected.
    client_ = std::make_unique<mqtt::async_client>(
      serverUri(config_), config_.client_id, static_cast<mqtt::iclient_persistence *>(nullptr));
    client_->set_connected_handler([this](const std::string &) {onConnected();});
    client_->set_connection_lost_handler(
      [this](const std::string & cause) {onConnectionLost(cause);});
  }

  ~Impl()
  {
    try {
      // Refused by the client library while a connection attempt is in progress; the client
      // is destroyed below, before the objects the callbacks use.
      client_->disable_callbacks();
    } catch (const mqtt::exception &) {
    }
    try {
      if (client_->is_connected()) {
        client_->disconnect()->wait_for(kDisconnectTimeout);
      }
    } catch (const mqtt::exception &) {
      // The connection is closed when the client is destroyed anyway.
    }
    client_.reset();
  }

  bool connect()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (status_.state != ConnectionState::kDisconnected) {
        return false;
      }
      status_.state = ConnectionState::kConnecting;
      ++status_.connect_attempts;
    }

    mqtt::connect_options_builder builder;
    builder.clean_session(true)
    .connect_timeout(kConnectTimeout)
    .keep_alive_interval(kKeepAliveInterval);
    if (config_.protocol == "ssl") {
      builder.ssl(mqtt::ssl_options());
    }
    try {
      client_->connect(builder.finalize(), nullptr, connect_listener_);
    } catch (const mqtt::exception & error) {
      onConnectFailed(error.what());
    }
    return true;
  }

  PublishResult publish(const std::string & payload)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (status_.state != ConnectionState::kConnected) {
        ++status_.dropped;
        return PublishResult::kNotConnected;
      }
    }
    try {
      client_->publish(mqtt::make_message(config_.topic, payload), nullptr, delivery_listener_);
    } catch (const mqtt::exception & error) {
      onPublishFailed(error.what());
      return PublishResult::kFailed;
    }
    return PublishResult::kQueued;
  }

  MqttStatus status() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }

  const MqttConfig & config() const {return config_;}

private:
  /// Reports the result of connection attempts.
  class ConnectListener : public mqtt::iaction_listener
  {
public:
    explicit ConnectListener(Impl & impl)
    : impl_(impl) {}
    void on_success(const mqtt::token &) override {impl_.onConnected();}
    void on_failure(const mqtt::token & token) override
    {
      impl_.onConnectFailed(tokenError("connection", token));
    }

private:
    Impl & impl_;
  };

  /// Reports the result of publications.
  class DeliveryListener : public mqtt::iaction_listener
  {
public:
    explicit DeliveryListener(Impl & impl)
    : impl_(impl) {}
    void on_success(const mqtt::token &) override {impl_.onSent();}
    void on_failure(const mqtt::token & token) override
    {
      impl_.onPublishFailed(tokenError("delivery", token));
    }

private:
    Impl & impl_;
  };

  void emit(const MqttEvent & event) const
  {
    if (on_event_) {
      on_event_(event);
    }
  }

  void onConnected()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // The connected handler and the connect listener both report the same connection.
      if (status_.state == ConnectionState::kConnected) {
        return;
      }
      status_.state = ConnectionState::kConnected;
      status_.consecutive_connect_failures = 0;
    }
    emit({MqttEvent::Type::kConnected, {}, 0});
  }

  void onConnectFailed(const std::string & reason)
  {
    std::uint64_t attempt = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (status_.state != ConnectionState::kConnecting) {
        return;
      }
      status_.state = ConnectionState::kDisconnected;
      attempt = ++status_.consecutive_connect_failures;
      status_.last_error = reason;
    }
    emit({MqttEvent::Type::kConnectFailed, reason, attempt});
  }

  void onConnectionLost(const std::string & cause)
  {
    const std::string reason = cause.empty() ? "connection lost" : cause;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (status_.state != ConnectionState::kConnected) {
        return;
      }
      status_.state = ConnectionState::kDisconnected;
      ++status_.connection_losses;
      status_.last_error = reason;
    }
    emit({MqttEvent::Type::kConnectionLost, reason, 0});
  }

  void onSent()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++status_.sent;
  }

  void onPublishFailed(const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++status_.failed;
      status_.last_error = reason;
    }
    emit({MqttEvent::Type::kPublishFailed, reason, 0});
  }

  const MqttConfig config_;
  const EventCallback on_event_;

  mutable std::mutex mutex_;
  MqttStatus status_;

  ConnectListener connect_listener_;
  DeliveryListener delivery_listener_;
  // Last member: destroyed first, so no callback runs on a partly destroyed object.
  std::unique_ptr<mqtt::async_client> client_;
};

MqttClient::MqttClient(MqttConfig config, EventCallback on_event)
: impl_(std::make_unique<Impl>(std::move(config), std::move(on_event)))
{
}

MqttClient::~MqttClient() = default;

bool MqttClient::connect() {return impl_->connect();}

PublishResult MqttClient::publish(const std::string & payload) {return impl_->publish(payload);}

MqttStatus MqttClient::status() const {return impl_->status();}

const MqttConfig & MqttClient::config() const {return impl_->config();}

}  // namespace ufil_mqtt
