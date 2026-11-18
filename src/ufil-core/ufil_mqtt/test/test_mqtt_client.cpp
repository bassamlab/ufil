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

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "fake_mqtt_broker.hpp"
#include "ufil_mqtt/mqtt_client.hpp"

using ufil_mqtt::ConnectionState;
using ufil_mqtt::MqttClient;
using ufil_mqtt::MqttConfig;
using ufil_mqtt::MqttEvent;
using ufil_mqtt::PublishResult;

namespace
{

MqttConfig localConfig(int port)
{
  MqttConfig config;
  config.host = "127.0.0.1";
  config.port = port;
  config.client_id = "ufil_mqtt_test";
  config.topic = "ufil/test";
  return config;
}

bool waitFor(const std::function<bool()> & condition)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return condition();
}

/// Thread-safe record of the events of a client.
class EventLog
{
public:
  MqttClient::EventCallback callback()
  {
    return [this](const MqttEvent & event) {
             std::lock_guard<std::mutex> lock(mutex_);
             events_.push_back(event);
           };
  }

  std::vector<MqttEvent> events() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

  bool contains(MqttEvent::Type type) const
  {
    for (const auto & event : events()) {
      if (event.type == type) {
        return true;
      }
    }
    return false;
  }

private:
  mutable std::mutex mutex_;
  std::vector<MqttEvent> events_;
};

}  // namespace

TEST(MqttClient, RejectsInvalidConfiguration)
{
  MqttConfig config;
  config.port = 0;
  EXPECT_THROW(MqttClient{config}, std::invalid_argument);
}

TEST(MqttClient, StartsDisconnectedAndDropsMessages)
{
  MqttClient client(localConfig(ufil_mqtt::test::unusedPort()));
  EXPECT_EQ(client.status().state, ConnectionState::kDisconnected);
  EXPECT_EQ(client.publish("{}"), PublishResult::kNotConnected);
  EXPECT_EQ(client.status().dropped, 1u);
  EXPECT_EQ(client.status().sent, 0u);
  EXPECT_STREQ(ufil_mqtt::toString(ConnectionState::kDisconnected), "disconnected");
}

TEST(MqttClient, ReportsFailedConnectionWithoutBroker)
{
  EventLog log;
  MqttClient client(localConfig(ufil_mqtt::test::unusedPort()), log.callback());
  EXPECT_TRUE(client.connect());
  EXPECT_FALSE(client.connect()) << "a second attempt must not start while connecting";

  ASSERT_TRUE(waitFor([&] {return log.contains(MqttEvent::Type::kConnectFailed);}));
  const auto status = client.status();
  EXPECT_EQ(status.state, ConnectionState::kDisconnected);
  EXPECT_EQ(status.connect_attempts, 1u);
  EXPECT_EQ(status.consecutive_connect_failures, 1u);
  EXPECT_FALSE(status.last_error.empty());
  EXPECT_EQ(log.events().front().attempt, 1u);

  // Retrying counts the consecutive failures.
  EXPECT_TRUE(client.connect());
  ASSERT_TRUE(waitFor([&] {return client.status().consecutive_connect_failures == 2;}));
  EXPECT_EQ(log.events().back().attempt, 2u);
}

TEST(MqttClient, PublishesToBroker)
{
  ufil_mqtt::test::FakeMqttBroker broker;
  EventLog log;
  MqttClient client(localConfig(broker.port()), log.callback());
  client.connect();
  ASSERT_TRUE(waitFor([&] {return client.status().state == ConnectionState::kConnected;}));
  EXPECT_TRUE(log.contains(MqttEvent::Type::kConnected));

  EXPECT_EQ(client.publish(R"({"objects": []})"), PublishResult::kQueued);
  const auto publication = broker.waitForPublication(std::chrono::seconds(5));
  ASSERT_TRUE(publication);
  EXPECT_EQ(publication->topic, "ufil/test");
  EXPECT_EQ(publication->payload, R"({"objects": []})");
  ASSERT_TRUE(waitFor([&] {return client.status().sent == 1;}));
  EXPECT_EQ(client.status().failed, 0u);
  EXPECT_FALSE(client.connect()) << "already connected";
}

TEST(MqttClient, ReportsLostConnection)
{
  auto broker = std::make_unique<ufil_mqtt::test::FakeMqttBroker>();
  EventLog log;
  MqttClient client(localConfig(broker->port()), log.callback());
  client.connect();
  ASSERT_TRUE(waitFor([&] {return client.status().state == ConnectionState::kConnected;}));

  broker.reset();
  ASSERT_TRUE(waitFor([&] {return log.contains(MqttEvent::Type::kConnectionLost);}));
  EXPECT_EQ(client.status().state, ConnectionState::kDisconnected);
  EXPECT_EQ(client.status().connection_losses, 1u);
  EXPECT_EQ(client.publish("{}"), PublishResult::kNotConnected);
}
