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

#include <string>

#include "ufil_mqtt/config.hpp"

using ufil_mqtt::MqttConfig;
using ufil_mqtt::validate;

TEST(Config, DefaultsAreValid)
{
  EXPECT_FALSE(validate(MqttConfig{}));
}

TEST(Config, ServerUri)
{
  MqttConfig config;
  config.protocol = "ssl";
  config.host = "broker.example.org";
  config.port = 8883;
  EXPECT_EQ(ufil_mqtt::serverUri(config), "ssl://broker.example.org:8883");
}

TEST(Config, RejectsInvalidValues)
{
  const auto invalid = [](auto modify) {
      MqttConfig config;
      modify(config);
      return validate(config).has_value();
    };
  EXPECT_TRUE(invalid([](MqttConfig & c) {c.protocol = "udp";}));
  EXPECT_TRUE(invalid([](MqttConfig & c) {c.protocol = "";}));
  EXPECT_TRUE(invalid([](MqttConfig & c) {c.host = "";}));
  EXPECT_TRUE(invalid([](MqttConfig & c) {c.host = "tcp://localhost";}));
  EXPECT_TRUE(invalid([](MqttConfig & c) {c.port = 0;}));
  EXPECT_TRUE(invalid([](MqttConfig & c) {c.port = 65536;}));
  EXPECT_TRUE(invalid([](MqttConfig & c) {c.client_id = "";}));
  EXPECT_TRUE(invalid([](MqttConfig & c) {c.topic = "";}));
  EXPECT_TRUE(invalid([](MqttConfig & c) {c.topic = "ufil/+";}));
  EXPECT_TRUE(invalid([](MqttConfig & c) {c.topic = "ufil/#";}));

  EXPECT_FALSE(invalid([](MqttConfig & c) {c.protocol = "ssl";}));
  EXPECT_FALSE(invalid([](MqttConfig & c) {c.host = "192.168.1.77";}));
  EXPECT_FALSE(invalid([](MqttConfig & c) {c.port = 65535;}));
}
