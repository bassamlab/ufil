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

#ifndef UFIL_MQTT__CONFIG_HPP_
#define UFIL_MQTT__CONFIG_HPP_

#include <optional>
#include <string>

#include "ufil_mqtt/visibility_control.hpp"

namespace ufil_mqtt
{

/// Connection to the MQTT broker and the MQTT topic the messages are published on.
struct MqttConfig
{
  /// Transport: "tcp" (plain) or "ssl" (TLS, broker certificate checked against the system CAs).
  std::string protocol = "tcp";
  std::string host = "localhost";
  int port = 1883;
  /// Client identifier, must be unique among the clients of the broker.
  std::string client_id = "ufil_mqtt";
  /// MQTT topic the messages are published on, without wildcards.
  std::string topic = "ufil/object_list";
};

/// @return a description of the first invalid value, or std::nullopt if the configuration is valid.
UFIL_MQTT_PUBLIC std::optional<std::string> validate(const MqttConfig & config);

/// Server URI of the broker, e.g. "tcp://localhost:1883".
UFIL_MQTT_PUBLIC std::string serverUri(const MqttConfig & config);

}  // namespace ufil_mqtt

#endif  // UFIL_MQTT__CONFIG_HPP_
