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

#include "ufil_mqtt/config.hpp"

#include <optional>
#include <string>

namespace ufil_mqtt
{

std::optional<std::string> validate(const MqttConfig & config)
{
  if (config.protocol != "tcp" && config.protocol != "ssl") {
    return "protocol must be 'tcp' or 'ssl', got '" + config.protocol + "'";
  }
  if (config.host.empty()) {
    return std::string("host must be set");
  }
  if (config.host.find_first_of(" /:") != std::string::npos) {
    return "host must be a host name or IPv4 address, got '" + config.host + "'";
  }
  if (config.port < 1 || config.port > 65535) {
    return "port must be in [1, 65535], got " + std::to_string(config.port);
  }
  if (config.client_id.empty()) {
    return std::string("client id must be set");
  }
  if (config.topic.empty()) {
    return std::string("topic must be set");
  }
  if (config.topic.find_first_of("+#") != std::string::npos) {
    return "topic must not contain the wildcards '+' or '#', got '" + config.topic + "'";
  }
  return std::nullopt;
}

std::string serverUri(const MqttConfig & config)
{
  return config.protocol + "://" + config.host + ":" + std::to_string(config.port);
}

}  // namespace ufil_mqtt
