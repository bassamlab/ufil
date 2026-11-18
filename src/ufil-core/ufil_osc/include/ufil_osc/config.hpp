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

#ifndef UFIL_OSC__CONFIG_HPP_
#define UFIL_OSC__CONFIG_HPP_

#include <optional>
#include <string>

#include "ufil_osc/visibility_control.hpp"

namespace ufil_osc
{

/// Receiver of the OSC messages and the OSC address they are sent to.
struct OscConfig
{
  /// Host name or IP address of the receiver.
  std::string host = "127.0.0.1";
  /// UDP port of the receiver.
  int port = 9000;
  /// OSC address pattern of the messages, e.g. "/objects". Must not contain pattern characters.
  std::string address = "/objects";
};

/// @return a description of the first invalid value, or std::nullopt if the configuration is valid.
UFIL_OSC_PUBLIC std::optional<std::string> validate(const OscConfig & config);

}  // namespace ufil_osc

#endif  // UFIL_OSC__CONFIG_HPP_
