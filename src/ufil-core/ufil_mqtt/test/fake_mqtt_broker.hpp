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

#ifndef FAKE_MQTT_BROKER_HPP_
#define FAKE_MQTT_BROKER_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace ufil_mqtt::test
{

/// Port on 127.0.0.1 nobody listens on (bound once and released).
inline int unusedPort()
{
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  socklen_t length = sizeof(address);
  ::bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address));
  ::getsockname(fd, reinterpret_cast<sockaddr *>(&address), &length);
  ::close(fd);
  return ntohs(address.sin_port);
}

/**
 * @brief Minimal MQTT 3.1.1 broker on 127.0.0.1 for tests.
 *
 * Accepts one client at a time, acknowledges CONNECT and PINGREQ and records the topic and payload
 * of every PUBLISH (QoS 0).
 */
class FakeMqttBroker
{
public:
  struct Publication
  {
    std::string topic;
    std::string payload;
  };

  FakeMqttBroker()
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    const int enable = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t length = sizeof(address);
    if (::bind(listen_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 ||
      ::listen(listen_fd_, 1) != 0 ||
      ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&address), &length) != 0)
    {
      throw std::runtime_error("cannot open the fake broker socket");
    }
    port_ = ntohs(address.sin_port);
    thread_ = std::thread([this] {run();});
  }

  ~FakeMqttBroker()
  {
    stop_ = true;
    thread_.join();
    ::close(listen_fd_);
  }

  int port() const {return port_;}

  std::optional<Publication> waitForPublication(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_.wait_for(lock, timeout, [this] {return !publications_.empty();})) {
      return std::nullopt;
    }
    Publication publication = publications_.front();
    publications_.pop_front();
    return publication;
  }

private:
  bool waitReadable(int fd)
  {
    while (!stop_) {
      pollfd entry{fd, POLLIN, 0};
      const int result = ::poll(&entry, 1, 50);
      if (result > 0) {
        return true;
      }
      if (result < 0) {
        return false;
      }
    }
    return false;
  }

  bool readExact(int fd, std::uint8_t * data, std::size_t size)
  {
    std::size_t received = 0;
    while (received < size) {
      if (!waitReadable(fd)) {
        return false;
      }
      const ssize_t result = ::recv(fd, data + received, size - received, 0);
      if (result <= 0) {
        return false;
      }
      received += static_cast<std::size_t>(result);
    }
    return true;
  }

  void sendBytes(int fd, std::vector<std::uint8_t> bytes)
  {
    (void)::send(fd, bytes.data(), bytes.size(), MSG_NOSIGNAL);
  }

  void serve(int fd)
  {
    while (true) {
      std::uint8_t header = 0;
      if (!readExact(fd, &header, 1)) {
        return;
      }
      std::size_t length = 0;
      std::size_t multiplier = 1;
      std::uint8_t byte = 0;
      do {
        if (!readExact(fd, &byte, 1)) {
          return;
        }
        length += (byte & 0x7F) * multiplier;
        multiplier *= 128;
      } while (byte & 0x80);
      std::vector<std::uint8_t> body(length);
      if (length > 0 && !readExact(fd, body.data(), length)) {
        return;
      }

      switch (header >> 4) {
        case 1:  // CONNECT -> CONNACK, accepted
          sendBytes(fd, {0x20, 0x02, 0x00, 0x00});
          break;
        case 3: {  // PUBLISH
            const std::size_t topic_length = (body[0] << 8) | body[1];
            std::size_t offset = 2 + topic_length;
            if (((header >> 1) & 0x03) > 0) {
              offset += 2;  // Packet identifier
            }
            Publication publication{
              std::string(body.begin() + 2, body.begin() + 2 + topic_length),
              std::string(body.begin() + offset, body.end())};
            {
              std::lock_guard<std::mutex> lock(mutex_);
              publications_.push_back(publication);
            }
            condition_.notify_all();
            break;
          }
        case 12:  // PINGREQ -> PINGRESP
          sendBytes(fd, {0xD0, 0x00});
          break;
        case 14:  // DISCONNECT
          return;
        default:
          break;
      }
    }
  }

  void run()
  {
    while (waitReadable(listen_fd_)) {
      const int fd = ::accept(listen_fd_, nullptr, nullptr);
      if (fd >= 0) {
        serve(fd);
        ::close(fd);
      }
    }
  }

  int listen_fd_ = -1;
  int port_ = 0;
  std::atomic<bool> stop_{false};
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Publication> publications_;
  std::thread thread_;
};

}  // namespace ufil_mqtt::test

#endif  // FAKE_MQTT_BROKER_HPP_
