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


/**
 * @file ssl_single_virtual.cpp
 * @author Marius Molz (marius.molz@rwth-aachen.de)
 * @brief Driver implementation, connects to virtual (simulated) mat
 * @version 1.0
 * @date 2024-01-12
 *
 */
#include "ssl_single_virtual.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <utility>
#include <rclcpp/clock.hpp>

#include "driver_interface.hpp"
#include "layer.hpp"

namespace cppdriver::impl
{
SingleSSLVirtual::~SingleSSLVirtual()
{
  this->stop_thread_ = true;
  if (this->is_threaded_) {
    this->thread_.join();
  }
}

void
SingleSSLVirtual::threadLoop()
{
  auto start = std::chrono::high_resolution_clock::now();
  uint64_t loop_count = 0;
  while (!this->stop_thread_) {
    try {
                // Update settings
      this->updateLineFilter();
      this->updateNoiseFilter();
      this->updateAutoZero();
      this->updateVoltageSharedCompensation();
      this->updateActivePointThreshold();
      this->updateActivePointMode();
      this->updateADCResolution();
      this->commandRequest();
    } catch (std::exception & ex) {
                // We continue to thread
      std::cerr << ex.what() << '\n';
      continue;
    }
    loop_count++;
    auto const time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::high_resolution_clock::now() - start)
      .count();
    if (time_elapsed > 1000U) {
      this->current_hertz = loop_count * 1000.f / time_elapsed;
      loop_count = 0;
      start = std::chrono::high_resolution_clock::now();
    }
  }
}

[[nodiscard]] SingleSSLVirtual::SingleSSLVirtual(
  __attribute__((unused)) uint8_t const index,
  Configuration configuration)
: configuration_(configuration)
{
  std::lock_guard<std::mutex> lock(this->serial_mutex_);
}

[[nodiscard, gnu::const]] auto
SingleSSLVirtual::dimensions() const -> std::pair<dimension_t, dimension_t> const
{
  return std::make_pair(configuration_.getCellsWidth(), configuration_.getCellsHeight());
}

[[nodiscard]] auto
SingleSSLVirtual::mapPtr() -> pressure_map const &
{
  return this->sensels_;
}

void
SingleSSLVirtual::commandStart()
{
  if (this->adc_resolution_cur != ADC_RESOLUTION::LOW) {
    throw std::ios_base::failure("Wrong ADC resolution specified");
  }
  this->stop_thread_ = false;
  this->thread_ = std::thread(&SingleSSLVirtual::threadLoop, this);
  this->is_threaded_ = true;
}

void
SingleSSLVirtual::commandStop()
{
  this->stop_thread_ = true;
  if (this->is_threaded_) {
    this->thread_.join();
  }
  this->is_threaded_ = false;
}

void
SingleSSLVirtual::commandRequest()
{
  //  Use ROS right-hand coordinate system
  // Update vehicle models

  pressure_map points;

  std::this_thread::sleep_for(std::chrono::milliseconds(23));
  std::lock_guard lock(this->map_mutex);

  last_update = ros_clock_->now();
  points = layer::emulateRandomNoise(
                  configuration_.getCellsWidth(),
                  configuration_.getCellsHeight(),
                  layer::emulateLineCrosstalk(
                          configuration_.getCellsWidth(),
                          configuration_.getCellsHeight(),
                          layer::calculateWheelPoints(configuration_, last_update, object_map_)));

  pressure_map points_cleared{};

  // Apply thresholding without explicit full copy
  std::copy_if(points.begin(),
                     points.end(),
                     std::inserter(points_cleared, points_cleared.end()),
    [this](auto const & kv_pair) {return kv_pair.second >= thresh_cur;});

  const uint16_t number_of_points = points_cleared.size();

  if (number_of_points == 0U) {
    // Early yielding
    // if (this->sensels_.empty())
    //   return;
    this->sensels_.clear();
    this->map_update_signal.notify_one();
  } else {
    this->sensels_.clear();
    this->sensels_ = points_cleared;
    this->map_update_signal.notify_one();
  }
}

[[nodiscard]] auto
SingleSSLVirtual::commandFirmware() -> std::string
{
  std::lock_guard<std::mutex> lock(this->serial_mutex_);
  return "VIRTUAL_MAT_CPM";
}

void
SingleSSLVirtual::commandLineFilter(LINE_FILTER lf)
{
  this->line_filter_req = lf;
  if (!this->is_threaded_) {
    updateLineFilter();
  }
}

void
SingleSSLVirtual::updateLineFilter()
{
  if (this->line_filter_cur == this->line_filter_req) {
    return;
  }
  std::lock_guard<std::mutex> lock(this->serial_mutex_);
  this->line_filter_cur = this->line_filter_req;
}

[[nodiscard]] auto
SingleSSLVirtual::commandLineFilter() -> LINE_FILTER
{
  return this->line_filter_cur;
}

void
SingleSSLVirtual::commandNoiseFilter(NOISE_FILTER nf)
{
  this->noise_filter_req = nf;
  if (!this->is_threaded_) {
    updateNoiseFilter();
  }
}

void
SingleSSLVirtual::updateNoiseFilter()
{
  if (this->noise_filter_cur == this->noise_filter_req) {
    return;
  }
  std::lock_guard<std::mutex> lock(this->serial_mutex_);
  this->noise_filter_cur = this->noise_filter_req;
}

[[nodiscard]] auto
SingleSSLVirtual::commandNoiseFilter() -> NOISE_FILTER
{
  return this->noise_filter_cur;
}

void
SingleSSLVirtual::commandAutoZero(AUTO_ZERO az)
{
  this->auto_zero_req = az;
  if (!this->is_threaded_) {
    updateAutoZero();
  }
}

void
SingleSSLVirtual::updateAutoZero()
{
  if (this->auto_zero_cur == this->auto_zero_req) {
    return;
  }
  std::lock_guard<std::mutex> lock(this->serial_mutex_);
  this->auto_zero_cur = this->auto_zero_req;
}

[[nodiscard]] auto
SingleSSLVirtual::commandAutoZero() -> AUTO_ZERO
{
  return this->auto_zero_cur;
}

void
SingleSSLVirtual::commandVoltageSharedCompensation(VOLTAGE_SHARED_COMPENSATION vc)
{
  this->voltage_shared_compensation_req = vc;
  if (!this->is_threaded_) {
    updateVoltageSharedCompensation();
  }
}

void
SingleSSLVirtual::updateVoltageSharedCompensation()
{
  if (this->voltage_shared_compensation_cur == this->voltage_shared_compensation_req) {
    return;
  }
  std::lock_guard<std::mutex> lock(this->serial_mutex_);
  this->voltage_shared_compensation_cur = this->voltage_shared_compensation_req;
}

[[nodiscard]] auto
SingleSSLVirtual::commandVoltageSharedCompensation() -> VOLTAGE_SHARED_COMPENSATION
{
  return this->voltage_shared_compensation_cur;
}

void
SingleSSLVirtual::commandActivePointThreshold(uint16_t threshold)
{
  this->thresh_req = threshold;
  if (!this->is_threaded_) {
    updateActivePointThreshold();
  }
}

void
SingleSSLVirtual::updateActivePointThreshold()
{
  if (this->thresh_cur == this->thresh_req) {
    return;
  }
  if (this->thresh_req == 0U || this->thresh_req > 4096U) {
    throw std::domain_error("Desired digital level not in valid range [1,4096]");
  }

  std::lock_guard<std::mutex> lock(this->serial_mutex_);
  this->thresh_cur = thresh_req;
}

[[nodiscard]] auto
SingleSSLVirtual::commandActivePointThreshold() -> uint16_t
{
  return this->thresh_cur;
}

void
SingleSSLVirtual::commandActivePointMode(ACTIVE_POINTS_MODE mode)
{
  this->active_points_mode_req = mode;
  if (!this->is_threaded_) {
    updateActivePointMode();
  }
}

void
SingleSSLVirtual::updateActivePointMode()
{
  if (this->active_points_mode_cur == this->active_points_mode_req) {
    return;
  }
  std::lock_guard<std::mutex> lock(this->serial_mutex_);
  this->active_points_mode_cur = this->active_points_mode_req;
  this->ros_clock_->sleep_for(std::chrono::milliseconds(100));
}

[[nodiscard]] auto
SingleSSLVirtual::commandActivePointMode() -> ACTIVE_POINTS_MODE
{
  return this->active_points_mode_cur;
}

void
SingleSSLVirtual::commandADCResolution(ADC_RESOLUTION resolution)
{
  this->adc_resolution_req = resolution;
  if (!this->is_threaded_) {
    updateADCResolution();
  }
}

void
SingleSSLVirtual::updateADCResolution()
{
  if (this->adc_resolution_cur == this->adc_resolution_req) {
    return;
  }
  std::lock_guard<std::mutex> lock(this->serial_mutex_);
  this->adc_resolution_cur = this->adc_resolution_req;
}

[[nodiscard]] auto
SingleSSLVirtual::commandADCResolution() -> ADC_RESOLUTION
{
  return this->adc_resolution_cur;
}

[[nodiscard]] auto
SingleSSLVirtual::settingsUpdatePending() const -> bool
{
  return this->line_filter_cur != this->line_filter_req ||
         this->noise_filter_cur != this->noise_filter_req ||
         this->auto_zero_cur != this->auto_zero_req ||
         this->voltage_shared_compensation_cur != this->voltage_shared_compensation_req ||
         this->active_points_mode_cur != this->active_points_mode_req ||
         this->thresh_cur != this->thresh_req ||
         this->adc_resolution_cur != this->adc_resolution_req;
}

[[nodiscard]] auto
SingleSSLVirtual::lastUpdate() const -> rclcpp::Time
{
  return this->last_update;
}

[[nodiscard]] auto
SingleSSLVirtual::currentHertz() const -> uint8_t
{
  return this->current_hertz;
}

void
SingleSSLVirtual::registerClock(std::shared_ptr<rclcpp::Clock> const & clock_ptr)
{
  ros_clock_ = clock_ptr;
}
}  // namespace cppdriver::impl
