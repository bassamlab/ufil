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
 * @file ssl_single_virtual.hpp
 * @author Marius Molz (marius.molz@rwth-aachen.de)
 * @brief Driver implementation, connects to virtual (simulated) mat
 * @version 1.0
 * @date 2024-01-12
 *
 */
#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/time.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "configuration.hpp"
#include "driver_interface.hpp"
#include "vehicle_model_interface.hpp"

/// @brief Namespace containing all actual implementations of the driver interface \link
/// cppdriver::IDriver \endlink
namespace cppdriver::impl
{
using configuration::Configuration;
    /// @brief Actual implementation of the virtual driver interface.
class SingleSSLVirtual : public IDriver {
public:
        /// @brief Construct a SingleSSL instance of the virtual driver interface
        /// @param devicePath POSIX device path to the USB hardware connector
  [[nodiscard]] SingleSSLVirtual(uint8_t index, Configuration configuration);

  ~SingleSSLVirtual() override;

        /// @brief Request the streaming of maps, updating as fast as possible
  void commandStart() override;

        /// @brief Abort the streaming of maps
  void commandStop() override;

        /// @brief Request one single map update
  void commandRequest() override;

        /// @brief Request firmware version from mat interface
        /// @return std::string Firmware version
  [[nodiscard]] auto commandFirmware() -> std::string override;

        /// @brief Request configuration change of the line filter
        /// @param  LINE_FILTER to be applied
  void commandLineFilter(LINE_FILTER) override;

        /// @brief Retrieve currently applied line filter
        /// @return current line filter
  [[nodiscard]] auto commandLineFilter() -> LINE_FILTER override;

        /// @brief Request configuration change of the noise filter level
        /// @param  NOISE_FILTER to be applied
  void commandNoiseFilter(NOISE_FILTER) override;

        /// @brief Retrieve currently applied noise filter level
        /// @return current noise filter level
  [[nodiscard]] auto commandNoiseFilter() -> NOISE_FILTER override;

        /// @brief Request configuration change of automatic zeroing
        /// @param  AUTO_ZERO to be applied
  void commandAutoZero(AUTO_ZERO) override;

        /// @brief Retrieve currently applied automatic zeroing setting
        /// @return current automatic zeroing configuration
  [[nodiscard]] auto commandAutoZero() -> AUTO_ZERO override;

        /// @brief Request configuration change of the voltage shared compensation
        /// @param  VOLTAGE_SHARED_COMPENSATION to be applied
  void commandVoltageSharedCompensation(VOLTAGE_SHARED_COMPENSATION) override;

        /// @brief Retrieve currently applied voltage shared compensation
        /// @return current voltage shared compensation
  [[nodiscard]] auto commandVoltageSharedCompensation() -> VOLTAGE_SHARED_COMPENSATION override;

        /// @brief Request configuration change of the active point threshold
        /// @param  uint16_t threshold to be applied
  void commandActivePointThreshold(uint16_t) override;

        /// @brief Retrieve currently applied active point mode threshold
        /// @return current threshold
  [[nodiscard]] auto commandActivePointThreshold() -> uint16_t override;

        /// @brief Request configuration change of the active point mode
        /// @param  ACTIVE_POINTS_MODE to be applied
  void commandActivePointMode(ACTIVE_POINTS_MODE) override;

        /// @brief Retrieve currently applied active point mode
        /// @return current active point mode
  [[nodiscard]] auto commandActivePointMode() -> ACTIVE_POINTS_MODE override;

        /// @brief Request configuration change of the ADC resolution
        /// @param ADC_RESOLUTION to be applied
  void commandADCResolution(ADC_RESOLUTION) override;

        /// @brief Retrieve currently applied ADC resolution
        /// @return current ADC resolution
  [[nodiscard]] auto commandADCResolution() -> ADC_RESOLUTION override;

        /// @brief Retrieve dimensions of the map structure
        /// @return (dimension_x, dimension_y) such that \f$X \in
        /// \{0,\dots,\text{dimension_x}-1\}, Y \in \{0,\dots,\text{dimension_y}-1\}\f$
        /// for all points \f$(X,Y)\f$ of the map structure
  [[nodiscard, gnu::const]] auto dimensions() const
  -> std::pair<dimension_t, dimension_t> const override;

        /// @brief Return access to map structure
        /// @return pointer to map, since we cannot prevent pointer decay
  [[nodiscard]] auto mapPtr() -> pressure_map const & override;

        /// @brief Check if there are pending configuration updates
        /// @return true if configuration changes are not applied yet
  [[nodiscard]] auto settingsUpdatePending() const -> bool override;

        /// @brief Retrieve current rate of successful map updates
        /// @return update frequency in hertz
  [[nodiscard]] auto currentHertz() const -> uint8_t override;

        /// @brief Retrieve last refresh time
        /// @return timestamp
  [[nodiscard]] auto lastUpdate() const -> rclcpp::Time override;

        /// @brief shared ptr to vehicle information used for rule-based pressure generation
  std::map<ufil_msgs::msg::Object::_id_type, std::shared_ptr<vehicle::IVehicleModel>> object_map_;

  void registerClock(std::shared_ptr<rclcpp::Clock> const & clock_ptr);

  std::mutex clock_used{};

private:
        /// @brief Configuration
  Configuration configuration_;

        /// @brief mutex that protects against concurrent access to the serial port
  std::mutex serial_mutex_{};
        /// @brief true iff the driver is started in threading mode to activate map streaming
  bool is_threaded_ = false;
        /// @brief internal map matrix with dimensions \link
        /// configuration::SIZE_X \endlink \f$\times\f$
        /// \link configuration::SIZE_Y \endlink
        /// @note stored in row-major order, required by the ROS2 OccupancyGrid
  pressure_map sensels_{};
        /// @brief thread object to be dispatched or halted
  std::thread thread_;

        /// @brief method which is dispatched as thread
  void threadLoop();

        /// @brief internal signal which halts the threading of map requests
  std::atomic<bool> stop_thread_{false};
  std::shared_ptr<rclcpp::Clock> ros_clock_;

        /// @brief Apply configuration change request for the line filter
  void updateLineFilter();

        /// @brief Apply configuration change request for the noise filter level
  void updateNoiseFilter();

        /// @brief Apply configuration change request for the automatic zeroing
  void updateAutoZero();

        /// @brief Apply configuration change request for the voltage shared compensation
  void updateVoltageSharedCompensation();

        /// @brief Apply configuration change request for active point threshold
  void updateActivePointThreshold();

        /// @brief Apply configuration change request for the map transmission mode
  void updateActivePointMode();

        /// @brief Apply configuration change request for the adc resolution
  void updateADCResolution();
};
}  // namespace cppdriver::impl
