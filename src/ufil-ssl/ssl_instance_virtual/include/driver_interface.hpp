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
 * @file driver_interface.hpp
 * @author Marius Molz (marius.molz@rwth-aachen.de)
 * @brief Generic mat driver interface
 * @version 1.0
 * @date 2024-01-12
 *
 */
#pragma once

#include <condition_variable>
#include <cstdint>
#include <map>
#include <string>
#include <utility>

#include <rclcpp/time.hpp>

/// @brief Namespace containing the complete c++-based driver framework
namespace cppdriver
{

    /// @brief dimension type, limits possible mat dimensions
using dimension_t = uint16_t;

    /// @brief Store pressure values in coordinate map
using pressure_map = std::map<std::pair<dimension_t, dimension_t>, uint8_t>;

    /// @brief Possible noise filter configuration options. The level \f$i\f$
    /// decays to the window size \f$2^{1+i}\f$ applied in hardware
enum class NOISE_FILTER : uint8_t
{
  AVG_2 = 2U,         ///< level 0 -> window  size 2
  AVG_4 = 4U,         ///< level 1 -> window  size 4
  AVG_8 = 8U,         ///< level 2 -> window  size 8
  AVG_16 = 16U,       ///< level 3 -> window  size 16
  AVG_32 = 32U        ///< level 4 -> window  size 32
};

    /// @brief Toggle line filtering. The line filter shall reduce linear
    /// ghosting in hardware across a sensel line
    /// @note Negative influence on the performance could not be detected during measurements
enum class LINE_FILTER : bool
{
  DISABLED = false,       ///< disable line filtering
  ENABLED = true          ///< enable line filtering (recommended)
};

    /// @brief Toggle automatic zeroing in hardware
    /// @attention This "feature" instructs the hardware to only send non-zero
    /// data if a pressure change is detected, actively resetting the map as a
    /// result if no movement / pressure change is detected
enum class AUTO_ZERO : bool
{
  DISABLED = false,       ///< disable automatic zeroing
  ENABLED = true          ///< enable automatic zeroing
};

    /// @brief Toggle voltage shared compensation. The compensation shall reduce
    /// ghosting across both dimensions of the sensor matrix
    /// @note Negative influence on the performance could not be detected during measurements
enum class VOLTAGE_SHARED_COMPENSATION : bool
{
  DISABLED = false,       ///< disable voltage shared compensation
  ENABLED = true          ///< enable voltage shared compensation (recommended)
};

    /// @brief Toggle mode of map transmission. Either the complete matrix is transmitted,
    ///, or only active points ( points with a pressure value above the configured \link
    /// IDriver::thresh_cur \endlink ) are transmitted.
enum class ACTIVE_POINTS_MODE : bool
{
  COMPLETE_MATRIX = false,        ///< transfer complete maps
  ONLY_ACTIVE_POINTS = true       ///< only transfer active points
};

    /// @brief Toggle map resolution
enum class ADC_RESOLUTION : bool
{
  LOW = false,        ///< 8 bit resolution, measured pressure mapped from 0 to 255
  MEDIUM = true       ///< 12 bit resolution, measured pressure mapped from 0 to 4095
};

    /// @brief Virtual driver interface. Every sensor mat implementation must implement the virtual
    /// methods and signal via the public mutex and conditional_variable
class IDriver {
public:
  virtual ~IDriver() = default;

        /// @brief Request the streaming of maps, updating as fast as possible
  virtual void commandStart() = 0;

        /// @brief Abort the streaming of maps
  virtual void commandStop() = 0;

        /// @brief Request one single map update
  virtual void commandRequest() = 0;

        /// @brief Request firmware version from mat interface
        /// @return std::string Firmware version
  [[nodiscard]] virtual auto commandFirmware() -> std::string = 0;

        /// @brief Request configuration change of the line filter
        /// @param  LINE_FILTER to be applied
  virtual void commandLineFilter(LINE_FILTER) = 0;

        /// @brief Retrieve currently applied line filter
        /// @return current line filter
  [[nodiscard]] virtual auto commandLineFilter() -> LINE_FILTER = 0;

        /// @brief Request configuration change of the noise filter level
        /// @param  NOISE_FILTER to be applied
  virtual void commandNoiseFilter(NOISE_FILTER) = 0;

        /// @brief Retrieve currently applied noise filter level
        /// @return current noise filter level
  [[nodiscard]] virtual auto commandNoiseFilter() -> NOISE_FILTER = 0;

        /// @brief Request configuration change of automatic zeroing
        /// @param  AUTO_ZERO to be applied
  virtual void commandAutoZero(AUTO_ZERO) = 0;

        /// @brief Retrieve currently applied automatic zeroing setting
        /// @return current automatic zeroing configuration
  [[nodiscard]] virtual auto commandAutoZero() -> AUTO_ZERO = 0;

        /// @brief Request configuration change of the voltage shared compensation
        /// @param  VOLTAGE_SHARED_COMPENSATION to be applied
  virtual void commandVoltageSharedCompensation(VOLTAGE_SHARED_COMPENSATION) = 0;

        /// @brief Retrieve currently applied voltage shared compensation
        /// @return current voltage shared compensation
  [[nodiscard]] virtual auto commandVoltageSharedCompensation() -> VOLTAGE_SHARED_COMPENSATION = 0;

        /// @brief Request configuration change of the active point threshold
        /// @param  uint16_t threshold to be applied
  virtual void commandActivePointThreshold(uint16_t) = 0;

        /// @brief Retrieve currently applied active point mode threshold
        /// @return current threshold
  [[nodiscard]] virtual auto commandActivePointThreshold() -> uint16_t = 0;

        /// @brief Request configuration change of the active point mode
        /// @param  ACTIVE_POINTS_MODE to be applied
  virtual void commandActivePointMode(ACTIVE_POINTS_MODE) = 0;

        /// @brief Retrieve currently applied active point mode
        /// @return current active point mode
  [[nodiscard]] virtual auto commandActivePointMode() -> ACTIVE_POINTS_MODE = 0;

        /// @brief Request configuration change of the ADC resolution
        /// @param ADC_RESOLUTION to be applied
  virtual void commandADCResolution(ADC_RESOLUTION) = 0;

        /// @brief Retrieve currently applied ADC resolution
        /// @return current ADC resolution
  [[nodiscard]] virtual auto commandADCResolution() -> ADC_RESOLUTION = 0;

        /// @brief Retrieve dimensions of the map structure
        /// @return (dimension_x, dimension_y) such that \f$X \in
        /// \{0,\dots,\text{dimension_x}-1\}, Y \in \{0,\dots,\text{dimension_y}-1\}\f$
        /// for all points \f$(X,Y)\f$ of the map structure
  [[nodiscard, gnu::pure]] virtual auto dimensions() const
  -> std::pair<dimension_t, dimension_t> const = 0;

        /// @brief Return access to map structure
        /// @return pointer to map, since we cannot prevent pointer decay
  [[nodiscard]] virtual auto mapPtr() -> pressure_map const & = 0;

        /// @brief Check if there are pending configuration updates
        /// @return true if configuration changes are not applied yet
  [[nodiscard]] virtual auto settingsUpdatePending() const -> bool = 0;

        /// @brief Retrieve current rate of successful map updates
        /// @return update frequency in hertz
  [[nodiscard]] virtual auto currentHertz() const -> uint8_t = 0;

        /// @brief Retrieve last refresh time
        /// @return timestamp
  [[nodiscard]] virtual auto lastUpdate() const -> rclcpp::Time = 0;

        /// @brief mutex that protects access to the map data
  std::mutex map_mutex{};
        /// @brief The condition_variable signals map updates
  std::condition_variable map_update_signal{};

protected:
        /// @brief Currently applied line filter setting
  LINE_FILTER line_filter_cur{};
        /// @brief (Last) requested line filter setting
  LINE_FILTER line_filter_req{};
        /// @brief Currently applied noise filter
  NOISE_FILTER noise_filter_cur{};
        /// @brief (Last) requested noise filter setting
  NOISE_FILTER noise_filter_req{};
        /// @brief Currently applied setting of auto zeroing
  AUTO_ZERO auto_zero_cur{};
        /// @brief (Last) requested setting of auto zeroing
  AUTO_ZERO auto_zero_req{};
        /// @brief Currently applied setting of voltage shared compensation
  VOLTAGE_SHARED_COMPENSATION voltage_shared_compensation_cur{};
        /// @brief (Last) requested setting of voltage shared compensation
  VOLTAGE_SHARED_COMPENSATION voltage_shared_compensation_req{};
        /// @brief Currently applied active point filtering threshold
  uint16_t thresh_cur{};
        /// @brief (Last) requested active point filtering threshold
  uint16_t thresh_req{};
        /// @brief Currently applied ADC resolution setting
  ADC_RESOLUTION adc_resolution_cur{};
        /// @brief (Last) requested ADC resolution setting
  ADC_RESOLUTION adc_resolution_req{};
        /// @brief Currently applied mode of map transmission
  ACTIVE_POINTS_MODE active_points_mode_cur{};
        /// @brief (Last) requested mde of map transmission
  ACTIVE_POINTS_MODE active_points_mode_req{};
        /// @brief current map update rate of the driver
  uint8_t current_hertz{};
        /// @brief last refresh time
  rclcpp::Time last_update{};
};
}  // namespace cppdriver
