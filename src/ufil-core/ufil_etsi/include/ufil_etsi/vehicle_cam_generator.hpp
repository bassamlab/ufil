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

#ifndef UFIL_ETSI__VEHICLE_CAM_GENERATOR_HPP_
#define UFIL_ETSI__VEHICLE_CAM_GENERATOR_HPP_

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include "ufil_etsi/cam.hpp"
#include "ufil_etsi/types.hpp"
#include "ufil_etsi/visibility_control.hpp"

namespace ufil_etsi
{

/// Configuration of the VehicleCamGenerator.
struct VehicleCamGeneratorConfig
{
  /// UTM zone of the objects passed to the generator.
  UtmZone utm_zone;
  /// DE_VehicleRole (ETSI TS 102 894-2), 0 (default) to 15.
  std::uint8_t vehicle_role = 0;
  /// Added to the measurement time before the generation delta time is computed. Allows to
  /// send CAMs for simulated or recorded data whose clock starts before 2004.
  Duration time_shift{0};
};

/// Returns a description of the first invalid value, std::nullopt if the configuration is valid.
UFIL_ETSI_PUBLIC std::optional<std::string> validate(const VehicleCamGeneratorConfig & config);

/// Reason for the generation of a CAM (ETSI EN 302 637-2, clause 6.1.3).
enum class GenerationTrigger
{
  kNone,
  kFirst,  ///< First CAM.
  kDynamics,  ///< Condition 1: heading, position or speed changed.
  kTimeout,  ///< Condition 2: T_GenCam elapsed.
};

/// Outcome of VehicleCamGenerator::check().
enum class GenerationEvent
{
  kGenerated,
  kNoObject,  ///< No new object since the last CAM.
  kNotDue,  ///< No trigger condition is fulfilled.
  kInvalidTime,  ///< The shifted measurement time is before 2004; the object is dropped.
  kInvalidPosition,  ///< The object position is not a valid UTM position; it is dropped.
};

struct GenerationResult
{
  GenerationEvent event = GenerationEvent::kNoObject;
  GenerationTrigger trigger = GenerationTrigger::kNone;
  /// Set if event is kGenerated.
  std::optional<Cam> cam;
};

/// Counters and state of the generator.
struct VehicleCamGeneratorStatus
{
  std::uint64_t objects_received = 0;
  std::uint64_t cams_generated = 0;
  std::uint64_t triggered_by_dynamics = 0;
  std::uint64_t triggered_by_timeout = 0;
  std::uint64_t low_frequency_containers = 0;
  std::uint64_t invalid_time = 0;
  std::uint64_t invalid_position = 0;
  /// Current upper limit of the generation interval (T_GenCam).
  Duration generation_interval{0};
  /// Clock time of the last object and the last CAM, if any.
  std::optional<Time> last_object_time;
  std::optional<Time> last_generation_time;
  std::size_t path_history_size = 0;
};

/**
 * @brief Generates the CAMs of a vehicle according to ETSI EN 302 637-2 V1.4.1, clause 6.1.3.
 *
 * The latest object of the vehicle is passed with setObject(); check() has to be called every
 * kCheckInterval (T_CheckCamGen) and returns a CAM if the generation conditions are fulfilled:
 * the first object, a change of the heading by more than 4 deg, of the position by more than
 * 4 m or of the speed by more than 0.5 m/s since the last CAM (at least T_GenCamMin after it),
 * or T_GenCam since the last CAM. Channel congestion (T_GenCam_DCC) is not considered. Every
 * object is used for at most one CAM.
 *
 * Every kLowFrequencyInterval the low-frequency container with the path history is added. The
 * path history consists of the reference positions of the previous CAMs, each path point
 * relative to the preceding (younger) one.
 */
class UFIL_ETSI_PUBLIC VehicleCamGenerator
{
public:
  static constexpr Duration kGenCamMin = std::chrono::milliseconds(100);
  static constexpr Duration kGenCamMax = std::chrono::milliseconds(1000);
  /// T_CheckCamGen, at most T_GenCamMin.
  static constexpr Duration kCheckInterval = std::chrono::milliseconds(50);
  /// N_GenCam: consecutive condition 2 CAMs after which T_GenCam is reset to T_GenCamMax.
  static constexpr int kNGenCam = 3;
  static constexpr Duration kLowFrequencyInterval = std::chrono::milliseconds(500);

  /// @throws std::invalid_argument if the configuration is invalid.
  explicit VehicleCamGenerator(VehicleCamGeneratorConfig config);

  /// Sets the latest object of the vehicle, measured at measurement_time, received at now.
  void setObject(const Object & object, Time measurement_time, Time now);

  /// Discards the latest object, e.g. because the input became invalid.
  void clearObject();

  /// Checks the generation conditions at the clock time now.
  GenerationResult check(Time now);

  const VehicleCamGeneratorConfig & config() const {return config_;}
  VehicleCamGeneratorStatus status() const;

private:
  struct PathEntry
  {
    std::int32_t latitude;
    std::int32_t longitude;
    Time time;
  };

  bool dynamicsChanged(const Object & object) const;
  std::vector<CamPathPoint> pathHistory(const CamReferencePosition & reference, Time time) const;
  void addToPathHistory(const CamReferencePosition & reference, Time time);

  VehicleCamGeneratorConfig config_;
  VehicleCamGeneratorStatus status_;

  std::optional<Object> object_;
  Time measurement_time_{0};

  // State of the last CAM, required for condition 1.
  std::optional<Object> last_cam_object_;
  std::optional<Time> last_low_frequency_time_;
  int consecutive_timeouts_ = 0;

  /// Reference positions of the previous CAMs, the youngest first.
  std::deque<PathEntry> path_history_;
};

}  // namespace ufil_etsi

#endif  // UFIL_ETSI__VEHICLE_CAM_GENERATOR_HPP_
