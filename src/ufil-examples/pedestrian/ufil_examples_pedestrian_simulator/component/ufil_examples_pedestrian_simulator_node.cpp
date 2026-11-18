// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
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

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include "ufil_examples_pedestrian_simulator/pedestrian_simulator.hpp"
#include "ufil_examples_pedestrian_simulator/scenario.hpp"
#include "SimulatorParameterRos.hpp"
#include "simulator_ros.hpp"

namespace ufil_examples_pedestrian_simulator
{

/**
 * @brief ROS 2 component simulating pedestrians of a scenario file.
 *
 * Publishes every timestep until the scenario ends:
 *   measured_objects  (ufil_msgs/ObjectList) noisy detections including clutter
 *   reference_objects (ufil_msgs/ObjectList) ground truth of the agents in the scene
 *   /diagnostics
 * Afterwards the node stays idle.
 */
class PedestrianSimulatorNode : public rclcpp::Node
{
public:
  explicit PedestrianSimulatorNode(const rclcpp::NodeOptions & options)
  : Node("pedestrian_simulator", options),
    parameters_(loadParameters(*this)),
    simulator_(loadScenario(*this, parameters_.settings())),
    diagnostics_(this)
  {
    const auto & settings = parameters_.settings();
    measured_publisher_ = create_publisher<ufil_msgs::msg::ObjectList>("measured_objects", 10);
    reference_publisher_ = create_publisher<ufil_msgs::msg::ObjectList>("reference_objects", 10);
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(settings.timestep), [this] {onTimer();});

    diagnostics_.setHardwareID("pedestrian_simulator");
    diagnostics_.add("Pedestrian simulator", this, &PedestrianSimulatorNode::produceDiagnostics);

    const auto & scenario = simulator_.scenario();
    RCLCPP_INFO(
      get_logger(), "Simulating scenario '%s' with %zu agents for %.1f s, timestep %.3f s.",
      scenario.name.c_str(), scenario.agents.size(), scenario.runtime, settings.timestep);
  }

private:
  /// Logs invalid parameters before construction fails, so the reason ends up in the ROS log.
  static SimulatorParameterRos loadParameters(rclcpp::Node & node)
  {
    try {
      return SimulatorParameterRos(node.get_node_parameters_interface());
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(node.get_logger(), "%s", error.what());
      throw;
    }
  }

  static PedestrianSimulator loadScenario(rclcpp::Node & node, const SimulatorSettings & settings)
  {
    try {
      std::string scenario_directory;
      if (!std::filesystem::path(settings.scenario_file).is_absolute()) {
        scenario_directory =
          ament_index_cpp::get_package_share_directory("ufil_examples_pedestrian_simulator") +
          "/config";
      }
      const auto path = resolveScenarioFile(settings.scenario_file, scenario_directory);
      return PedestrianSimulator(loadScenarioFile(path), settings.timestep);
    } catch (const std::exception & error) {
      RCLCPP_FATAL(node.get_logger(), "Cannot load the scenario: %s", error.what());
      throw;
    }
  }

  void onTimer()
  {
    const auto step = simulator_.step();
    if (!step) {
      RCLCPP_INFO(
        get_logger(), "Scenario '%s' finished after %.1f s.",
        simulator_.status().scenario.c_str(), simulator_.status().time);
      timer_->cancel();
      return;
    }
    RCLCPP_DEBUG(
      get_logger(), "Step at %.2f s: %zu agents in the scene, %zu detected, %zu clutter.",
      step->time, simulator_.status().agents_in_scene, simulator_.status().detected_agents,
      simulator_.status().clutter);

    std_msgs::msg::Header header;
    header.frame_id = parameters_.settings().frame_id;
    header.stamp = now();
    measured_publisher_->publish(conversions::measuredObjectsToMsg(*step, header));
    reference_publisher_->publish(conversions::referenceObjectsToMsg(*step, header));
  }

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & diagnostic)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    const SimulatorStatus & status = simulator_.status();
    if (status.finished) {
      diagnostic.summaryf(DiagnosticStatus::OK, "Scenario '%s' finished", status.scenario.c_str());
    } else if (status.steps == 0) {
      diagnostic.summaryf(
        DiagnosticStatus::OK, "Scenario '%s' not started yet", status.scenario.c_str());
    } else {
      diagnostic.summaryf(
        DiagnosticStatus::OK, "Running scenario '%s' (%.1f of %.1f s)", status.scenario.c_str(),
        status.time, status.runtime);
    }
    diagnostic.add("Scenario", status.scenario);
    diagnostic.add("Scenario file", parameters_.settings().scenario_file);
    diagnostic.add("Scenario time [s]", status.time);
    diagnostic.add("Runtime [s]", status.runtime);
    diagnostic.add("Steps", status.steps);
    diagnostic.add("Agents in scene", status.agents_in_scene);
    diagnostic.add("Detected agents", status.detected_agents);
    diagnostic.add("False detections", status.clutter);
  }

  SimulatorParameterRos parameters_;
  PedestrianSimulator simulator_;

  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr measured_publisher_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr reference_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  diagnostic_updater::Updater diagnostics_;
};

}  // namespace ufil_examples_pedestrian_simulator

RCLCPP_COMPONENTS_REGISTER_NODE(ufil_examples_pedestrian_simulator::PedestrianSimulatorNode)
