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


#include <memory>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <ufil_msgs/msg/object_list.hpp>

#include <ufil_examples_pedestrian_simulator/ufil_examples_pedestrian_simulator.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class PedestrianSimulatorNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_measured_;
  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_ground_truth_;

  rclcpp::TimerBase::SharedPtr timer_;
  ufil_examples_pedestrian_simulator::PedestrianSimulator::UniquePtr pedestrian_simulator_;

  void timer_callback() const
  {
    RCLCPP_INFO(this->get_logger(), "Stepping simulation");

    std::map<int64_t, ufil_examples_pedestrian_simulator::ScenarioState> ids_with_states;
    bool valid = this->pedestrian_simulator_->step(ids_with_states);
    if (!valid) {
      RCLCPP_INFO(this->get_logger(), "Simulation finished.");
      this->timer_->cancel();
      rclcpp::shutdown();
    }
    ufil_msgs::msg::ObjectList object_list;
    object_list.header.frame_id = "map";
    object_list.header.stamp = this->now();

    for (const auto & [id, state] : ids_with_states) {
      ufil_msgs::msg::Object object;
      object.id = id;
      object.state.state.x = state.measured.position.x();
      object.state.state.y = state.measured.position.y();
      object.state.state.yaw = state.measured.orientation;
      object.state.covariance = {};
      object.state.covariance[0] = 1;
      object.state.covariance[9] = 1;
      object.state.covariance[54] = 0.1;

      // Indicate not measured
      object.state.covariance[18] = -1;
      object.state.covariance[27] = -1;
      object.state.covariance[36] = -1;
      object.state.covariance[45] = -1;
      object.state.covariance[63] = -1;

      object.dimension.dimension.length = state.measured.dimension.x();
      object.dimension.dimension.width = state.measured.dimension.y();
      object.dimension.dimension.height = state.measured.dimension.z();
      object.dimension.covariance[0] = 1;
      object.dimension.covariance[4] = 1;
      object.dimension.covariance[8] = 1;

      object.classification.classification[ufil_msgs::msg::Classification::PEDESTRIAN] = 1.0;
      object_list.objects.push_back(object);
    }
    this->publisher_measured_->publish(object_list);

    object_list.objects.clear();

    for (const auto & [id, state] : ids_with_states) {
      ufil_msgs::msg::Object object;
      object.id = id;
      object.state.state.x = state.ground_truth.position.x();
      object.state.state.y = state.ground_truth.position.y();
      object.state.state.yaw = state.ground_truth.orientation;
      object.dimension.dimension.length = state.ground_truth.dimension.x();
      object.dimension.dimension.width = state.ground_truth.dimension.y();
      object.dimension.dimension.height = state.ground_truth.dimension.z();

      object.classification.classification[ufil_msgs::msg::Classification::PEDESTRIAN] = 1.0;
      object_list.objects.push_back(object);
    }
    this->publisher_ground_truth_->publish(object_list);
  }

public:
  PedestrianSimulatorNode()
  : Node("ufil_examples_pedestrian_simulator")
  {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("ufil_examples_pedestrian_simulator");
    std::string path = package_share_directory + "/config/crossing.yaml";
    this->declare_parameter("scenario_file", path);

    std::string scenario_file = this->get_parameter("scenario_file").as_string();
    const std::filesystem::path scenario_file_path{scenario_file};

    int simulation_timestep = 100;
    this->declare_parameter("simulation_timestep", simulation_timestep);
    simulation_timestep = this->get_parameter("simulation_timestep").as_int();
    double timestep_in_seconds = static_cast<double>(simulation_timestep * 1e-3);

    this->pedestrian_simulator_ =
      std::make_unique<ufil_examples_pedestrian_simulator::PedestrianSimulator>();

    RCLCPP_INFO(this->get_logger(), "Loading scenario file %s", scenario_file.c_str());
    if (std::filesystem::exists(scenario_file_path)) {
      this->pedestrian_simulator_->loadScenarioFile(scenario_file);
      RCLCPP_INFO(this->get_logger(), "Loading scenario file successful.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Loading scenario file failed. File not found.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Generating trajectories with timestep %fs",
      timestep_in_seconds);
    this->pedestrian_simulator_->generateTrajectories(timestep_in_seconds);

    RCLCPP_INFO(this->get_logger(), "Connect to ROS2 network.");
    this->publisher_measured_ =
      this->create_publisher<ufil_msgs::msg::ObjectList>("measured_objects", 10);
    this->publisher_ground_truth_ =
      this->create_publisher<ufil_msgs::msg::ObjectList>("reference_objects", 10);

    this->timer_ = rclcpp::create_timer(this, this->get_clock(),
      std::chrono::milliseconds(simulation_timestep),
                                        std::bind(&PedestrianSimulatorNode::timer_callback, this));
  }

  bool initialized() const
  {
    if (!this->pedestrian_simulator_) {
      return false;
    }
    return this->pedestrian_simulator_->initialized();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PedestrianSimulatorNode>();
  if (node->initialized()) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
