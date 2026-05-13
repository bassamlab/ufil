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


#include <random>

#include <rclcpp/rclcpp.hpp>

#include <etsi_its_cam_msgs/msg/cam.hpp>
#include <etsi_its_msgs_utils/cam_access.hpp>

using std::chrono_literals::operator""s;
using etsi_its_cam_msgs::msg::CAM;
using etsi_its_cam_msgs::msg::StationID;
using etsi_its_cam_msgs::msg::ItsPduHeader;
using etsi_its_cam_msgs::msg::StationType;
using etsi_its_cam_msgs::msg::HeadingValue;
using etsi_its_cam_msgs::msg::SemiAxisLength;
using etsi_its_cam_msgs::msg::AltitudeConfidence;
using etsi_its_cam_msgs::msg::HighFrequencyContainer;

class RsuCamGeneratorNode : public rclcpp::Node {
private:
    // CAM
  CAM cam_message_;

    // Publishers
  rclcpp::Publisher<CAM>::SharedPtr cam_publisher_;

    // Timers
  rclcpp::TimerBase::SharedPtr cam_gen_timer_;

  void generateCam()
  {
        // We only need to update the generation timestamp
    etsi_its_cam_msgs::access::setGenerationDeltaTime(this->cam_message_,
      this->now().nanoseconds());

        // Publish cam
    this->cam_publisher_->publish(this->cam_message_);
  }

public:
  RsuCamGeneratorNode()
  : Node("rsu_cam_generator_node")
  {
        // We can prepare the CAM in advance, as most of it is static data

    std::random_device random_device;      // Obtain a random seed from the hardware
    std::mt19937 engine(random_device());     // Seed the generator
    std::uniform_int_distribution<StationID::_value_type> distribution(
      StationID::MIN,
      StationID::MAX
    );     // Define the distribution for allow range

        // Generate Station ID
    auto station_id = distribution(engine);
    RCLCPP_INFO(this->get_logger(), "Station ID: %u", station_id);

        // RSU Position
    auto position_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    position_param_desc.description =
      "Position of the RSU in WGS84 coordinates. Order: Latitude [degrees], "
      "Longitude [degrees], Altitude [m]";
    position_param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    this->declare_parameter("position", std::vector<double>{0., 0., 0.}, position_param_desc);
    const auto position = this->get_parameter("position").as_double_array();

        // Check if the vector has exactly three elements
    if (position.size() != 3) {
      std::cerr << "Error: Position parameter does not have exactly three elements." << std::endl;
      std::exit(EXIT_FAILURE);
    }

    RCLCPP_INFO(this->get_logger(), "Position: [%f, %f, %f]", position[0], position[1],
      position[2]);

        // Header
    this->cam_message_.header.protocol_version = 2;     // V1.4.1
    this->cam_message_.header.message_id = ItsPduHeader::MESSAGE_ID_CAM;
    this->cam_message_.header.station_id.value = station_id;

        // Basic Container
    this->cam_message_.cam.cam_parameters.basic_container.station_type.value =
      StationType::ROAD_SIDE_UNIT;  // We are a RSU
    etsi_its_cam_msgs::access::setReferencePosition(this->cam_message_, position[0], position[1],
      position[2]);
    this->cam_message_.cam.cam_parameters.basic_container.reference_position.
    position_confidence_ellipse.semi_major_orientation.value =
      HeadingValue::WGS84_NORTH;
    this->cam_message_.cam.cam_parameters.basic_container.reference_position.
    position_confidence_ellipse.semi_minor_confidence.value =
      SemiAxisLength::MIN;
    this->cam_message_.cam.cam_parameters.basic_container.reference_position.
    position_confidence_ellipse.semi_major_confidence.value =
      SemiAxisLength::MIN;
    this->cam_message_.cam.cam_parameters.basic_container.reference_position.altitude.
    altitude_confidence.value =
      AltitudeConfidence::ALT_000_01;

        // High Frequency Container
    this->cam_message_.cam.cam_parameters.high_frequency_container.choice =
      HighFrequencyContainer::CHOICE_RSU_CONTAINER_HIGH_FREQUENCY;
    this->cam_message_.cam.cam_parameters.high_frequency_container.rsu_container_high_frequency.
    protected_communication_zones_rsu_is_present =
      false;           // We are not in a protected communication zone

    // No other containers are present
    // Not defined for RSUs yet
    this->cam_message_.cam.cam_parameters.low_frequency_container_is_present = false;
    // Not applicable for RSUs
    this->cam_message_.cam.cam_parameters.special_vehicle_container_is_present = false;

        // Publishers
    this->cam_publisher_ = this->create_publisher<etsi_its_cam_msgs::msg::CAM>("cam",
      rclcpp::SystemDefaultsQoS());

        // Timers
        // The CAM generation frequency for RSU ITS-Ss defined by the time interval
        // between two consecutive CAM generations
        // shall be set in such a way, that at least one CAM is transmitted
        // while a vehicle is in the communication zone of the RSU ITS-S.
        // The time interval shall be greater than or equal to 1 000 ms.
        // This corresponds to a maximum CAM generation rate of 1 Hz.
    this->cam_gen_timer_ = this->create_wall_timer(
                1s,
      [this] {generateCam();}
    );
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RsuCamGeneratorNode>();

  RCLCPP_INFO(node->get_logger(), "Started RSU CAM generator.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Stopped RSU CAM generator.");

  return 0;
}
