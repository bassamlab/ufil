// Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWHT Aachen University
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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <optional>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <ufil_ros/ufil_ros.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ufil_central_fusion/object_tracker.hpp>

#include "central_fusion_node_ros.hpp"

using namespace std::chrono_literals;

namespace ufil_central_fusion_node
{

// Convert yaw angle to ROS quaternion
inline geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion tf_q;
  tf_q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(tf_q);
}


class CentralFusionNode : public rclcpp::Node
{
public:
  CentralFusionNode()
  : rclcpp::Node("central_fusion_node", "/rsu")
  {
  // QOS
    auto const qos = rclcpp::SystemDefaultsQoS();

  // Subscriptions
    auto subscriber_callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = subscriber_callback_group;

    if(!this->has_parameter("num_sensors")) {
      this->declare_parameter<int>("num_sensors", 0);
    }
    int num_sensors = this->get_parameter("num_sensors").as_int();
    this->prediction_sensor_ = ufil_central_fusion::SensorData();
    this->prediction_sensor_.currentTopic() = "Prediction";
    this->prediction_sensor_.isPredictionSensor() = true;

    for(int i = 1; i <= num_sensors; i++) {
      std::string sensor_name = "sensor_" + std::to_string(i) + "_configuration";
      if(!this->has_parameter(sensor_name)) {
        this->declare_parameter(sensor_name, std::vector<std::string>{});
      }
      std::vector<std::string> sensor_string_data =
        this->get_parameter(sensor_name).as_string_array();
      ufil_central_fusion::SensorData sensor_data =
        ufil_central_fusion::SensorData();
      if(sensor_string_data.size() != 10) {
        throw std::runtime_error("sensor vector size needs to be 14!");
      }
      sensor_data.existenceProbTrust() = std::stof(sensor_string_data[0]);
      sensor_data.carTrust() = std::stof(sensor_string_data[1]);
      sensor_data.truckTrust() = std::stof(sensor_string_data[2]);
      sensor_data.motorcycleTrust() = std::stof(sensor_string_data[3]);
      sensor_data.pedestrianTrust() = std::stof(sensor_string_data[4]);
      sensor_data.bicycleTrust() = std::stof(sensor_string_data[5]);
      sensor_data.stationaryTrust() = std::stof(sensor_string_data[6]);
      sensor_data.otherTrust() = std::stof(sensor_string_data[7]);
      sensor_data.currentTopic() = sensor_string_data[8];
      sensor_data.occlusionPossible() = false;
      sensor_data.persistencePossible() = false;
      sensor_data.isCam() = sensor_string_data[9] == "true";
      this->sensor_data_vector_.push_back(sensor_data);
    }

    // declaration and pull of non-sensor parameters
    this->declare_parameter("min_existence_weight", 0.2);
    this->declare_parameter("max_existence_weight", 0.8);
    this->declare_parameter("delta_d", 0.1);
    this->declare_parameter("alpha", 0.95);
    this->declare_parameter("p_min", 0.1);
    this->declare_parameter("p_ref", 0.5);
    this->declare_parameter("p_max", 0.9);

    this->declare_parameter("association_threshold", this->association_threshold_);
    this->association_threshold_ = this->get_parameter("association_threshold").as_double();
    RCLCPP_INFO(this->get_logger(), "Using association threshold: %f.",
        this->association_threshold_);

    this->declare_parameter("existence_decay_factor", this->decay_factor_);
    this->decay_factor_ = this->get_parameter("existence_decay_factor").as_double();
    RCLCPP_INFO(this->get_logger(), "Using existence decay factor: %f.", this->decay_factor_);

    min_existence_weight_ = this->get_parameter("min_existence_weight").as_double();
    max_existence_weight_ = this->get_parameter("max_existence_weight").as_double();
    delta_d_ = this->get_parameter("delta_d").as_double();
    alpha_ = this->get_parameter("alpha").as_double();
    p_min_ = this->get_parameter("p_min").as_double();
    p_ref_ = this->get_parameter("p_ref").as_double();
    p_max_ = this->get_parameter("p_max").as_double();

    this->declare_parameter<int>("timer_period_ms", 100);
    auto timer_period = std::chrono::milliseconds(this->get_parameter("timer_period_ms").as_int());

    this->declare_parameter("buffer_time", 2.0);
    buffer_time_ = this->get_parameter("buffer_time").as_double();

    // Create subscriptions for each sensor topic (object lists, FOV, occlusion)
    for (auto & sensor_data : sensor_data_vector_) {
      std::string sensor_ns;
      std::string fov_topic;
      std::string occlusion_topic;

      auto const pos = sensor_data.currentTopic().find('/', 1);
      sensor_ns = sensor_data.currentTopic().substr(0, pos);
      fov_topic = sensor_ns + "/sensor_fov";
      occlusion_topic = sensor_ns + "/dynamic_occlusion_grid";

      const auto topic_callback = [this, sensor_data](const ufil_msgs::msg::ObjectList & msg) {
          objectListCallback(std::move(msg), sensor_data.currentTopic());
        };

      this->subscriptions_[sensor_data.currentTopic()] =
        this->create_subscription<ufil_msgs::msg::ObjectList>(sensor_data.currentTopic(),
        qos,
        topic_callback);

      this->fov_subscriptions_[fov_topic] =
        this->create_subscription<ufil_msgs::msg::SensorFOV>(
          fov_topic,
          qos,
        [this, sensor_data](const ufil_msgs::msg::SensorFOV & fov_msg) {
          this->sensorFovCallback(fov_msg, sensor_data.currentTopic());
          }
        );

      this->occlusion_subscriptions_[occlusion_topic] =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          occlusion_topic,
          qos,
        [this, sensor_data](const nav_msgs::msg::OccupancyGrid & /* occ_msg */) {
          this->sensorOcclusionCallback(sensor_data.currentTopic());
          }
        );
    }

  // Publisher
    this->publisher_ = this->create_publisher<ufil_msgs::msg::ObjectList>("object_list", qos);
    this->fusion_timer_ = rclcpp::create_timer(this, this->get_clock(), timer_period, [&] {
          fusionTimerCallback();
                              });
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    this->initialize();
  }

private:
  bool checkIfTimeIsInQueue(const ufil::type::Timestamp & timestamp) const
  {
    if(time_indices_.empty()) {
      return true;
    }

    ufil::type::Timestamp last_timestamp = time_indices_.back();
    if(timestamp >= last_timestamp) {
      return true;
    }

    double duration = ufil::to_seconds(last_timestamp - timestamp);
    if(duration <= buffer_time_) {
      return true;
    }

    return false;
  }

  bool selectSensorDataFromTopic(
    const std::string & topic, ufil_central_fusion::SensorData & sensor_data) const
  {
    for(const auto & sensor_data_i : this->sensor_data_vector_) {
      if(sensor_data_i.currentTopic() != topic) {
        continue;
      }
      sensor_data = sensor_data_i;
      return true;
    }
    return false;
  }

  // Insert object list measurements into the ring buffer (no immediate processing)
  void objectListCallback(const ufil_msgs::msg::ObjectList & msg, const std::string & topic)
  {
    rclcpp::Time msg_time = msg.header.stamp;
    auto ufil_msg_time = ufil::from_nanoseconds<ufil::type::Timestamp>(msg_time.nanoseconds());

     // Find sensor configuration
    ufil_central_fusion::SensorData currentSensor;
    if(!selectSensorDataFromTopic(topic, currentSensor)) {
      RCLCPP_WARN(this->get_logger(),
          "Received message from topic not registered in sensor data list, discarding message.");
      return;
    }

    // Check time jump guard from previous callbacks (keep original behavior)
    double dt = (last_callback_msg_time_ - msg_time).seconds();
    if(std::abs(dt) > 10.0) {
      if(!currentSensor.isCam()) {
        RCLCPP_WARN(this->get_logger(), "Detected time jump, reset tracker.");
        this->reset();
      } else {
        RCLCPP_WARN(this->get_logger(), "Detected broken UDP package, reject message.");
      }
      return;
    }
    last_callback_msg_time_ = msg_time;

    // Ensure message is within buffer window
    if(!checkIfTimeIsInQueue(ufil_msg_time)) {
      RCLCPP_WARN(this->get_logger(),
          "Received message outside of message buffer queue size, discarding message.");
      return;
    }

    // Transform message into 'map' frame (same as before)
    ufil_msgs::msg::ObjectList transformed_msg;
    try {
      transformed_msg = this->tf_buffer_->transform(msg, "map");
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }

    // Convert objects to measurements
    std::set<ufil_central_fusion::DynamicMeasurement> measurements{};
    for(const auto & object : transformed_msg.objects) {
      auto measurement =
        ufil_ros::measurementFromMsg<ufil_central_fusion::DynamicMeasurement>(object);
      measurement.isCam() = currentSensor.isCam();
      measurements.insert(measurement);
    }

    // Insert into the time-ordered buffer
    {
      // insert into appropriate position to keep time_indices_ sorted
      if(time_indices_.empty() || time_indices_.back() <= ufil_msg_time) {
        time_indices_.push_back(ufil_msg_time);
        measurement_sets_.push_back(measurements);
        sensor_data_indices_.push_back(currentSensor);
      } else {
        // find insertion position
        size_t insert_pos = 0;
        while(insert_pos < time_indices_.size() && time_indices_[insert_pos] <= ufil_msg_time) {
          ++insert_pos;
        }
        time_indices_.insert(time_indices_.begin() + insert_pos, ufil_msg_time);
        measurement_sets_.insert(measurement_sets_.begin() + insert_pos, measurements);
        sensor_data_indices_.insert(sensor_data_indices_.begin() + insert_pos, currentSensor);
      }

      // update earliest recalc time and flag
      if(!earliest_recalc_time_.has_value() || ufil_msg_time < earliest_recalc_time_.value()) {
        earliest_recalc_time_ = ufil_msg_time;
      }
      need_recalc_ = true;
    }
  }

  // Timer callback: perform trimming, recompute tracker from earliest insertion if needed,
  // otherwise do normal prediction and publishing.
  void fusionTimerCallback()
  {
    rclcpp::Time currentRosTime = this->now();
    ufil::type::Timestamp currentTime =
      ufil::from_nanoseconds<ufil::type::Timestamp>(currentRosTime.nanoseconds());

    bool do_recalc = false;
    std::optional<ufil::type::Timestamp> recalc_from;

    // Local snapshot (used only if we need to replay)
    std::vector<ufil::type::Timestamp> times_snapshot;
    std::vector<std::set<ufil_central_fusion::DynamicMeasurement>> meas_snapshot;
    std::vector<ufil_central_fusion::SensorData> sensors_snapshot;

    {
      // Keep this critical section short: trim + decide + snapshot

      // 1) trim old entries outside buffer_time_
      if (!time_indices_.empty()) {
        double duration = ufil::to_seconds(time_indices_.back() - time_indices_.front());
        while (!time_indices_.empty() && duration > buffer_time_) {
          time_indices_.erase(time_indices_.begin());
          measurement_sets_.erase(measurement_sets_.begin());
          sensor_data_indices_.erase(sensor_data_indices_.begin());
          if (time_indices_.empty()) {break;}
          duration = ufil::to_seconds(time_indices_.back() - time_indices_.front());
        }
      }

      // 2) decide whether to recompute
      if (need_recalc_ && earliest_recalc_time_.has_value()) {
        do_recalc = true;
        recalc_from = earliest_recalc_time_;

        // Clear flags now; new inserts can set them again while we replay.
        earliest_recalc_time_.reset();
        need_recalc_ = false;

        // 3) snapshot the current buffer so we can replay without holding the lock
        times_snapshot = time_indices_;
        meas_snapshot = measurement_sets_;
        sensors_snapshot = sensor_data_indices_;
      }
    }

    if (do_recalc && recalc_from.has_value()) {
      // Heavy work happens WITHOUT buffer_mutex_ held
      this->tracker_->rollbackHistory(recalc_from.value());

      for (size_t i = 0; i < times_snapshot.size(); ++i) {
        if (times_snapshot[i] < recalc_from.value()) {
          continue;
        }
        this->tracker_->setCurrentSensorData(sensors_snapshot[i]);

        // Make a local set so update() can std::move it (preserves your current semantics)
        auto current_measurements = meas_snapshot[i];
        this->tracker_->update(std::move(current_measurements), times_snapshot[i]);
      }

      this->tracker_->setCurrentSensorData(this->prediction_sensor_);
      this->tracker_->predict(currentTime);
    } else {
      // No replay needed: keep light and fast
      this->tracker_->setCurrentSensorData(this->prediction_sensor_);
      this->tracker_->predict(currentTime);
    }

    // publish fused tracks
    auto tracks = this->tracker_->tracks();
    ufil_msgs::msg::ObjectList fused_objects = ufil_ros::tracksToMsg(tracks, true, 0.2);
    fused_objects.header.frame_id = "map";
    fused_objects.header.stamp = currentRosTime;
    this->publisher_->publish(fused_objects);
  }


  void sensorFovCallback(
    const ufil_msgs::msg::SensorFOV & msg,
    const std::string & topic)
  {
    for (auto & sensor_data : this->sensor_data_vector_) {
      if (sensor_data.currentTopic() == topic) {
        ufil_msgs::msg::SensorFOV transformed_msg;
        try {
          transformed_msg = this->tf_buffer_->transform(msg, "map");
        } catch (tf2::TransformException & ex) {
          RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
          return;
        }

        sensor_data.persistencePossible() = true;
        sensor_data.hasFov() = true;
        sensor_data.sensorFOV() = ufil_ros::sensorFovFromMsg(transformed_msg);
        break;
      }
    }
  }

  void sensorOcclusionCallback(const std::string & topic)
  {
    for (auto & sensor_data : this->sensor_data_vector_) {
      if (sensor_data.currentTopic() == topic) {
        sensor_data.occlusionPossible() = true;
        break;
      }
    }
  }

private:
  void reset()
  {
    this->clear();
    this->initialize();
  }

  void clear()
  {
    time_indices_.clear();
    measurement_sets_.clear();
    sensor_data_indices_.clear();
    need_recalc_ = false;
    earliest_recalc_time_.reset();
  }

  void initialize()
  {
    last_callback_msg_time_ = this->now();
    this->tracker_ = std::make_unique<ufil_central_fusion::ObjectTracker>(min_existence_weight_,
        max_existence_weight_, decay_factor_,
        delta_d_, alpha_, p_min_, p_ref_,
        p_max_, association_threshold_);
  }

  // members
  ufil::type::Scalar min_existence_weight_;
  ufil::type::Scalar max_existence_weight_;
  ufil::type::Scalar decay_factor_ = 0.01f;
  ufil::type::Scalar delta_d_;
  ufil::type::Scalar alpha_;
  ufil::type::Scalar p_min_;
  ufil::type::Scalar p_ref_;
  ufil::type::Scalar p_max_;
  ufil::type::Scalar association_threshold_ = 5.0f;
  ufil::type::Scalar buffer_time_ = 2.0f;

  rclcpp::Time last_callback_msg;
  std::vector<ufil_central_fusion::SensorData> sensor_data_vector_;
  ufil_central_fusion::SensorData prediction_sensor_;
  std::vector<ufil_central_fusion::SensorData> sensor_data_indices_;
  std::vector<ufil::type::Timestamp> time_indices_;
  std::vector<std::set<ufil_central_fusion::DynamicMeasurement>> measurement_sets_;

  rclcpp::Time last_callback_msg_time_;

  std::map<std::string, rclcpp::Subscription<ufil_msgs::msg::ObjectList>::SharedPtr> subscriptions_;
  std::map<std::string,
    rclcpp::Subscription<ufil_msgs::msg::SensorFOV>::SharedPtr> fov_subscriptions_;
  std::map<std::string,
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> occlusion_subscriptions_;

  rclcpp::Publisher<ufil_msgs::msg::ObjectList>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr fusion_timer_;

  ufil_central_fusion::ObjectTracker::UniquePtr tracker_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // synchronization and recompute tracking
  mutable std::mutex buffer_mutex_;
  bool need_recalc_ = false;
  std::optional<ufil::type::Timestamp> earliest_recalc_time_;
};
}  // namespace ufil_central_fusion_node


auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ufil_central_fusion_node::CentralFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
