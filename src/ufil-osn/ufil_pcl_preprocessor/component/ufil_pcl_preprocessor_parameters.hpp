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

#ifndef UFIL_PCL_PREPROCESSOR__COMPONENT__UFIL_PCL_PREPROCESSOR_PARAMETERS_HPP_
#define UFIL_PCL_PREPROCESSOR__COMPONENT__UFIL_PCL_PREPROCESSOR_PARAMETERS_HPP_

#include <Eigen/Dense>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace ufil_pcl_preprocessor
{

struct PCLPreprocessorParameters
{
  // Filter parameters
  double filter_voxel_size;
  double filter_outlier_radius;
  double filter_min_number_of_neighbors;
  double filter_z_min;
  double filter_z_max;

  // Frame and time parameters
  std::string target_frame;
  bool use_ros_time;

  // FOV estimator parameters
  int fov_num_bins;
  int fov_history_length;
  float fov_max_range;

  // Depth buffer parameters
  size_t depth_buffer_history_length;
  double depth_buffer_min_range;
  double depth_buffer_max_range;
  double depth_buffer_grid_resolution;
  double depth_buffer_grid_size_x;
  double depth_buffer_grid_size_y;
  double depth_buffer_decay_rate;
  int depth_buffer_az_bins;
  int depth_buffer_el_bins;
  int depth_buffer_preprocess_T_low;
  int depth_buffer_preprocess_T_high;
  int depth_buffer_preprocess_K;
  int depth_buffer_preprocess_R;
  int depth_buffer_preprocess_min_count;
};

class PCLPreprocessorParameterHandler
{
public:
  explicit PCLPreprocessorParameterHandler(rclcpp::Node * node)
  : node_(node)
  {
    declareParameters();
    readParameters();
    setupDynamicParameterCallback();
  }

  ~PCLPreprocessorParameterHandler() = default;

  /**
   * @brief Get current parameters
   * @return Reference to parameters struct
   */
  const PCLPreprocessorParameters & getParameters() const
  {
    return params_;
  }

  /**
   * @brief Update all parameters from node (call this periodically or on demand)
   */
  void updateParameters()
  {
    readParameters();
  }

private:
  rclcpp::Node * node_;
  PCLPreprocessorParameters params_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  void declareParameters()
  {
    // Filter parameters
    node_->declare_parameter("filter_voxel_size", 0.4);
    node_->declare_parameter("filter_outlier_radius", 1.5);
    node_->declare_parameter("filter_min_number_of_neighbors", 5.0);
    node_->declare_parameter("filter_z_min", 0.5);
    node_->declare_parameter("filter_z_max", 5.0);

    // Frame and time parameters
    node_->declare_parameter("target_frame", "sensor_node_base");
    node_->declare_parameter("use_ros_time", false);

    // FOV estimator parameters
    node_->declare_parameter("fov_num_bins", 360);
    node_->declare_parameter("fov_history_length", 100);
    node_->declare_parameter("fov_max_range", 44.0);

    // Depth buffer parameters
    node_->declare_parameter("depth_buffer_history_length", 10);
    node_->declare_parameter("depth_buffer_min_range", 5.0);
    node_->declare_parameter("depth_buffer_max_range", 44.0);
    node_->declare_parameter("depth_buffer_grid_resolution", 0.4);
    node_->declare_parameter("depth_buffer_grid_size_x", 80.0);
    node_->declare_parameter("depth_buffer_grid_size_y", 80.0);
    node_->declare_parameter("depth_buffer_decay_rate", 0.5);
    node_->declare_parameter("depth_buffer_az_bins", 500);
    node_->declare_parameter("depth_buffer_el_bins", 100);
    node_->declare_parameter("depth_buffer_preprocess_T_low", 30);
    node_->declare_parameter("depth_buffer_preprocess_T_high", 40);
    node_->declare_parameter("depth_buffer_preprocess_K", 3);
    node_->declare_parameter("depth_buffer_preprocess_R", 1);
    node_->declare_parameter("depth_buffer_preprocess_min_count", 4);
  }

  void readParameters()
  {
    // Filter parameters
    params_.filter_voxel_size = node_->get_parameter("filter_voxel_size").as_double();
    params_.filter_outlier_radius = node_->get_parameter("filter_outlier_radius").as_double();
    params_.filter_min_number_of_neighbors =
      node_->get_parameter("filter_min_number_of_neighbors").as_double();
    params_.filter_z_min = node_->get_parameter("filter_z_min").as_double();
    params_.filter_z_max = node_->get_parameter("filter_z_max").as_double();

    // Frame and time parameters
    params_.target_frame = node_->get_parameter("target_frame").as_string();
    params_.use_ros_time = node_->get_parameter("use_ros_time").as_bool();

    // FOV estimator parameters
    params_.fov_num_bins = node_->get_parameter("fov_num_bins").as_int();
    params_.fov_history_length = node_->get_parameter("fov_history_length").as_int();
    params_.fov_max_range = node_->get_parameter("fov_max_range").as_double();

    // Depth buffer parameters
    params_.depth_buffer_history_length =
      static_cast<size_t>(node_->get_parameter("depth_buffer_history_length").as_int());
    params_.depth_buffer_min_range = node_->get_parameter("depth_buffer_min_range").as_double();
    params_.depth_buffer_max_range = node_->get_parameter("depth_buffer_max_range").as_double();
    params_.depth_buffer_grid_resolution =
      node_->get_parameter("depth_buffer_grid_resolution").as_double();
    params_.depth_buffer_grid_size_x =
      node_->get_parameter("depth_buffer_grid_size_x").as_double();
    params_.depth_buffer_grid_size_y =
      node_->get_parameter("depth_buffer_grid_size_y").as_double();
    params_.depth_buffer_decay_rate = node_->get_parameter("depth_buffer_decay_rate").as_double();
    params_.depth_buffer_az_bins = node_->get_parameter("depth_buffer_az_bins").as_int();
    params_.depth_buffer_el_bins = node_->get_parameter("depth_buffer_el_bins").as_int();
    params_.depth_buffer_preprocess_T_low =
      node_->get_parameter("depth_buffer_preprocess_T_low").as_int();
    params_.depth_buffer_preprocess_T_high =
      node_->get_parameter("depth_buffer_preprocess_T_high").as_int();
    params_.depth_buffer_preprocess_K = node_->get_parameter("depth_buffer_preprocess_K").as_int();
    params_.depth_buffer_preprocess_R = node_->get_parameter("depth_buffer_preprocess_R").as_int();
    params_.depth_buffer_preprocess_min_count =
      node_->get_parameter("depth_buffer_preprocess_min_count").as_int();
  }

  void setupDynamicParameterCallback()
  {
    param_callback_handle_ = node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) -> rcl_interfaces::msg::
      SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & p : params) {
          try {
            validateParameter(p);
          } catch (const std::exception & e) {
            result.successful = false;
            result.reason = e.what();
            RCLCPP_WARN(node_->get_logger(), "Parameter validation failed: %s", e.what());
            break;
          }
        }

        if (result.successful) {
          readParameters();
          RCLCPP_INFO(node_->get_logger(), "Parameters updated successfully");
        }

        return result;
      });
  }

  void validateParameter(const rclcpp::Parameter & p)
  {
    const auto & name = p.get_name();

    // Validate filter parameters
    if (name == "filter_voxel_size" && p.as_double() <= 0.0) {
      throw std::invalid_argument("filter_voxel_size must be > 0");
    }
    if (name == "filter_outlier_radius" && p.as_double() <= 0.0) {
      throw std::invalid_argument("filter_outlier_radius must be > 0");
    }
    if (name == "filter_min_number_of_neighbors" && p.as_double() < 0.0) {
      throw std::invalid_argument("filter_min_number_of_neighbors must be >= 0");
    }
    if (name == "filter_z_min" && p.as_double() < 0.0) {
      throw std::invalid_argument("filter_z_min must be >= 0");
    }
    if (name == "filter_z_max" && p.as_double() < 0.0) {
      throw std::invalid_argument("filter_z_max must be >= 0");
    }

    // Validate FOV estimator parameters
    if (name == "fov_num_bins" && p.as_int() <= 0) {
      throw std::invalid_argument("fov_num_bins must be > 0");
    }
    if (name == "fov_history_length" && p.as_int() <= 0) {
      throw std::invalid_argument("fov_history_length must be > 0");
    }
    if (name == "fov_max_range" && p.as_double() <= 0.0) {
      throw std::invalid_argument("fov_max_range must be > 0");
    }

    // Validate depth buffer parameters
    if (name == "depth_buffer_history_length" && p.as_int() <= 0) {
      throw std::invalid_argument("depth_buffer_history_length must be > 0");
    }
    if (name == "depth_buffer_min_range" && p.as_double() < 0.0) {
      throw std::invalid_argument("depth_buffer_min_range must be >= 0");
    }
    if (name == "depth_buffer_max_range" && p.as_double() < 0.0) {
      throw std::invalid_argument("depth_buffer_max_range must be >= 0");
    }
    if (name == "depth_buffer_grid_resolution" && p.as_double() <= 0.0) {
      throw std::invalid_argument("depth_buffer_grid_resolution must be > 0");
    }
    if (name == "depth_buffer_grid_size_x" && p.as_double() <= 0.0) {
      throw std::invalid_argument("depth_buffer_grid_size_x must be > 0");
    }
    if (name == "depth_buffer_grid_size_y" && p.as_double() <= 0.0) {
      throw std::invalid_argument("depth_buffer_grid_size_y must be > 0");
    }
    if (name == "depth_buffer_decay_rate") {
      double val = p.as_double();
      if (val < 0.0 || val > 1.0) {
        throw std::invalid_argument("depth_buffer_decay_rate must be in [0.0, 1.0]");
      }
    }
    if (name == "depth_buffer_az_bins" && p.as_int() <= 0) {
      throw std::invalid_argument("depth_buffer_az_bins must be > 0");
    }
    if (name == "depth_buffer_el_bins" && p.as_int() <= 0) {
      throw std::invalid_argument("depth_buffer_el_bins must be > 0");
    }
  }
};

}  // namespace ufil_pcl_preprocessor

#endif  // UFIL_PCL_PREPROCESSOR__COMPONENT__UFIL_PCL_PREPROCESSOR_PARAMETERS_HPP_
