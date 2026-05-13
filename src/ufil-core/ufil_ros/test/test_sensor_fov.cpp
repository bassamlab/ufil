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

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ufil_ros/ufil_ros.hpp>

TEST(SensorFOVTransform, HandlesPhiWrapping) {
  ufil_msgs::msg::SensorFOV in;
  in.phi_min = static_cast<float>(M_PI - 0.03);   // ~179°
  in.phi_max = static_cast<float>(-M_PI + 0.03);  // ~-179°
  in.theta_min = 0.0f;
  in.theta_max = 0.1f;
  in.origin.x = in.origin.y = in.origin.z = 0.0f;

  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "world";
  tf.child_frame_id = "sensor";
  tf.transform.translation.x = tf.transform.translation.y = tf.transform.translation.z = 0.0;
  double yaw = 0.05;  // small rotation
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  ufil_msgs::msg::SensorFOV out;
  tf2::doTransform(in, out, tf);

  double span = static_cast<double>(out.phi_max) - static_cast<double>(out.phi_min);
  if (span < 0) {span += 2.0 * M_PI;}
  EXPECT_LT(span, 0.2);  // small span expected, e.g. < 0.2 rad
}

TEST(SensorFOVTransform, Rotate90AroundY) {
  ufil_msgs::msg::SensorFOV in;
  in.phi_min = static_cast<float>(-60.0 * M_PI / 180.0);
  in.phi_max = static_cast<float>(60.0 * M_PI / 180.0);
  in.theta_min = static_cast<float>(-20.0 * M_PI / 180.0);
  in.theta_max = static_cast<float>(20.0 * M_PI / 180.0);
  in.origin.x = in.origin.y = in.origin.z = 0.0f;

  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "world";
  tf.child_frame_id = "sensor";
  tf.transform.translation.x = tf.transform.translation.y = tf.transform.translation.z = 0.0;
  double pitch = 0.1;
  tf2::Quaternion q;
  q.setRPY(0.0, pitch, 0.0);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  ufil_msgs::msg::SensorFOV out;
  tf2::doTransform(in, out, tf);

  auto normalize_angle = [](double a){
      while (a > M_PI) {a -= 2.0 * M_PI;}
      while (a <= -M_PI) {a += 2.0 * M_PI;}
      return a;
    };


  EXPECT_NEAR(out.phi_min, in.phi_min, 1e-6);
  EXPECT_NEAR(out.phi_max, in.phi_max, 1e-6);

  EXPECT_NEAR(out.theta_min, in.theta_min + 0.1, 1e-6);
  EXPECT_NEAR(out.theta_max, in.theta_max + 0.1, 1e-6);
}
