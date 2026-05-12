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

#ifndef UFIL_CENTRAL_FUSION__SENSOR_DATA_HPP_
#define UFIL_CENTRAL_FUSION__SENSOR_DATA_HPP_

#include <string>

#include <ufil_object_tracking/types/scalar.hpp>
#include <ufil_object_tracking/types/sensor_fov.hpp>
#include <ufil_object_tracking/types/vector.hpp>

namespace ufil_central_fusion
{

class SensorData
{
public:
  using VectorType = ufil::type::Vector<7>;

private:
  ufil::type::Scalar existence_probability_trust_ = 0.0f;
  ufil::type::SensorFOV sensor_fov_;
  bool occlusion_measurements_ = false;
  bool persistence_measurements_ = false;
  bool cam_message_ = false;
  bool prediction_sensor_ = false;
  bool has_fov_ = false;
  std::string topic_ = "";

  VectorType classification_trust_vector_ = VectorType::Zero();

public:
  static constexpr int CAR = 0;
  static constexpr int TRUCK = 1;
  static constexpr int MOTORCYCLE = 2;
  static constexpr int BICYCLE = 3;
  static constexpr int PEDESTRIAN = 4;
  static constexpr int STATIONARY = 5;
  static constexpr int OTHER = 6;

  ufil::type::Scalar carTrust() const;
  ufil::type::Scalar truckTrust() const;
  ufil::type::Scalar motorcycleTrust() const;
  ufil::type::Scalar pedestrianTrust() const;
  ufil::type::Scalar bicycleTrust() const;
  ufil::type::Scalar stationaryTrust() const;
  ufil::type::Scalar otherTrust() const;

  ufil::type::Scalar & carTrust();
  ufil::type::Scalar & truckTrust();
  ufil::type::Scalar & motorcycleTrust();
  ufil::type::Scalar & pedestrianTrust();
  ufil::type::Scalar & bicycleTrust();
  ufil::type::Scalar & stationaryTrust();
  ufil::type::Scalar & otherTrust();
  ufil::type::Scalar existenceProbTrust() const;
  ufil::type::Scalar & existenceProbTrust();

  ufil::type::SensorFOV sensorFOV() const;
  ufil::type::SensorFOV & sensorFOV();
  bool occlusionPossible() const;
  bool & occlusionPossible();
  bool persistencePossible() const;
  bool & persistencePossible();
  bool hasFov() const;
  bool & hasFov();
  bool isCam() const;
  bool & isCam();
  bool isPredictionSensor() const;
  bool & isPredictionSensor();
  std::string currentTopic() const;
  std::string & currentTopic();

  [[nodiscard]] const VectorType & classificationTrustVector() const;

  [[nodiscard]] VectorType & classificationTrustVector();
};

}  // namespace ufil_central_fusion


#endif  // UFIL_CENTRAL_FUSION__SENSOR_DATA_HPP_
