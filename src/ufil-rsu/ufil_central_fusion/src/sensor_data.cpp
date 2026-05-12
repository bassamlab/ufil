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

#include "ufil_central_fusion/sensor_data.hpp"

namespace ufil_central_fusion
{

ufil::type::Scalar SensorData::carTrust() const
{
  return this->classification_trust_vector_[CAR];
}
ufil::type::Scalar SensorData::truckTrust() const
{
  return this->classification_trust_vector_[TRUCK];
}
ufil::type::Scalar SensorData::motorcycleTrust() const
{
  return this->classification_trust_vector_[MOTORCYCLE];
}
ufil::type::Scalar SensorData::pedestrianTrust() const
{
  return this->classification_trust_vector_[PEDESTRIAN];
}
ufil::type::Scalar SensorData::bicycleTrust() const
{
  return this->classification_trust_vector_[BICYCLE];
}
ufil::type::Scalar SensorData::stationaryTrust() const
{
  return this->classification_trust_vector_[STATIONARY];
}
ufil::type::Scalar SensorData::otherTrust() const
{
  return this->classification_trust_vector_[OTHER];
}

ufil::type::Scalar & SensorData::carTrust()
{
  return this->classification_trust_vector_[CAR];
}
ufil::type::Scalar & SensorData::truckTrust()
{
  return this->classification_trust_vector_[TRUCK];
}
ufil::type::Scalar & SensorData::motorcycleTrust()
{
  return this->classification_trust_vector_[MOTORCYCLE];
}
ufil::type::Scalar & SensorData::pedestrianTrust()
{
  return this->classification_trust_vector_[PEDESTRIAN];
}
ufil::type::Scalar & SensorData::bicycleTrust()
{
  return this->classification_trust_vector_[BICYCLE];
}
ufil::type::Scalar & SensorData::stationaryTrust()
{
  return this->classification_trust_vector_[STATIONARY];
}
ufil::type::Scalar & SensorData::otherTrust()
{
  return this->classification_trust_vector_[OTHER];
}

ufil::type::Scalar SensorData::existenceProbTrust() const
{
  return this->existence_probability_trust_;
}
ufil::type::Scalar & SensorData::existenceProbTrust()
{
  return this->existence_probability_trust_;
}

ufil::type::SensorFOV SensorData::sensorFOV() const
{
  return this->sensor_fov_;
}
ufil::type::SensorFOV & SensorData::sensorFOV()
{
  return this->sensor_fov_;
}

bool SensorData::occlusionPossible() const
{
  return this->occlusion_measurements_;
}
bool & SensorData::occlusionPossible()
{
  return this->occlusion_measurements_;
}
bool SensorData::persistencePossible() const
{
  return this->persistence_measurements_;
}
bool & SensorData::persistencePossible()
{
  return this->persistence_measurements_;
}
bool SensorData::hasFov() const
{
  return this->has_fov_;
}
bool & SensorData::hasFov()
{
  return this->has_fov_;
}
bool SensorData::isCam() const
{
  return this->cam_message_;
}
bool & SensorData::isCam()
{
  return this->cam_message_;
}
bool SensorData::isPredictionSensor() const
{
  return this->prediction_sensor_;
}
bool & SensorData::isPredictionSensor()
{
  return this->prediction_sensor_;
}
std::string SensorData::currentTopic() const
{
  return this->topic_;
}
std::string & SensorData::currentTopic()
{
  return this->topic_;
}

[[nodiscard]] const SensorData::VectorType & SensorData::classificationTrustVector() const
{
  return this->classification_trust_vector_;
}

[[nodiscard]] SensorData::VectorType & SensorData::classificationTrustVector()
{
  return this->classification_trust_vector_;
}
}   // namespace ufil_central_fusion
