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

#include <iostream>

#include "classifier.hpp"

namespace ufil_osn_tracker
{

Classifier::Classifier()
: classes_{ufil::type::classification::ObjectClassification::CAR,
    ufil::type::classification::ObjectClassification::TRUCK,
    ufil::type::classification::ObjectClassification::PEDESTRIAN,
    ufil::type::classification::ObjectClassification::BICYCLE,
    ufil::type::classification::ObjectClassification::MOTORCYCLE}
  , length_distributions_{{ufil::type::classification::ObjectClassification::CAR,
      boost::math::normal_distribution(4.78136594658106f, 0.7405154662553934f)},
    {ufil::type::classification::ObjectClassification::TRUCK,
      boost::math::normal_distribution(14.94116370718271f, 4.401035030979956f)},
    {ufil::type::classification::ObjectClassification::PEDESTRIAN,
      boost::math::normal_distribution(0.67344f, 0.08745747309407012f)},
    {ufil::type::classification::ObjectClassification::BICYCLE,
      boost::math::normal_distribution(1.6624409090909091f, 0.15823143840328807f)},
    {ufil::type::classification::ObjectClassification::MOTORCYCLE,
      boost::math::normal_distribution(2.0362854838709676f, 0.25687370804746873f)}}
  , width_distributions_{{ufil::type::classification::ObjectClassification::CAR,
      boost::math::normal_distribution(2.065247167255217f, 0.1713185490328349f)},
    {ufil::type::classification::ObjectClassification::TRUCK,
      boost::math::normal_distribution(3.338806607811286f, 0.34003353294077876f)},
    {ufil::type::classification::ObjectClassification::PEDESTRIAN,
      boost::math::normal_distribution(0.67344f, 0.08745747309407012f)},
    {ufil::type::classification::ObjectClassification::BICYCLE,
      boost::math::normal_distribution(0.8660125000000001f, 0.09036279882745897f)},
    {ufil::type::classification::ObjectClassification::MOTORCYCLE,
      boost::math::normal_distribution(0.8852185483870968f, 0.105957147734154f)}}
{
}

void Classifier::update(
  Track & track,
  const std::optional<ufil::type::measurement::Pose2DWithDimension3D> & /*measurement*/,
  const ufil::type::Timestamp & /*timestamp*/)
{
  auto & classification = track.currentClassification();
  const auto & dimension = track.currentDimension();
  ufil::type::Scalar length = dimension.length();
  ufil::type::Scalar width = dimension.width();
  if (length < width) {
    const ufil::type::Scalar buffer = length;
    length = width;
    width = buffer;
  }

  // std::cout << " L:" << length << " W:" << width << std::endl;
  for (const auto type : this->classes_) {
    const auto & length_distribution = this->length_distributions_.at(type);
    const auto & width_distribution = this->width_distributions_.at(type);

    const ufil::type::Scalar length_probability =
      (boost::math::cdf(length_distribution, length + classification_probability_radius_) -
      boost::math::cdf(length_distribution, length - classification_probability_radius_));

    const ufil::type::Scalar width_probability =
      (boost::math::cdf(width_distribution, width + classification_probability_radius_) -
      boost::math::cdf(width_distribution, width - classification_probability_radius_));

    const ufil::type::Scalar probability = length_probability * width_probability;
    classification.classificationVector()(type) = probability;
    // std::cout << "T: " << type << " p:" << probability << "| ";
  }
  classification.normalize();
  // std::cout << std::endl;
}

}  // namespace ufil_osn_tracker
