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

#ifndef UFIL_OBJECT_TRACKING__UPDATE__CLASSIFICATION__DIMENSION_CLASSIFICATION_UPDATER_HPP_
#define UFIL_OBJECT_TRACKING__UPDATE__CLASSIFICATION__DIMENSION_CLASSIFICATION_UPDATER_HPP_

#include <algorithm>
#include <array>
#include <cassert>
#include <optional>
#include <utility>

#include <boost/math/distributions/normal.hpp>

#include "ufil_object_tracking/types/classification.hpp"
#include "ufil_object_tracking/update/classification/classification_updater.hpp"

namespace ufil
{
namespace update
{
namespace classification
{
namespace detail
{

struct DimensionClassificationModel
{
  int type;
  boost::math::normal_distribution<ufil::type::Scalar> length_distribution;
  boost::math::normal_distribution<ufil::type::Scalar> width_distribution;
};

inline const std::array<DimensionClassificationModel, 5> &
default_dimension_classification_models()
{
  static const std::array<DimensionClassificationModel, 5> models {{
    {ufil::type::classification::ObjectClassification::CAR,
      boost::math::normal_distribution<ufil::type::Scalar>(4.78136594658106, 0.7405154662553934),
      boost::math::normal_distribution<ufil::type::Scalar>(2.065247167255217, 0.1713185490328349)},
    {ufil::type::classification::ObjectClassification::TRUCK,
      boost::math::normal_distribution<ufil::type::Scalar>(14.94116370718271, 4.401035030979956),
      boost::math::normal_distribution<ufil::type::Scalar>(3.338806607811286, 0.34003353294077876)},
    {ufil::type::classification::ObjectClassification::PEDESTRIAN,
      boost::math::normal_distribution<ufil::type::Scalar>(0.67344, 0.08745747309407012),
      boost::math::normal_distribution<ufil::type::Scalar>(0.67344, 0.08745747309407012)},
    {ufil::type::classification::ObjectClassification::BICYCLE,
      boost::math::normal_distribution<ufil::type::Scalar>(1.6624409090909091, 0.15823143840328807),
      boost::math::normal_distribution<ufil::type::Scalar>(0.8660125000000001,
                  0.09036279882745897)},
    {ufil::type::classification::ObjectClassification::MOTORCYCLE,
      boost::math::normal_distribution<ufil::type::Scalar>(2.0362854838709676, 0.25687370804746873),
      boost::math::normal_distribution<ufil::type::Scalar>(0.8852185483870968, 0.105957147734154)},
  }};

  return models;
}

}  // namespace detail

/**
 * Requires a DimensionType with length()/width() accessors and an
 * ObjectClassification-compatible classification vector layout.
 */
template<typename T>
class DimensionClassificationUpdater : public ClassificationUpdater<T>
{
protected:
  using Base = ClassificationUpdater<T>;
  using TrackType = typename Base::TrackType;
  using MeasurementType = typename Base::MeasurementType;
  using ClassificationType = typename Base::ClassificationType;
  using DimensionType = typename TrackType::DimensionType;

public:
  using Base::update;

  explicit DimensionClassificationUpdater(
    const ufil::type::Scalar classification_probability_radius = 0.2)
  : classification_probability_radius_(classification_probability_radius)
  {
    assert(classification_probability_radius >= 0.0);

    static_assert(
      requires(const DimensionType & dimension) {
        dimension.length();
        dimension.width();
      },
      "DimensionClassificationUpdater requires a DimensionType with length() and width().");
  }

  void update(
    const TrackType & track, const std::optional<MeasurementType> & /*measurement*/,
    const ufil::type::Timestamp & /*timestamp*/,
    ClassificationType & resulting_classification) override
  {
    ufil::type::Scalar length = track.currentDimension().length();
    ufil::type::Scalar width = track.currentDimension().width();
    if (length < width) {
      std::swap(length, width);
    }

    for (const auto & model : detail::default_dimension_classification_models()) {
      const auto length_probability =
        boost::math::cdf(
        model.length_distribution, length + this->classification_probability_radius_) -
        boost::math::cdf(
        model.length_distribution, length - this->classification_probability_radius_);

      const auto width_probability =
        boost::math::cdf(
        model.width_distribution, width + this->classification_probability_radius_) -
        boost::math::cdf(
        model.width_distribution, width - this->classification_probability_radius_);

      resulting_classification.classificationVector()(model.type) =
        length_probability * width_probability;
    }

    resulting_classification.normalize();
  }

private:
  ufil::type::Scalar classification_probability_radius_ = 0.2;
};

}  // namespace classification
}  // namespace update
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__UPDATE__CLASSIFICATION__DIMENSION_CLASSIFICATION_UPDATER_HPP_
