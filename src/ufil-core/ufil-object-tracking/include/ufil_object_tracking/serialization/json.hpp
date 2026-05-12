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

#ifndef UFIL_OBJECT_TRACKING__SERIALIZATION__JSON_HPP_
#define UFIL_OBJECT_TRACKING__SERIALIZATION__JSON_HPP_

#include <iomanip>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <boost/uuid/uuid_io.hpp>

#include "ufil_object_tracking/types/dimension.hpp"
#include "ufil_object_tracking/types/id.hpp"
#include "ufil_object_tracking/types/measurement.hpp"
#include "ufil_object_tracking/types/sensor_fov.hpp"
#include "ufil_object_tracking/types/time.hpp"

namespace ufil
{
namespace serialization
{
namespace detail
{

inline void indent(std::ostream & output, std::size_t level)
{
  for (std::size_t i = 0; i < level; ++i) {
    output.put(' ');
  }
}

inline void write_json_string(std::ostream & output, const std::string & value)
{
  output.put('"');
  for (char c : value) {
    switch (c) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        output.put(c);
        break;
    }
  }
  output.put('"');
}

inline void write_scalar(std::ostream & output, const type::Scalar value)
{
  output << std::setprecision(15) << value;
}

inline void write_bool(std::ostream & output, bool value)
{
  output << (value ? "true" : "false");
}

inline void write_id(std::ostream & output, const type::Id & id)
{
  write_json_string(output, boost::uuids::to_string(id));
}

template<typename TimeLike>
inline void write_time_ns(std::ostream & output, const TimeLike & time)
{
  output << ufil::to_nanoseconds(time);
}

template<typename VectorLike>
inline void write_vector(std::ostream & output, const VectorLike & vector)
{
  output << "[";
  for (int i = 0; i < vector.size(); ++i) {
    if (i > 0) {
      output << ", ";
    }
    write_scalar(output, vector(i));
  }
  output << "]";
}

template<typename T>
inline void write_optional_scalar(std::ostream & output, const std::optional<T> & value)
{
  if (value.has_value()) {
    if constexpr (std::is_same_v<T, type::Scalar>) {
      write_scalar(output, *value);
    } else {
      output << *value;
    }
  } else {
    output << "null";
  }
}

template<typename MatrixLike>
inline void write_matrix(std::ostream & output, const MatrixLike & matrix)
{
  output << "[";
  for (int row = 0; row < matrix.rows(); ++row) {
    if (row > 0) {
      output << ", ";
    }
    output << "[";
    for (int col = 0; col < matrix.cols(); ++col) {
      if (col > 0) {
        output << ", ";
      }
      write_scalar(output, matrix(row, col));
    }
    output << "]";
  }
  output << "]";
}

template<typename ControlType>
inline void serialize_control(
  const ControlType & control, std::ostream & output, std::size_t indent_level)
{
  output << "{\n";
  indent(output, indent_level + 2);
  output << "\"control_vector\": ";
  write_vector(output, control.controlVector());
  output << "\n";
  indent(output, indent_level);
  output << "}";
}

template<typename ClassificationType>
inline void serialize_classification(
  const ClassificationType & classification, std::ostream & output, std::size_t indent_level)
{
  output << "{\n";
  indent(output, indent_level + 2);
  output << "\"classification_vector\": ";
  write_vector(output, classification.classificationVector());
  output << "\n";
  indent(output, indent_level);
  output << "}";
}

template<typename ExistenceProbabilityType>
inline void serialize_existence_probability(
  const ExistenceProbabilityType & existence_probability,
  std::ostream & output,
  std::size_t indent_level)
{
  output << "{\n";
  indent(output, indent_level + 2);
  output << "\"basic_belief_vector\": ";
  write_vector(output, existence_probability.basicBeliefVector());
  output << "\n";
  indent(output, indent_level);
  output << "}";
}

template<typename MeasurementType>
inline void serialize_measurement(
  const MeasurementType & measurement, std::ostream & output, std::size_t indent_level)
{
  output << "{\n";
  indent(output, indent_level + 2);
  output << "\"measurement_id\": ";
  write_id(output, measurement.uuid());
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"measurement_vector\": ";
  write_vector(output, measurement.measurementVector());
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"covariance\": ";
  write_matrix(output, measurement.covariance());
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"classification\": ";
  write_vector(output, measurement.classification().classificationVector());
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"existence_probability\": ";
  write_scalar(output, measurement.existenceProbability());
  output << "\n";
  indent(output, indent_level);
  output << "}";
}

template<typename DimensionType>
inline void serialize_dimension(
  const DimensionType & dimension, std::ostream & output, std::size_t indent_level)
{
  output << "{\n";
  indent(output, indent_level + 2);
  output << "\"dimension_vector\": ";
  write_vector(output, dimension.dimensionVector());
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"covariance\": ";
  write_matrix(output, dimension.covariance());
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"gridmap_buffer\": [";
  const auto & buffer = dimension.dimensionGridmapBuffer();
  for (std::size_t row = 0; row < buffer.size(); ++row) {
    if (row > 0) {
      output << ", ";
    }
    output << "[";
    for (std::size_t col = 0; col < buffer[row].size(); ++col) {
      if (col > 0) {
        output << ", ";
      }
      const auto & cell = buffer[row][col];
      output << "{\"position\": ";
      write_scalar(output, cell.position);
      output << ", \"log_odds\": ";
      write_scalar(output, cell.log_odds);
      output << ", \"probability\": ";
      write_scalar(output, cell.probability);
      output << "}";
    }
    output << "]";
  }
  output << "]\n";
  indent(output, indent_level);
  output << "}";
}

template<typename StateType>
inline void serialize_state(
  const StateType & state, std::ostream & output,
  std::size_t indent_level)
{
  output << "{\n";
  indent(output, indent_level + 2);
  output << "\"timestamp_ns\": " << ufil::to_nanoseconds(state.timestamp()) << ",\n";
  indent(output, indent_level + 2);
  output << "\"state_vector\": ";
  write_vector(output, state.stateVector());
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"covariance\": ";
  write_matrix(output, state.covariance());
  output << "\n";
  indent(output, indent_level);
  output << "}";
}

template<typename EntryType>
inline void serialize_history_entry(
  const EntryType & entry, std::ostream & output, std::size_t indent_level)
{
  output << "{\n";
  indent(output, indent_level + 2);
  output << "\"state\": ";
  serialize_state(entry.state(), output, indent_level + 2);
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"dimension\": ";
  serialize_dimension(entry.dimension(), output, indent_level + 2);
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"classification\": ";
  serialize_classification(entry.classification(), output, indent_level + 2);
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"existence_probability\": ";
  serialize_existence_probability(entry.existenceProbability(), output, indent_level + 2);
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"control_input\": ";
  if (entry.hasControlInput()) {
    serialize_control(*entry.controlInput(), output, indent_level + 2);
  } else {
    output << "null";
  }
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"associated_measurement\": ";
  if (entry.hasAssociatedMeasurement()) {
    serialize_measurement(*entry.associatedMeasurement(), output, indent_level + 2);
  } else {
    output << "null";
  }
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"nis\": ";
  if (entry.nis().has_value()) {
    write_scalar(output, *entry.nis());
  } else {
    output << "null";
  }
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"occluded\": " << (entry.occluded() ? "true" : "false") << "\n";
  indent(output, indent_level);
  output << "}";
}

template<typename TrackType>
inline void serialize_track(
  const TrackType & track,
  std::ostream & output,
  std::size_t indent_level,
  bool include_history)
{
  output << "{\n";
  indent(output, indent_level + 2);
  output << "\"track_id\": ";
  write_id(output, track.uuid());
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"creation_time_ns\": " << ufil::to_nanoseconds(track.creationTime());
  if (include_history) {
    output << ",\n";
    indent(output, indent_level + 2);
    output << "\"history\": [\n";
    bool first_entry = true;
    for (const auto & [timestamp, entry] : track.history()) {
      (void)timestamp;
      if (!first_entry) {
        output << ",\n";
      }
      indent(output, indent_level + 4);
      serialize_history_entry(entry, output, indent_level + 4);
      first_entry = false;
    }
    output << "\n";
    indent(output, indent_level + 2);
    output << "]";
  }
  output << "\n";
  indent(output, indent_level);
  output << "}";
}

}  // namespace detail

template<typename TrackType>
inline std::string to_json(const TrackType & track, bool include_history = true)
{
  std::ostringstream output;
  detail::serialize_track(track, output, 0, include_history);
  return output.str();
}

template<typename TrackType>
inline void write_json(
  const TrackType & track,
  std::ostream & output,
  bool include_history = true,
  std::size_t indent_level = 0)
{
  detail::serialize_track(track, output, indent_level, include_history);
}

namespace detail
{

template<typename T>
inline void serialize_basic_type(const T & value, std::ostream & output, std::size_t indent_level)
{
  if constexpr (std::is_same_v<T, type::Scalar>) {
    write_scalar(output, value);
  } else if constexpr (std::is_same_v<T, bool>) {
    write_bool(output, value);
  } else if constexpr (std::is_same_v<T, type::Id>) {
    write_id(output, value);
  } else {
    static_assert(!sizeof(T), "Unsupported type for JSON serialization");
  }
  (void)indent_level;
}

template<int N>
inline void serialize_vector_type(const type::Vector<N> & vector, std::ostream & output)
{
  write_vector(output, vector);
}

template<int N, int M>
inline void serialize_matrix_type(const type::Matrix<N, M> & matrix, std::ostream & output)
{
  write_matrix(output, matrix);
}

inline void serialize_sensor_fov(
  const type::SensorFOV & sensor_fov, std::ostream & output, std::size_t indent_level)
{
  output << "{\n";
  indent(output, indent_level + 2);
  output << "\"origin\": ";
  write_vector(output, sensor_fov.origin());
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"fov_vector\": ";
  write_vector(output, sensor_fov.vector());
  output << ",\n";
  indent(output, indent_level + 2);
  output << "\"covariance\": ";
  write_matrix(output, sensor_fov.covariance());
  output << "\n";
  indent(output, indent_level);
  output << "}";
}

template<typename ValueT>
inline void serialize_optional(const std::optional<ValueT> & value, std::ostream & output)
{
  if (value.has_value()) {
    serialize_basic_type(*value, output, 0);
  } else {
    output << "null";
  }
}

}  // namespace detail

inline std::string to_json(const type::SensorFOV & sensor_fov)
{
  std::ostringstream output;
  detail::serialize_sensor_fov(sensor_fov, output, 0);
  return output.str();
}

template<int N>
inline std::string to_json(const type::Vector<N> & vector)
{
  std::ostringstream output;
  detail::write_vector(output, vector);
  return output.str();
}

template<int N, int M>
inline std::string to_json(const type::Matrix<N, M> & matrix)
{
  std::ostringstream output;
  detail::write_matrix(output, matrix);
  return output.str();
}

}  // namespace serialization
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__SERIALIZATION__JSON_HPP_
