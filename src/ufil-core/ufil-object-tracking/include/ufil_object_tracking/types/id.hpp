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

#ifndef UFIL_OBJECT_TRACKING__TYPES__ID_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__ID_HPP_

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace ufil
{
namespace type
{

using Id = boost::uuids::uuid;

}  // namespace type

inline type::Id generateId()
{
  thread_local static boost::uuids::random_generator generator;
  return generator();
}

template<typename HashType>
HashType hashId(const type::Id & id)
{
  static_assert(std::is_integral_v<HashType>, "HashType must be an integral type");

  boost::hash<type::Id> uuid_hasher;
  auto hash_value = uuid_hasher(id);

  return static_cast<HashType>(hash_value);
}

class IdInterface
{
protected:
  type::Id uuid_;

public:
  IdInterface()
  : uuid_(generateId())
  {
  }

  explicit IdInterface(const type::Id & uuid)
  : uuid_(uuid)
  {
  }

  virtual ~IdInterface() = default;

  [[nodiscard]] virtual const type::Id uuid() const
  {
    return this->uuid_;
  }

  void uuid(const type::Id & uuid)
  {
    this->uuid_ = uuid;
  }

  [[nodiscard]] virtual bool valid() const
  {
    return !this->uuid_.is_nil();
  }

  bool operator==(const IdInterface & other) const
  {
    return this->uuid_ == other.uuid();
  }

  bool operator<(const IdInterface & other) const
  {
    return this->uuid_ < other.uuid();
  }

  bool operator>(const IdInterface & other) const
  {
    return this->uuid_ > other.uuid();
  }

  bool operator<=(const IdInterface & other) const
  {
    return (*this < other) || (*this == other);
  }

  bool operator>=(const IdInterface & other) const
  {
    return (*this > other) || (*this == other);
  }
};

}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__ID_HPP_
