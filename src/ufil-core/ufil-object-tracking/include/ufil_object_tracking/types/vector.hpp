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

#ifndef UFIL_OBJECT_TRACKING__TYPES__VECTOR_HPP_
#define UFIL_OBJECT_TRACKING__TYPES__VECTOR_HPP_

namespace ufil
{
namespace type
{

template<int N>
class VectorInterface
{
public:
  using Type = Vector<N>;

protected:
  Type vector_;

public:
  VectorInterface()
  {
    if constexpr (N > 0) {
      this->vector_ = Type::Zero();
    }
  }

  explicit VectorInterface(const Type & vector)
  : vector_(vector)
  {
  }

  [[nodiscard]] virtual const Type & vector() const
  {
    return vector_;
  }

  [[nodiscard]] virtual Type & vector()
  {
    return vector_;
  }

  [[nodiscard]] virtual size_t numberOfElements() const
  {
    return vector_.size();
  }
};

}  // namespace type
}  // namespace ufil

#endif  // UFIL_OBJECT_TRACKING__TYPES__VECTOR_HPP_
