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

#include <algorithm>
#include <string>
#include <unordered_set>

#include <boost/functional/hash.hpp>

#include <ufil_object_tracking/types/id.hpp>

TEST(Id, generateIdUniqueness)
{
  std::unordered_set<ufil::type::Id, boost::hash<ufil::type::Id>> ids;
  for (int i = 0; i < 100; ++i) {
    auto id = ufil::generateId();
    EXPECT_EQ(ids.count(id), 0u) << "Duplicate ID generated";
    ids.insert(id);
  }
}

TEST(Id, idHashReturnsExpectedType)
{
  auto id = ufil::generateId();
  auto hash_value = ufil::hashId<std::size_t>(id);
  static_assert(std::is_same_v<decltype(hash_value), std::size_t>, "Expected std::size_t");
  EXPECT_GT(hash_value, 0u);
}

TEST(Id, idHashIsDeterministic)
{
  auto id = ufil::generateId();
  auto hash1 = ufil::hashId<uint64_t>(id);
  auto hash2 = ufil::hashId<uint64_t>(id);
  EXPECT_EQ(hash1, hash2);
}

TEST(Id, idCanBeUsedInUnorderedMap)
{
  std::unordered_map<ufil::type::Id, std::string, boost::hash<ufil::type::Id>> id_map;
  auto id = ufil::generateId();
  id_map[id] = "test";
  EXPECT_EQ(id_map.at(id), "test");
}

TEST(Id, idIsNotAllZero)
{
  auto id = ufil::generateId();
  bool all_zero = std::all_of(id.begin(), id.end(), [](auto byte) {return byte == 0;});
  EXPECT_FALSE(all_zero);
}

TEST(Id, manyIdsFast)
{
  constexpr int num_ids = 10000;
  std::unordered_set<ufil::type::Id, boost::hash<ufil::type::Id>> id_set;
  for (int i = 0; i < num_ids; ++i) {
    id_set.insert(ufil::generateId());
  }
  EXPECT_EQ(id_set.size(), num_ids);
}

TEST(Id, equalityOperator)
{
  ufil::type::Id id1 = ufil::generateId();
  ufil::type::Id id2 = ufil::generateId();

  EXPECT_FALSE(id1 == id2);
  EXPECT_TRUE(id1 != id2);
}

TEST(Id, lessThanOperator)
{
  ufil::type::Id id1 = ufil::generateId();
  ufil::type::Id id2 = ufil::generateId();

  EXPECT_TRUE((id1 < id2) || (id2 < id1) || (id1 == id2));
}

TEST(Id, greaterThanOperator)
{
  ufil::type::Id id1 = ufil::generateId();
  ufil::type::Id id2 = ufil::generateId();

  EXPECT_TRUE((id1 > id2) || (id2 > id1) || (id1 == id2));
}

TEST(Id, lessThanOrEqualOperator)
{
  ufil::type::Id id1 = ufil::generateId();
  ufil::type::Id id2 = ufil::generateId();

  EXPECT_TRUE((id1 <= id2) || (id2 <= id1));
}

TEST(Id, greaterThanOrEqualOperator)
{
  ufil::type::Id id1 = ufil::generateId();
  ufil::type::Id id2 = ufil::generateId();

  EXPECT_TRUE((id1 >= id2) || (id2 >= id1));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
