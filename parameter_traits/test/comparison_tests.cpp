// Copyright 2022 PickNik Inc
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "gtest/gtest.h"

#include <string>
#include <vector>

#include <parameter_traits/comparison.hpp>

namespace pv = parameter_traits;

TEST(ComparisonTests, NotContainsEmptyStrVec) {
  EXPECT_FALSE(pv::contains(std::vector<std::string>(), std::string{}));
}

TEST(ComparisonTests, ContainsEmptyStr) {
  EXPECT_TRUE(pv::contains(std::vector<std::string>({""}), std::string{}));
}

TEST(ComparisonTests, ContainsAStr) {
  EXPECT_TRUE(pv::contains(std::vector<std::string>({"a", "b", "c"}),
                           std::string{"a"}));
}

TEST(ComparisonTests, NotContainsAStr) {
  EXPECT_FALSE(pv::contains(std::vector<std::string>({"aa", "bb", "cc"}),
                            std::string{"a"}));
}

TEST(ComparisonTests, NotContainsEmptyIntVec) {
  EXPECT_FALSE(pv::contains<int>({}, int{}));
}

TEST(ComparisonTests, ContainsAInt) {
  EXPECT_TRUE(pv::contains<int>({1, 2, 3}, int{1}));
}

TEST(ComparisonTests, NotContainsAInt) {
  EXPECT_FALSE(pv::contains<int>({11, 22, 33}, int{1}));
}

TEST(ComparisonTests, EmptyIntIsUnique) { EXPECT_TRUE(pv::is_unique<int>({})); }

TEST(ComparisonTests, EmptyStrIsUnique) {
  EXPECT_TRUE(pv::is_unique<std::string>({}));
}

TEST(ComparisonTests, RepeatedIntNotIsUnique) {
  EXPECT_FALSE(pv::is_unique<int>({1, 1}));
}

TEST(ComparisonTests, NotRepeatedIntIsUnique) {
  EXPECT_TRUE(pv::is_unique<int>({1, 2, 3, 4, 5}));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
