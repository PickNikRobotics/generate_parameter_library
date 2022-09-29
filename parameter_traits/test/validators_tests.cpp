// Copyright (c) 2022, PickNik Inc.
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
//    * Neither the name of the copyright holder nor the names of its
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

#include <rclcpp/parameter.hpp>

#include <parameter_traits/result.hpp>
#include <parameter_traits/validators.hpp>

namespace parameter_traits {
using rclcpp::Parameter;

TEST(ValidatorsTests, Bounds) {
  EXPECT_TRUE(bounds<double>(Parameter{"", 1.0}, 1.0, 5.0).success());
  EXPECT_TRUE(bounds<double>(Parameter{"", 4.3}, 1.0, 5.0).success());
  EXPECT_TRUE(bounds<double>(Parameter{"", 5.0}, 1.0, 5.0).success());
  EXPECT_FALSE(bounds<double>(Parameter{"", -4.3}, 1.0, 5.0).success());
  EXPECT_FALSE(bounds<double>(Parameter{"", 10.2}, 1.0, 5.0).success());

  EXPECT_TRUE(bounds<int64_t>(Parameter{"", 1}, 1, 5).success());
  EXPECT_TRUE(bounds<int64_t>(Parameter{"", 4}, 1, 5).success());
  EXPECT_TRUE(bounds<int64_t>(Parameter{"", 5}, 1, 5).success());
  EXPECT_FALSE(bounds<int64_t>(Parameter{"", -4}, 1, 5).success());
  EXPECT_FALSE(bounds<int64_t>(Parameter{"", 10}, 1, 5).success());

  EXPECT_TRUE(bounds<bool>(Parameter{"", true}, false, true).success());
  EXPECT_FALSE(bounds<bool>(Parameter{"", false}, true, true).success());
  EXPECT_FALSE(bounds<bool>(Parameter{"", true}, false, false).success());

  EXPECT_ANY_THROW(bounds<int64_t>(Parameter{"", ""}, 1, 5));
  EXPECT_ANY_THROW(bounds<std::string>(Parameter{"", 4}, "", "foo"));
}

TEST(ValidatorsTests, LowerBounds) {
  EXPECT_TRUE(lower_bounds<double>(Parameter{"", 2.0}, 2.0).success());
  EXPECT_TRUE(lower_bounds<double>(Parameter{"", 4.3}, 1.0).success());
  EXPECT_FALSE(lower_bounds<double>(Parameter{"", -4.3}, 1.0).success());

  EXPECT_TRUE(lower_bounds<int64_t>(Parameter{"", 1}, 1).success());
  EXPECT_TRUE(lower_bounds<int64_t>(Parameter{"", 4}, 1).success());
  EXPECT_FALSE(lower_bounds<int64_t>(Parameter{"", -4}, 1).success());

  EXPECT_TRUE(lower_bounds<bool>(Parameter{"", true}, false).success());
  EXPECT_FALSE(lower_bounds<bool>(Parameter{"", false}, true).success());
  EXPECT_TRUE(lower_bounds<bool>(Parameter{"", true}, true).success());
}

TEST(ValidatorsTests, UpperBounds) {
  EXPECT_TRUE(upper_bounds<double>(Parameter{"", 2.0}, 2.0).success());
  EXPECT_FALSE(upper_bounds<double>(Parameter{"", 4.3}, 1.0).success());
  EXPECT_TRUE(upper_bounds<double>(Parameter{"", -4.3}, 1.0).success());

  EXPECT_TRUE(upper_bounds<int64_t>(Parameter{"", 1}, 1).success());
  EXPECT_FALSE(upper_bounds<int64_t>(Parameter{"", 4}, 1).success());
  EXPECT_TRUE(upper_bounds<int64_t>(Parameter{"", -4}, 1).success());

  EXPECT_FALSE(upper_bounds<bool>(Parameter{"", true}, false).success());
  EXPECT_TRUE(upper_bounds<bool>(Parameter{"", false}, true).success());
  EXPECT_TRUE(upper_bounds<bool>(Parameter{"", true}, true).success());
}

TEST(ValidatorsTests, GreaterThan) {
  EXPECT_FALSE(gt<double>(Parameter{"", 2.0}, 2.0).success());
  EXPECT_TRUE(gt<double>(Parameter{"", 4.3}, 1.0).success());
  EXPECT_FALSE(gt<double>(Parameter{"", -4.3}, 1.0).success());

  EXPECT_FALSE(gt<int64_t>(Parameter{"", 1}, 1).success());
  EXPECT_TRUE(gt<int64_t>(Parameter{"", 4}, 1).success());
  EXPECT_FALSE(gt<int64_t>(Parameter{"", -4}, 1).success());

  EXPECT_TRUE(gt<bool>(Parameter{"", true}, false).success());
  EXPECT_FALSE(gt<bool>(Parameter{"", false}, true).success());
  EXPECT_FALSE(gt<bool>(Parameter{"", true}, true).success());
}

TEST(ValidatorsTests, LessThan) {
  EXPECT_FALSE(lt<double>(Parameter{"", 2.0}, 2.0).success());
  EXPECT_FALSE(lt<double>(Parameter{"", 4.3}, 1.0).success());
  EXPECT_TRUE(lt<double>(Parameter{"", -4.3}, 1.0).success());

  EXPECT_FALSE(lt<int64_t>(Parameter{"", 1}, 1).success());
  EXPECT_FALSE(lt<int64_t>(Parameter{"", 4}, 1).success());
  EXPECT_TRUE(lt<int64_t>(Parameter{"", -4}, 1).success());

  EXPECT_FALSE(lt<bool>(Parameter{"", true}, false).success());
  EXPECT_TRUE(lt<bool>(Parameter{"", false}, true).success());
  EXPECT_FALSE(lt<bool>(Parameter{"", true}, true).success());
}

TEST(ValidatorsTests, GreaterThanOrEqual) {
  EXPECT_TRUE(gt_eq<double>(Parameter{"", 2.0}, 2.0).success());
  EXPECT_TRUE(gt_eq<double>(Parameter{"", 4.3}, 1.0).success());
  EXPECT_FALSE(gt_eq<double>(Parameter{"", -4.3}, 1.0).success());

  EXPECT_TRUE(gt_eq<int64_t>(Parameter{"", 1}, 1).success());
  EXPECT_TRUE(gt_eq<int64_t>(Parameter{"", 4}, 1).success());
  EXPECT_FALSE(gt_eq<int64_t>(Parameter{"", -4}, 1).success());

  EXPECT_TRUE(gt_eq<bool>(Parameter{"", true}, false).success());
  EXPECT_FALSE(gt_eq<bool>(Parameter{"", false}, true).success());
  EXPECT_TRUE(gt_eq<bool>(Parameter{"", true}, true).success());
}

TEST(ValidatorsTests, LessThanOrEqual) {
  EXPECT_TRUE(lt_eq<double>(Parameter{"", 2.0}, 2.0).success());
  EXPECT_FALSE(lt_eq<double>(Parameter{"", 4.3}, 1.0).success());
  EXPECT_TRUE(lt_eq<double>(Parameter{"", -4.3}, 1.0).success());

  EXPECT_TRUE(lt_eq<int64_t>(Parameter{"", 1}, 1).success());
  EXPECT_FALSE(lt_eq<int64_t>(Parameter{"", 4}, 1).success());
  EXPECT_TRUE(lt_eq<int64_t>(Parameter{"", -4}, 1).success());

  EXPECT_FALSE(lt_eq<bool>(Parameter{"", true}, false).success());
  EXPECT_TRUE(lt_eq<bool>(Parameter{"", false}, true).success());
  EXPECT_TRUE(lt_eq<bool>(Parameter{"", true}, true).success());
}

TEST(ValidatorsTests, OneOf) {
  EXPECT_TRUE(
      one_of<double>(Parameter{"", 2.0}, std::vector<double>{2.0}).success());
  EXPECT_TRUE(
      one_of<double>(Parameter{"", 2.0}, std::vector<double>{1.0, 2.0, 3.5})
          .success());
  EXPECT_FALSE(
      one_of<double>(Parameter{"", 0.0}, std::vector<double>{1.0, 2.0, 3.5})
          .success());
  EXPECT_FALSE(
      one_of<double>(Parameter{"", 0.0}, std::vector<double>{}).success());

  EXPECT_TRUE(
      one_of<int64_t>(Parameter{"", 1}, std::vector<int64_t>{1}).success());
  EXPECT_TRUE(
      one_of<int64_t>(Parameter{"", 1}, std::vector<int64_t>{1, 2, 3, 4})
          .success());
  EXPECT_FALSE(
      one_of<int64_t>(Parameter{"", 0}, std::vector<int64_t>{1, 2, 3, 4})
          .success());
  EXPECT_FALSE(
      one_of<int64_t>(Parameter{"", 0}, std::vector<int64_t>{}).success());

  EXPECT_FALSE(
      one_of<bool>(Parameter{"", true}, std::vector<bool>{false}).success());
  EXPECT_TRUE(one_of<bool>(Parameter{"", true}, std::vector<bool>{false, true})
                  .success());
  EXPECT_TRUE(
      one_of<bool>(Parameter{"", true}, std::vector<bool>{true}).success());
  EXPECT_FALSE(
      one_of<bool>(Parameter{"", true}, std::vector<bool>{}).success());
}

TEST(ValidatorsTests, FixedSizeString) {
  EXPECT_TRUE(fixed_size<std::string>(Parameter{"", "foo"}, 3).success());
  EXPECT_TRUE(fixed_size<std::string>(Parameter{"", ""}, 0).success());
  EXPECT_FALSE(fixed_size<std::string>(Parameter{"", "foo"}, 0).success());
  EXPECT_FALSE(fixed_size<std::string>(Parameter{"", "foo"}, 5).success());
}

TEST(ValidatorsTests, SizeGtString) {
  EXPECT_TRUE(size_gt<std::string>(Parameter{"", "foo"}, 2).success());
  EXPECT_FALSE(size_gt<std::string>(Parameter{"", ""}, 0).success());
  EXPECT_FALSE(size_gt<std::string>(Parameter{"", "foo"}, 5).success());
}

TEST(ValidatorsTests, SizeLtString) {
  EXPECT_TRUE(size_lt<std::string>(Parameter{"", "foo"}, 5).success());
  EXPECT_FALSE(size_lt<std::string>(Parameter{"", ""}, 0).success());
  EXPECT_FALSE(size_lt<std::string>(Parameter{"", "foo"}, 3).success());
}

TEST(ValidatorsTests, NotEmptyString) {
  EXPECT_TRUE(not_empty<std::string>(Parameter{"", "foo"}).success());
  EXPECT_FALSE(not_empty<std::string>(Parameter{"", ""}).success());
}

TEST(ValidatorsTests, OneOfString) {
  EXPECT_TRUE(one_of<std::string>(Parameter{"", "foo"},
                                  std::vector<std::string>{"foo", "baz"})
                  .success());
  EXPECT_FALSE(one_of<std::string>(Parameter{"", ""},
                                   std::vector<std::string>{"foo", "baz"})
                   .success());
}

TEST(ValidatorsTests, UniqueArray) {
  EXPECT_TRUE(
      unique<std::string>(Parameter{"", std::vector<std::string>{"", "1", "2"}})
          .success());
  EXPECT_TRUE(
      unique<std::string>(Parameter{"", std::vector<std::string>{}}).success());
  EXPECT_FALSE(
      unique<std::string>(Parameter{"", std::vector<std::string>{"foo", "foo"}})
          .success());

  EXPECT_TRUE(unique<double>(Parameter{"", std::vector<double>{1.0, 2.2, 1.1}})
                  .success());
  EXPECT_TRUE(unique<double>(Parameter{"", std::vector<double>{}}).success());
  EXPECT_FALSE(
      unique<double>(Parameter{"", std::vector<double>{1.1, 1.1}}).success());

  EXPECT_TRUE(
      unique<int64_t>(Parameter{"", std::vector<int64_t>{1, 2, 3}}).success());
  EXPECT_TRUE(unique<int64_t>(Parameter{"", std::vector<int64_t>{}}).success());
  EXPECT_FALSE(
      unique<int64_t>(Parameter{"", std::vector<int64_t>{-1, -1}}).success());

  EXPECT_TRUE(
      unique<bool>(Parameter{"", std::vector<bool>{true, false}}).success());
  EXPECT_TRUE(unique<bool>(Parameter{"", std::vector<bool>{}}).success());
  EXPECT_FALSE(
      unique<bool>(Parameter{"", std::vector<bool>{false, false}}).success());
}

TEST(ValidatorsTests, SubsetOfArray) {
  EXPECT_TRUE(subset_of<std::string>(
                  Parameter{"", std::vector<std::string>{"", "1", "2"}},
                  std::vector<std::string>{"", "1", "2", "three"})
                  .success());
  EXPECT_TRUE(
      subset_of<std::string>(Parameter{"", std::vector<std::string>{}},
                             std::vector<std::string>{"", "1", "2", "three"})
          .success());
  EXPECT_FALSE(subset_of<std::string>(
                   Parameter{"", std::vector<std::string>{"foo", "foo"}},
                   std::vector<std::string>{"", "1", "2", "three"})
                   .success());

  EXPECT_TRUE(
      subset_of<double>(Parameter{"", std::vector<double>{1.0, 2.2, 1.1}},
                        std::vector<double>{1.0, 2.2, 1.1})
          .success());
  EXPECT_TRUE(subset_of<double>(Parameter{"", std::vector<double>{}},
                                std::vector<double>{10, 22})
                  .success());
  EXPECT_FALSE(subset_of<double>(Parameter{"", std::vector<double>{1.1, 1.1}},
                                 std::vector<double>{1.0, 2.2})
                   .success());

  EXPECT_TRUE(subset_of<int64_t>(Parameter{"", std::vector<int64_t>{1, 2, 3}},
                                 std::vector<int64_t>{1, 2, 3})
                  .success());
  EXPECT_TRUE(subset_of<int64_t>(Parameter{"", std::vector<int64_t>{}},
                                 std::vector<int64_t>{1, 2, 3})
                  .success());
  EXPECT_FALSE(subset_of<int64_t>(Parameter{"", std::vector<int64_t>{-1, -1}},
                                  std::vector<int64_t>{1, 2, 3})
                   .success());

  EXPECT_TRUE(subset_of<bool>(Parameter{"", std::vector<bool>{true, false}},
                              std::vector<bool>{true, false})
                  .success());
  EXPECT_TRUE(subset_of<bool>(Parameter{"", std::vector<bool>{}},
                              std::vector<bool>{true, false})
                  .success());
  EXPECT_FALSE(subset_of<bool>(Parameter{"", std::vector<bool>{false, false}},
                               std::vector<bool>{true})
                   .success());
}

TEST(ValidatorsTests, FixedSizeArray) {
  EXPECT_TRUE(fixed_size<std::string>(
                  Parameter{"", std::vector<std::string>{"", "1", "2"}}, 3)
                  .success());
  EXPECT_TRUE(
      fixed_size<std::string>(Parameter{"", std::vector<std::string>{}}, 0)
          .success());
  EXPECT_FALSE(fixed_size<std::string>(
                   Parameter{"", std::vector<std::string>{"foo", "foo"}}, 3)
                   .success());

  EXPECT_TRUE(
      fixed_size<double>(Parameter{"", std::vector<double>{1.0, 2.2, 1.1}}, 3)
          .success());
  EXPECT_TRUE(
      fixed_size<double>(Parameter{"", std::vector<double>{}}, 0).success());
  EXPECT_FALSE(
      fixed_size<double>(Parameter{"", std::vector<double>{1.1, 1.1}}, 3)
          .success());

  EXPECT_TRUE(
      fixed_size<int64_t>(Parameter{"", std::vector<int64_t>{1, 2, 3}}, 3)
          .success());
  EXPECT_TRUE(
      fixed_size<int64_t>(Parameter{"", std::vector<int64_t>{}}, 0).success());
  EXPECT_FALSE(
      fixed_size<int64_t>(Parameter{"", std::vector<int64_t>{-1, -1}}, 0)
          .success());

  EXPECT_TRUE(fixed_size<bool>(Parameter{"", std::vector<bool>{true, false}}, 2)
                  .success());
  EXPECT_TRUE(
      fixed_size<bool>(Parameter{"", std::vector<bool>{}}, 0).success());
  EXPECT_FALSE(
      fixed_size<bool>(Parameter{"", std::vector<bool>{false, false}}, 0)
          .success());
}

TEST(ValidatorsTests, SizeGtArray) {
  EXPECT_TRUE(size_gt<std::string>(
                  Parameter{"", std::vector<std::string>{"", "1", "2"}}, 1)
                  .success());
  EXPECT_FALSE(
      size_gt<std::string>(Parameter{"", std::vector<std::string>{}}, 0)
          .success());
  EXPECT_FALSE(size_gt<std::string>(
                   Parameter{"", std::vector<std::string>{"foo", "foo"}}, 3)
                   .success());

  EXPECT_TRUE(
      size_gt<double>(Parameter{"", std::vector<double>{1.0, 2.2, 1.1}}, 2)
          .success());
  EXPECT_FALSE(
      size_gt<double>(Parameter{"", std::vector<double>{}}, 0).success());
  EXPECT_FALSE(size_gt<double>(Parameter{"", std::vector<double>{1.1, 1.1}}, 3)
                   .success());

  EXPECT_TRUE(size_gt<int64_t>(Parameter{"", std::vector<int64_t>{1, 2, 3}}, 2)
                  .success());
  EXPECT_FALSE(
      size_gt<int64_t>(Parameter{"", std::vector<int64_t>{}}, 0).success());
  EXPECT_TRUE(size_gt<int64_t>(Parameter{"", std::vector<int64_t>{-1, -1}}, 0)
                  .success());

  EXPECT_TRUE(size_gt<bool>(Parameter{"", std::vector<bool>{true, false}}, 0)
                  .success());
  EXPECT_FALSE(size_gt<bool>(Parameter{"", std::vector<bool>{}}, 0).success());
  EXPECT_TRUE(size_gt<bool>(Parameter{"", std::vector<bool>{false, false}}, 0)
                  .success());
}

TEST(ValidatorsTests, SizeLtArray) {
  EXPECT_FALSE(size_lt<std::string>(
                   Parameter{"", std::vector<std::string>{"", "1", "2"}}, 1)
                   .success());
  EXPECT_FALSE(
      size_lt<std::string>(Parameter{"", std::vector<std::string>{}}, 0)
          .success());
  EXPECT_TRUE(size_lt<std::string>(
                  Parameter{"", std::vector<std::string>{"foo", "foo"}}, 3)
                  .success());

  EXPECT_FALSE(
      size_lt<double>(Parameter{"", std::vector<double>{1.0, 2.2, 1.1}}, 2)
          .success());
  EXPECT_FALSE(
      size_lt<double>(Parameter{"", std::vector<double>{}}, 0).success());
  EXPECT_TRUE(size_lt<double>(Parameter{"", std::vector<double>{1.1, 1.1}}, 3)
                  .success());

  EXPECT_FALSE(size_lt<int64_t>(Parameter{"", std::vector<int64_t>{1, 2, 3}}, 2)
                   .success());
  EXPECT_FALSE(
      size_lt<int64_t>(Parameter{"", std::vector<int64_t>{}}, 0).success());
  EXPECT_FALSE(size_lt<int64_t>(Parameter{"", std::vector<int64_t>{-1, -1}}, 0)
                   .success());

  EXPECT_TRUE(size_lt<bool>(Parameter{"", std::vector<bool>{true, false}}, 3)
                  .success());
  EXPECT_FALSE(size_lt<bool>(Parameter{"", std::vector<bool>{}}, 0).success());
  EXPECT_FALSE(size_lt<bool>(Parameter{"", std::vector<bool>{false, false}}, 0)
                   .success());
}

TEST(ValidatorsTests, NotEmptyArray) {
  EXPECT_TRUE(not_empty<std::string>(
                  Parameter{"", std::vector<std::string>{"", "1", "2"}})
                  .success());
  EXPECT_FALSE(not_empty<std::string>(Parameter{"", std::vector<std::string>{}})
                   .success());

  EXPECT_TRUE(
      not_empty<double>(Parameter{"", std::vector<double>{1.0, 2.2, 1.1}})
          .success());
  EXPECT_FALSE(
      not_empty<double>(Parameter{"", std::vector<double>{}}).success());

  EXPECT_TRUE(not_empty<int64_t>(Parameter{"", std::vector<int64_t>{1, 2, 3}})
                  .success());
  EXPECT_FALSE(
      not_empty<int64_t>(Parameter{"", std::vector<int64_t>{}}).success());

  EXPECT_TRUE(
      not_empty<bool>(Parameter{"", std::vector<bool>{true, false}}).success());
  EXPECT_FALSE(not_empty<bool>(Parameter{"", std::vector<bool>{}}).success());
}

TEST(ValidatorsTests, ElementBoundsArray) {
  EXPECT_TRUE(element_bounds<double>(
                  Parameter{"", std::vector<double>{1.0, 2.2, 1.1}}, 0.0, 3.0)
                  .success());
  EXPECT_TRUE(
      element_bounds<double>(Parameter{"", std::vector<double>{}}, 0.0, 1.0)
          .success());

  EXPECT_TRUE(element_bounds<int64_t>(
                  Parameter{"", std::vector<int64_t>{1, 2, 3}}, 0, 5)
                  .success());
  EXPECT_FALSE(element_bounds<int64_t>(
                   Parameter{"", std::vector<int64_t>{1, 2, 3}}, -5, 0)
                   .success());
  EXPECT_TRUE(
      element_bounds<int64_t>(Parameter{"", std::vector<int64_t>{}}, 0, 1)
          .success());

  EXPECT_TRUE(element_bounds<bool>(
                  Parameter{"", std::vector<bool>{true, false}}, false, true)
                  .success());
  EXPECT_FALSE(element_bounds<bool>(
                   Parameter{"", std::vector<bool>{true, false}}, true, false)
                   .success());
  EXPECT_FALSE(element_bounds<bool>(
                   Parameter{"", std::vector<bool>{true, false}}, false, false)
                   .success());
  EXPECT_TRUE(
      element_bounds<bool>(Parameter{"", std::vector<bool>{}}, true, false)
          .success());
}

TEST(ValidatorsTests, LowerElementBoundsArray) {
  EXPECT_TRUE(lower_element_bounds<double>(
                  Parameter{"", std::vector<double>{1.0, 2.2, 1.1}}, 0.0)
                  .success());
  EXPECT_TRUE(
      lower_element_bounds<double>(Parameter{"", std::vector<double>{}}, 0.0)
          .success());

  EXPECT_TRUE(lower_element_bounds<int64_t>(
                  Parameter{"", std::vector<int64_t>{1, 2, 3}}, 0)
                  .success());
  EXPECT_FALSE(lower_element_bounds<int64_t>(
                   Parameter{"", std::vector<int64_t>{1, 2, 3}}, 3)
                   .success());
  EXPECT_TRUE(
      lower_element_bounds<int64_t>(Parameter{"", std::vector<int64_t>{}}, 0)
          .success());

  EXPECT_TRUE(lower_element_bounds<bool>(
                  Parameter{"", std::vector<bool>{true, false}}, false)
                  .success());
  EXPECT_FALSE(lower_element_bounds<bool>(
                   Parameter{"", std::vector<bool>{true, false}}, true)
                   .success());
  EXPECT_TRUE(lower_element_bounds<bool>(
                  Parameter{"", std::vector<bool>{true, false}}, false)
                  .success());
  EXPECT_TRUE(
      lower_element_bounds<bool>(Parameter{"", std::vector<bool>{}}, true)
          .success());
}

TEST(ValidatorsTests, UpperElementBoundsArray) {
  EXPECT_FALSE(upper_element_bounds<double>(
                   Parameter{"", std::vector<double>{1.0, 2.2, 1.1}}, 0.0)
                   .success());
  EXPECT_TRUE(
      upper_element_bounds<double>(Parameter{"", std::vector<double>{}}, 0.0)
          .success());

  EXPECT_FALSE(upper_element_bounds<int64_t>(
                   Parameter{"", std::vector<int64_t>{1, 2, 3}}, 0)
                   .success());
  EXPECT_TRUE(upper_element_bounds<int64_t>(
                  Parameter{"", std::vector<int64_t>{1, 2, 3}}, 3)
                  .success());
  EXPECT_TRUE(
      upper_element_bounds<int64_t>(Parameter{"", std::vector<int64_t>{}}, 0)
          .success());

  EXPECT_FALSE(upper_element_bounds<bool>(
                   Parameter{"", std::vector<bool>{true, false}}, false)
                   .success());
  EXPECT_TRUE(upper_element_bounds<bool>(
                  Parameter{"", std::vector<bool>{true, false}}, true)
                  .success());
  EXPECT_FALSE(upper_element_bounds<bool>(
                   Parameter{"", std::vector<bool>{true, false}}, false)
                   .success());
  EXPECT_TRUE(
      upper_element_bounds<bool>(Parameter{"", std::vector<bool>{}}, true)
          .success());
}

}  // namespace parameter_traits

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
