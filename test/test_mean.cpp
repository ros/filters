// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <vector>
#include "filters/mean.hpp"

namespace filters
{

TEST(MultiChannelMeanFilterDouble, ConfirmIdentityNRows) {
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;

  auto node = rclcpp::Node::make_shared("MultiChannelMeanFilterDouble5");

  MultiChannelFilterBase<double> * filter = new MultiChannelMeanFilter<double>();
  EXPECT_TRUE(filter->configure(rows, node));

  double input1[] = {1, 2, 3, 4, 5};
  double input1a[] = {1, 2, 3, 4, 5};
  std::vector<double> v1(input1, input1 + sizeof(input1) / sizeof(double));
  std::vector<double> v1a(input1a, input1a + sizeof(input1a) / sizeof(double));

  for (int32_t i = 0; i < rows * 10; i++) {
    EXPECT_TRUE(filter->update(v1, v1a));
    for (int i = 1; i < length; i++) {
      EXPECT_NEAR(v1[i], v1a[i], epsilon);
    }
  }
}

TEST(MultiChannelMeanFilterDouble, ThreeRows) {
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;

  auto node = rclcpp::Node::make_shared("MultiChannelMeanFilterDouble5");

  MultiChannelFilterBase<double> * filter = new MultiChannelMeanFilter<double>();

  EXPECT_TRUE(filter->configure(rows, node));

  double input1[] = {0, 1, 2, 3, 4};
  std::vector<double> v1(input1, input1 + sizeof(input1) / sizeof(double));
  double input2[] = {1, 2, 3, 4, 5};
  std::vector<double> v2(input2, input2 + sizeof(input2) / sizeof(double));
  double input3[] = {2, 3, 4, 5, 6};
  std::vector<double> v3(input3, input3 + sizeof(input3) / sizeof(double));
  double input1a[] = {1, 2, 3, 4, 5};
  std::vector<double> v1a(input1a, input1a + sizeof(input1a) / sizeof(double));

  EXPECT_TRUE(filter->update(v1, v1a));
  EXPECT_TRUE(filter->update(v2, v1a));
  EXPECT_TRUE(filter->update(v3, v1a));

  for (int i = 1; i < length; i++) {
    EXPECT_NEAR(v2[i], v1a[i], epsilon);
  }
}

TEST(MeanFilterDouble, ConfirmIdentityNRows) {
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;

  FilterBase<double> * filter = new MeanFilter<double>();
  auto node = rclcpp::Node::make_shared("MeanFilterDouble5");
  EXPECT_TRUE(filter->configure(node));

  double input = 1;
  double output = 0;

  for (int32_t i = 0; i < rows * 10; i++) {
    EXPECT_TRUE(filter->update(input, output));
    for (int i = 1; i < length; i++) {
      EXPECT_NEAR(input, output, epsilon);
    }
  }
}

TEST(MeanFilterDouble, ThreeRows) {
  double epsilon = 1e-6;
  FilterBase<double> * filter = new MeanFilter<double>();

  auto node = rclcpp::Node::make_shared("MeanFilterDouble5");

  EXPECT_TRUE(filter->configure(node));
  double input1 = 0;
  double input2 = 1;
  double input3 = 2;
  double output = 3;

  EXPECT_TRUE(filter->update(input1, output));
  EXPECT_NEAR(input1, output, epsilon);
  EXPECT_TRUE(filter->update(input2, output));
  EXPECT_NEAR((input1 + input2) / 2.0, output, epsilon);
  EXPECT_TRUE(filter->update(input3, output));
  EXPECT_NEAR((input1 + input2 + input3) / 3, output, epsilon);
}
}  //  namespace filters
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
