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
#include <sys/time.h>
#include <string>
#include <vector>
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "filters/median.hpp"

namespace filters
{
TEST(MultiChannelMedianFilterDouble, ConfirmIdentityNRows) {
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;

  auto node = rclcpp::Node::make_shared("MultiChannelMedianFilterDouble5");
  MultiChannelFilterBase<double> * filter =
    new filters::MultiChannelMedianFilter<double>();

  EXPECT_TRUE(filter->configure(rows, node));

  double input1[] = {1, 2, 3, 4, 5};
  double input1a[] = {11, 12, 13, 14, 15};
  std::vector<double> v1(input1, input1 + sizeof(input1) / sizeof(double));
  std::vector<double> v1a(input1a, input1a + sizeof(input1a) / sizeof(double));

  for (int i = 0; i < rows * 10; i++) {
    EXPECT_TRUE(filter->update(v1, v1a));

    for (int j = 1; j < length; j++) {
      EXPECT_NEAR(input1[j], v1a[j], epsilon);
    }
  }

  delete filter;
}

TEST(MultiChannelMedianFilterDouble, ThreeRows) {
  double epsilon = 1e-6;
  int length = 5;
  int rows = 5;

  auto node = rclcpp::Node::make_shared("MultiChannelMedianFilterDouble5");
  MultiChannelFilterBase<double> * filter =
    new MultiChannelMedianFilter<double>();

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

TEST(MultiChannelMedianFilterFloat, ConfirmIdentityNRows) {
  float epsilon = 1e-6;
  int length = 5;
  int rows = 5;

  auto node = rclcpp::Node::make_shared("MultiChannelMedianFilterFloat5");
  MultiChannelFilterBase<float> * filter =
    new filters::MultiChannelMedianFilter<float>();
  EXPECT_TRUE(filter->configure(rows, node));

  float input1[] = {1, 2, 3, 4, 5};
  float input1a[] = {1, 2, 3, 4, 5};
  std::vector<float> v1(input1, input1 + sizeof(input1) / sizeof(float));
  std::vector<float> v1a(input1a, input1a + sizeof(input1a) / sizeof(float));

  for (int i = 0; i < rows * 10; i++) {
    EXPECT_TRUE(filter->update(v1, v1a));

    for (int j = 1; j < length; j++) {
      EXPECT_NEAR(input1[j], v1a[j], epsilon);
    }
  }

  delete filter;
}

TEST(MultiChannelMedianFilterFloat, ThreeRows) {
  float epsilon = 1e-6;
  int length = 5;
  int rows = 5;

  auto node = rclcpp::Node::make_shared("MultiChannelMedianFilterFloat5");
  MultiChannelFilterBase<float> * filter = new MultiChannelMedianFilter<float>();
  EXPECT_TRUE(filter->configure(rows, node));

  float input1[] = {0, 1, 2, 3, 4};
  std::vector<float> v1(input1, input1 + sizeof(input1) / sizeof(float));
  float input2[] = {1, 2, 3, 4, 5};
  std::vector<float> v2(input2, input2 + sizeof(input2) / sizeof(float));
  float input3[] = {2, 3, 4, 5, 6};
  std::vector<float> v3(input3, input3 + sizeof(input3) / sizeof(float));
  float input1a[] = {1, 2, 3, 4, 5};
  std::vector<float> v1a(input1a, input1a + sizeof(input1a) / sizeof(float));

  EXPECT_TRUE(filter->update(v1, v1a));
  EXPECT_TRUE(filter->update(v2, v1a));
  EXPECT_TRUE(filter->update(v3, v1a));

  for (int i = 1; i < length; i++) {
    EXPECT_NEAR(v2[i], v1a[i], epsilon);
  }
}
}  //  namespace filters
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
