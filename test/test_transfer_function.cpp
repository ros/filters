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

//  #include <ros/ros.h>
#include <gtest/gtest.h>
#include <sys/time.h>
#include <vector>
#include <string>
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "filters/transfer_function.hpp"

namespace filters
{

TEST(MultiChannelTransferFunctionDoubleFilter, LowPass) {
  double epsilon = 1e-4;

  auto node = rclcpp::Node::make_shared("LowPass");
  MultiChannelFilterBase<double> * filter =
    new MultiChannelTransferFunctionFilter<double>();
  EXPECT_TRUE(filter->configure(1, node));

  std::vector<double> in1, in2, in3, in4, in5, in6, in7;
  std::vector<double> out1;

  in1.push_back(10.0);
  in2.push_back(70.0);
  in3.push_back(10.0);
  in4.push_back(44.0);
  in5.push_back(10.0);
  in6.push_back(5.0);
  in7.push_back(6.0);
  out1.push_back(11.8008);
  EXPECT_TRUE(filter->update(in1, in1));
  EXPECT_TRUE(filter->update(in2, in2));
  EXPECT_TRUE(filter->update(in3, in3));
  EXPECT_TRUE(filter->update(in4, in4));
  EXPECT_TRUE(filter->update(in5, in5));
  EXPECT_TRUE(filter->update(in6, in6));
  EXPECT_TRUE(filter->update(in7, in7));

  EXPECT_NEAR(out1[0], in7[0], epsilon);
}

TEST(SingleChannelTransferFunctionDoubleFilter, SingleLowPass) {
  double epsilon = 1e-4;

  auto node = rclcpp::Node::make_shared("LowPassSingle");
  FilterBase<double> * filter =
    new SingleChannelTransferFunctionFilter<double>();
  EXPECT_TRUE(filter->configure(node));

  double in1, in2, in3, in4, in5, in6, in7;
  double out1;

  in1 = 10.0;
  in2 = 70.0;
  in3 = 10.0;
  in4 = 44.0;
  in5 = 10.0;
  in6 = 5.0;
  in7 = 6.0;
  out1 = 11.8008;
  EXPECT_TRUE(filter->update(in1, in1));
  EXPECT_TRUE(filter->update(in2, in2));
  EXPECT_TRUE(filter->update(in3, in3));
  EXPECT_TRUE(filter->update(in4, in4));
  EXPECT_TRUE(filter->update(in5, in5));
  EXPECT_TRUE(filter->update(in6, in6));
  EXPECT_TRUE(filter->update(in7, in7));

  EXPECT_NEAR(out1, in7, epsilon);
}

TEST(MultiChannelTransferFunctionDoubleFilter, LowPassNonUnity) {
  double epsilon = 1e-4;

  auto node = rclcpp::Node::make_shared("LowPassNonUnity");
  MultiChannelFilterBase<double> * filter =
    new MultiChannelTransferFunctionFilter<double>();
  EXPECT_TRUE(filter->configure(1, node));

  std::vector<double> in1, in2, in3, in4, in5, in6, in7;
  std::vector<double> out1;

  in1.push_back(10.0);
  in2.push_back(70.0);
  in3.push_back(10.0);
  in4.push_back(44.0);
  in5.push_back(10.0);
  in6.push_back(5.0);
  in7.push_back(6.0);
  out1.push_back(2.4088);
  EXPECT_TRUE(filter->update(in1, in1));
  EXPECT_TRUE(filter->update(in2, in2));
  EXPECT_TRUE(filter->update(in3, in3));
  EXPECT_TRUE(filter->update(in4, in4));
  EXPECT_TRUE(filter->update(in5, in5));
  EXPECT_TRUE(filter->update(in6, in6));
  EXPECT_TRUE(filter->update(in7, in7));

  EXPECT_NEAR(out1[0], in7[0], epsilon);
}

TEST(MultiChannelTransferFunctionDoubleFilter, LowPassMulti) {
  double epsilon = 1e-4;

  auto node = rclcpp::Node::make_shared("LowPassMulti");
  MultiChannelFilterBase<double> * filter =
    new MultiChannelTransferFunctionFilter<double>();
  EXPECT_TRUE(filter->configure(3, node));

  std::vector<double> in1, in2, in3, in4, in5, in6, in7;
  std::vector<double> out1;

  in1.push_back(10.0);
  in1.push_back(10.0);
  in1.push_back(10.0);
  //
  in2.push_back(70.0);
  in2.push_back(30.0);
  in2.push_back(8.0);
  //
  in3.push_back(-1.0);
  in3.push_back(5.0);
  in3.push_back(22.0);
  //
  in4.push_back(44.0);
  in4.push_back(23.0);
  in4.push_back(8.0);
  //
  in5.push_back(10.0);
  in5.push_back(10.0);
  in5.push_back(10.0);
  //
  in6.push_back(5.0);
  in6.push_back(-1.0);
  in6.push_back(5.0);
  //
  in7.push_back(6.0);
  in7.push_back(-30.0);
  in7.push_back(2.0);
  //
  out1.push_back(60.6216);
  out1.push_back(33.9829);
  out1.push_back(28.1027);
  EXPECT_TRUE(filter->update(in1, in1));
  EXPECT_TRUE(filter->update(in2, in2));
  EXPECT_TRUE(filter->update(in3, in3));
  EXPECT_TRUE(filter->update(in4, in4));
  EXPECT_TRUE(filter->update(in5, in5));
  EXPECT_TRUE(filter->update(in6, in6));
  EXPECT_TRUE(filter->update(in7, in7));

  for (unsigned int i = 0; i < out1.size(); i++) {
    EXPECT_NEAR(out1[i], in7[i], epsilon);
  }
}

TEST(MultiChannelTransferFunctionDoubleFilter, LowPassIrrational) {
  double epsilon = 1e-4;

  auto node = rclcpp::Node::make_shared("LowPassIrrational");
  MultiChannelFilterBase<double> * filter =
    new MultiChannelTransferFunctionFilter<double>();
  EXPECT_TRUE(filter->configure(3, node));

  std::vector<double> in1, in2, in3, in4, in5, in6, in7;
  std::vector<double> out1;

  in1.push_back(10.0);
  in1.push_back(10.0);
  in1.push_back(10.0);
  //
  in2.push_back(70.0);
  in2.push_back(30.0);
  in2.push_back(8.0);
  //
  in3.push_back(-1.0);
  in3.push_back(5.0);
  in3.push_back(22.0);
  //
  in4.push_back(44.0);
  in4.push_back(23.0);
  in4.push_back(8.0);
  //
  in5.push_back(10.0);
  in5.push_back(10.0);
  in5.push_back(10.0);
  //
  in6.push_back(5.0);
  in6.push_back(-1.0);
  in6.push_back(5.0);
  //
  in7.push_back(6.0);
  in7.push_back(-30.0);
  in7.push_back(2.0);
  //
  out1.push_back(17.1112);
  out1.push_back(9.0285);
  out1.push_back(8.3102);
  EXPECT_TRUE(filter->update(in1, in1));
  EXPECT_TRUE(filter->update(in2, in2));
  EXPECT_TRUE(filter->update(in3, in3));
  EXPECT_TRUE(filter->update(in4, in4));
  EXPECT_TRUE(filter->update(in5, in5));
  EXPECT_TRUE(filter->update(in6, in6));
  EXPECT_TRUE(filter->update(in7, in7));

  for (unsigned int i = 0; i < out1.size(); i++) {
    EXPECT_NEAR(out1[i], in7[i], epsilon);
  }
}
}  //  namespace filters
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
