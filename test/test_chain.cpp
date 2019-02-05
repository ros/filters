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
#include <string>
#include <vector>
#include "filters/filter_chain.hpp"

TEST(MultiChannelFilterChain, configuring) {
  double epsilon = 1e-9;
  auto node = rclcpp::Node::make_shared("MultiChannelMeanFilterDouble5");
  filters::MultiChannelFilterChain<double> chain("double");
  EXPECT_TRUE(chain.configure("MultiChannelMeanFilterDouble5", 5, node));
  double input1[] = {1, 2, 3, 4, 5};
  double input1a[] = {9, 9, 9, 9, 9};  //  seed w/incorrect values
  std::vector<double> v1(input1, input1 + sizeof(input1) / sizeof(double));
  std::vector<double> v1a(input1a, input1a + sizeof(input1a) / sizeof(double));

  EXPECT_TRUE(chain.update(v1, v1a));
  chain.clear();

  for (unsigned int i = 1; i < v1.size(); i++) {
    EXPECT_NEAR(input1[i], v1a[i], epsilon);
  }
}

TEST(FilterChain, configuring) {
  double epsilon = 1e-9;

  auto node = rclcpp::Node::make_shared("MeanFilterFloat5");
  filters::FilterChain<float> chain("float");

  EXPECT_TRUE(chain.configure("MeanFilterFloat5", node));

  float v1 = 1;
  float v1a = 9;

  EXPECT_TRUE(chain.update(v1, v1a));
  chain.clear();
  EXPECT_NEAR(v1, v1a, epsilon);
}

TEST(MultiChannelFilterChain, TwoFilters) {
  double epsilon = 1e-9;

  auto node = rclcpp::Node::make_shared("TwoFilters");
  filters::MultiChannelFilterChain<double> chain("double");

  EXPECT_TRUE(chain.configure("TwoFilters", 5, node));

  double input1[] = {1, 2, 3, 4, 5};
  double input1a[] = {9, 9, 9, 9, 9};  // seed w/incorrect values
  std::vector<double> v1(input1, input1 + sizeof(input1) / sizeof(double));
  std::vector<double> v1a(input1a, input1a + sizeof(input1a) / sizeof(double));

  EXPECT_TRUE(chain.update(v1, v1a));

  chain.clear();

  for (unsigned int i = 1; i < v1.size(); i++) {
    EXPECT_NEAR(input1[i], v1a[i], epsilon);
  }
}

TEST(MultiChannelFilterChain, MisconfiguredNumberOfChannels) {
  filters::MultiChannelFilterChain<double> chain("double");

  auto node = rclcpp::Node::make_shared("MultiChannelMedianFilterDouble5");

  EXPECT_TRUE(chain.configure("MultiChannelMedianFilterDouble5", 10, node));

  double input1[] = {1, 2, 3, 4, 5};
  double input1a[] = {1, 2, 3, 4, 5};
  std::vector<double> v1(input1, input1 + sizeof(input1) / sizeof(double));
  std::vector<double> v1a(input1a, input1a + sizeof(input1a) / sizeof(double));

  EXPECT_FALSE(chain.update(v1, v1a));
  chain.clear();
}

TEST(FilterChain, ReconfiguringChain) {
  filters::FilterChain<int> chain("int");

  int v1 = 1;
  int v1a = 9;
  auto node1 = rclcpp::Node::make_shared("OneIncrements");

  EXPECT_TRUE(chain.configure("OneIncrements", node1));

  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(2, v1a);
  chain.clear();

  auto node2 = rclcpp::Node::make_shared("TwoIncrements");

  EXPECT_TRUE(chain.configure("TwoIncrements", node2));

  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(3, v1a);
  chain.clear();
}

TEST(FilterChain, ThreeIncrementChains) {
  filters::FilterChain<int> chain("int");
  int v1 = 1;
  int v1a = 9;
  auto node = rclcpp::Node::make_shared("ThreeIncrements");

  EXPECT_TRUE(chain.configure("ThreeIncrements", node));
  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(4, v1a);
  chain.clear();
}

TEST(FilterChain, TenIncrementChains) {
  filters::FilterChain<int> chain("int");
  int v1 = 1;
  int v1a = 9;
  auto node = rclcpp::Node::make_shared("TenIncrements");

  EXPECT_TRUE(chain.configure("TenIncrements", node));
  EXPECT_TRUE(chain.update(v1, v1a));
  EXPECT_EQ(11, v1a);
  chain.clear();
}

TEST(MultiChannelFilterChain, TenMultiChannelIncrementChains) {
  filters::MultiChannelFilterChain<int> chain("int");
  std::vector<int> v1;
  v1.push_back(1);
  v1.push_back(1);
  v1.push_back(1);
  std::vector<int> v1a = v1;
  auto node = rclcpp::Node::make_shared("TenMultiChannelIncrements");

  EXPECT_TRUE(chain.configure("TenMultiChannelIncrements", 3, node));
  EXPECT_TRUE(chain.update(v1, v1a));
  for (unsigned int i = 0; i < 3; i++) {
    EXPECT_EQ(11, v1a[i]);
  }
  chain.clear();
}

TEST(MultiChannelFilterChain, TransferFunction) {
  double epsilon = 1e-4;
  auto node = rclcpp::Node::make_shared("TransferFunction");
  filters::MultiChannelFilterChain<double> chain("double");
  EXPECT_TRUE(chain.configure("TransferFunction", 3, node));

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
  EXPECT_TRUE(chain.update(in1, in1));
  EXPECT_TRUE(chain.update(in2, in2));
  EXPECT_TRUE(chain.update(in3, in3));
  EXPECT_TRUE(chain.update(in4, in4));
  EXPECT_TRUE(chain.update(in5, in5));
  EXPECT_TRUE(chain.update(in6, in6));
  EXPECT_TRUE(chain.update(in7, in7));
  chain.clear();

  for (unsigned int i = 0; i < out1.size(); i++) {
    EXPECT_NEAR(out1[i], in7[i], epsilon);
  }
}

int main(int argc, char ** argv)
{
  //  cerr<<"Rohit Main"<<endl;
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
