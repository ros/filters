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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "filters/param_test.hpp"

TEST(Parameters, Double)
{
  auto node = rclcpp::Node::make_shared("TestDouble");
  double epsilon = 1e-6;

  filters::FilterBase<double> * filter = new filters::ParamTest<double>();
  EXPECT_TRUE(filter->configure(node));
  double out;
  filter->update(out, out);
  EXPECT_NEAR(4, out, epsilon);
  delete filter;
}

TEST(Parameters, Int)
{
  auto node = rclcpp::Node::make_shared("TestInt");

  filters::FilterBase<int> * filter = new filters::ParamTest<int>();
  EXPECT_TRUE(filter->configure(node));
  int out;
  filter->update(out, out);
  EXPECT_EQ(4, out);
  delete filter;
}

TEST(Parameters, UInt)
{
  auto node = rclcpp::Node::make_shared("TestUInt");
  filters::FilterBase<unsigned int> * filter = new filters::ParamTest<unsigned int>();
  EXPECT_TRUE(filter->configure(node));
  unsigned int out;
  filter->update(out, out);
  EXPECT_EQ(4, out);
  delete filter;
}

TEST(Parameters, String)
{
  auto node = rclcpp::Node::make_shared("TestString");
  filters::FilterBase<std::string> * filter = new filters::ParamTest<std::string>();
  EXPECT_TRUE(filter->configure(node));
  std::string out;
  filter->update(out, out);
  EXPECT_STREQ("four", out.c_str());
  delete filter;
}

TEST(Parameters, DoubleVector)
{
  auto node = rclcpp::Node::make_shared("TestDoubleVector");
  double epsilon = 1e-6;

  filters::FilterBase<std::vector<double>> * filter = new filters::ParamTest<std::vector<double>>();
  EXPECT_TRUE(filter->configure(node));
  std::vector<double> out;
  filter->update(out, out);
  for (std::vector<double>::iterator it = out.begin(); it != out.end(); ++it) {
    EXPECT_NEAR(4, *it, epsilon);
  }
  delete filter;
}

TEST(Parameters, StringVector)
{
  auto node = rclcpp::Node::make_shared("TestStringVector");
  filters::FilterBase<std::vector<std::string>> * filter =
    new filters::ParamTest<std::vector<std::string>>();
  EXPECT_TRUE(filter->configure(node));
  std::vector<std::string> out;
  filter->update(out, out);
  for (std::vector<std::string>::iterator it = out.begin(); it != out.end(); ++it) {
    EXPECT_STREQ("four", it->c_str());
  }
  delete filter;
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
