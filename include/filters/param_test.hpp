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

#ifndef FILTERS__PARAM_TEST_HPP_
#define FILTERS__PARAM_TEST_HPP_

#include <stdint.h>
#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include "std_msgs/msg/string.hpp"
#include "filters/filter_base.hpp"
namespace filters
{

/** \brief A mean filter which works on doubles.
 *
 */
template<typename T>
class ParamTest : public FilterBase<T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  ParamTest();

  /** \brief Destructor to clean up
   */
  ~ParamTest();
  virtual bool get_configure(
    const std::string & param_name,
    rclcpp::Node::SharedPtr node);

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const T & data_in, T & data_out);

protected:
  T temp_;
};
template<typename T>
ParamTest<T>::ParamTest() {}
template<typename T>
bool ParamTest<T>::get_configure(
  const std::string & param_name,
  rclcpp::Node::SharedPtr node)
{
  std::string param_name1 = param_name + "params.key";
  if (!node->get_parameter(param_name1, temp_)) {
    return false;
  }
  return true;
}
template<typename T>
ParamTest<T>::~ParamTest() {}
template<typename T>
bool ParamTest<T>::update(const T & data_in, T & data_out)
{
  data_out = temp_;
  return true;
}
}  //  namespace filters
#endif  // FILTERS__PARAM_TEST_HPP_
