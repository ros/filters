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

#ifndef FILTERS__INCREMENT_HPP_
#define FILTERS__INCREMENT_HPP_
#include <stdint.h>
#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>
#include <cstring>
#include <iostream>
#include <iterator>
#include <vector>
#include <map>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "filters/filter_base.hpp"


namespace filters
{
/** \brief A increment filter which works on doubles.
 *
 */
template<typename T>
class IncrementFilter : public FilterBase<T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  IncrementFilter();
  /** \brief Destructor to clean up
   */
  ~IncrementFilter();
  virtual bool configure();
  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const T & data_in, T & data_out);
};
template<typename T>
IncrementFilter<T>::IncrementFilter()
{
}
template<typename T>
bool IncrementFilter<T>::configure()
{
  return true;
}
template<typename T>
IncrementFilter<T>::~IncrementFilter()
{
}
template<typename T>
bool IncrementFilter<T>::update(const T & data_in, T & data_out)
{
  data_out = data_in + 1;

  return true;
}
/** \brief A increment filter which works on arrays.
 *
 */
template<typename T>
class MultiChannelIncrementFilter : public MultiChannelFilterBase<T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  MultiChannelIncrementFilter();

  /** \brief Destructor to clean up
   */
  ~MultiChannelIncrementFilter();
  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const std::vector<T> & data_in, std::vector<T> & data_out);

protected:
  using MultiChannelFilterBase<T>::number_of_channels_;  //< Number of elements per observation
  //  using MultiChannelFilterBase<T>::param_name_;
  using MultiChannelFilterBase<T>::node_;
};
template<typename T>
MultiChannelIncrementFilter<T>::MultiChannelIncrementFilter()
{
}
template<typename T>
bool MultiChannelIncrementFilter<T>::configure()
{
  return true;
}
template<typename T>
MultiChannelIncrementFilter<T>::~MultiChannelIncrementFilter()
{
}
template<typename T>
bool MultiChannelIncrementFilter<T>::update(
  const std::vector<T> & data_in,
  std::vector<T> & data_out)
{
  if (data_in.size() != number_of_channels_ || data_out.size() != number_of_channels_) {
//  RCLCPP_DEBUG(node_->get_logger(), "IncrementFilter configured with wrong size config\n ");
    return false;
  }
  //  Return each value
  for (uint32_t i = 0; i < number_of_channels_; i++) {
    data_out[i] = data_in[i] + 1;
  }
  return true;
}
}  // namespace filters
#endif  // FILTERS__INCREMENT_HPP_
