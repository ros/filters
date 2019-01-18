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

#ifndef FILTERS__MEAN_HPP_
#define FILTERS__MEAN_HPP_

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
#include "filters/realtime_circular_buffer.hpp"
namespace filters
{

/** \brief A mean filter which works on doubles.
 *
 */
template<typename T>
class MeanFilter : public FilterBase<T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  MeanFilter();
  /** \brief Destructor to clean up
   */
  ~MeanFilter();

  virtual bool configure();
 
  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const T & data_in, T & data_out);

protected:
  boost::scoped_ptr<RealtimeCircularBuffer<T>>
  data_storage_;     //  < Storage for data between updates
  uint32_t
    last_updated_row_;  //  < The last row to have been updated by the filter
  T temp_;               //  Temporary storage
  uint32_t number_of_observations_;  //  < Number of observations over which to filter
    using FilterBase<T>::param_name_; 
    using FilterBase<T>::node_; 
};
rclcpp::Parameter parameter;
template<typename T>
MeanFilter<T>::MeanFilter()
:number_of_observations_(0) {}

template<typename T>

bool MeanFilter<T>::configure()
{
  std::string param_name1 = FilterBase<T>::param_name_ + "params.number_of_observations";

  if (!FilterBase<T>::node_->get_parameter(param_name1, number_of_observations_)) {
    return false;
  }
  data_storage_.reset(
    new RealtimeCircularBuffer<T>(number_of_observations_, temp_));
  return true;
}
template<typename T>
MeanFilter<T>::~MeanFilter() {}
template<typename T>
bool MeanFilter<T>::update(const T & data_in, T & data_out)
{
  if (last_updated_row_ >= number_of_observations_ - 1) {
    last_updated_row_ = 0;
  } else {
    last_updated_row_++;
  }
  data_storage_->push_back(data_in);
  unsigned int length = data_storage_->size();
  data_out = 0;
  for (uint32_t row = 0; row < length; row++) {
    data_out += data_storage_->at(row);
  }
  data_out /= length;
  return true;
}

/** \brief A mean filter which works on double arrays.
 *
 */
template<typename T>
class MultiChannelMeanFilter : public MultiChannelFilterBase<T>
{
public:
  /** \brief Construct the filter with the expected width and height */
 // MultiChannelMeanFilter(unsigned int number_of_channels,const std::string &  param_name,rclcpp::Node::SharedPtr node);
  MultiChannelMeanFilter();
  /** \brief Destructor to clean up
   */
  ~MultiChannelMeanFilter();
  
  virtual bool configure();
  
  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const std::vector<T> & data_in, std::vector<T> & data_out);

protected:
  boost::scoped_ptr<RealtimeCircularBuffer<std::vector<T>>>
  data_storage_;     //  < Storage for data between updates
  uint32_t
    last_updated_row_;   //  < The last row to have been updated by the filter

  std::vector<T> temp;     //  used for preallocation and copying from non vector source

  uint32_t number_of_observations_;   //  < Number of observations over which to filter
  using MultiChannelFilterBase<T>::number_of_channels_;  //  < Number of elements
                                                         //  per observation
  using MultiChannelFilterBase<T>::param_name_;
  using MultiChannelFilterBase<T>::node_;
  
};

template<typename T>

MultiChannelMeanFilter<T>::MultiChannelMeanFilter()
: number_of_observations_(0) {}

template<typename T>

bool MultiChannelMeanFilter<T>::configure()
{
  std::string param_name1 = MultiChannelFilterBase<T>::param_name_ + "params.number_of_observations";
  if (!MultiChannelFilterBase<T>::node_->get_parameter(param_name1, number_of_observations_)) {
    return false;
  }
  temp.resize(number_of_channels_);
  data_storage_.reset(new RealtimeCircularBuffer<std::vector<T>>(
      number_of_observations_, temp));
  return true;
}

template<typename T>
MultiChannelMeanFilter<T>::~MultiChannelMeanFilter() {}

template<typename T>
bool MultiChannelMeanFilter<T>::update(
  const std::vector<T> & data_in,
  std::vector<T> & data_out)
{
  if (data_in.size() != number_of_channels_ ||
    data_out.size() != number_of_channels_)
  {
    return false;
  }
  //  update active row
  if (last_updated_row_ >= number_of_observations_ - 1) {
    last_updated_row_ = 0;
  } else {
    last_updated_row_++;
  }
  data_storage_->push_back(data_in);
  unsigned int length = data_storage_->size();
  //  Return each value
  for (uint32_t i = 0; i < number_of_channels_; i++) {
    data_out[i] = 0;
    for (uint32_t row = 0; row < length; row++) {
      data_out[i] += data_storage_->at(row)[i];
    }
    data_out[i] /= length;
  }
  return true;
}
}  //  namespace filters
#endif  // FILTERS__MEAN_HPP_
