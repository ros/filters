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

#ifndef FILTERS__MEDIAN_HPP_
#define FILTERS__MEDIAN_HPP_
#include <stdint.h>
#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <string>
//  #include "ros/assert.h"
#include <cstring>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include "std_msgs/msg/string.hpp"
#include "filters/filter_base.hpp"
#include "filters/realtime_circular_buffer.hpp"

/*********************************************************************/
/*
 * Algorithm from N. Wirth's book, implementation by N. Devillard.
 * This code in public domain.
 */
#define ELEM_SWAP(a, b) \
  { \
    register elem_type t = (a); \
    (a) = (b); \
    (b) = t; \
  }
namespace filters
{
/*---------------------------------------------------------------------------
  Function : kth_smallest()
  In : array of elements, # of elements in the array, rank k
  Out : one element
  Job : find the kth smallest element in the array
  Notice : use the median() macro defined below to get the median.
  Reference:
  Author: Wirth, Niklaus
  Title: Algorithms + data structures = programs
  Publisher: Englewood Cliffs: Prentice-Hall, 1976
  Physical description: 366 p.
  Series: Prentice-Hall Series in Automatic Computation
  ---------------------------------------------------------------------------*/
template<typename elem_type>
elem_type kth_smallest(elem_type a[], int n, int k)
{
  register int i, j, l, m;
  register elem_type x;
  l = 0;
  m = n - 1;
  while (l < m) {
    x = a[k];
    i = l;
    j = m;
    do {
      while (a[i] < x) {
        i++;
      }
      while (x < a[j]) {
        j--;
      }
      if (i <= j) {
        ELEM_SWAP(a[i], a[j]);
        i++;
        j--;
      }
    } while (i <= j);
    if (j < k) {
      l = i;
    }
    if (k < i) {
      m = j;
    }
  }
  return a[k];
}
#define median(a, n) kth_smallest(a, n, (((n) & 1) ? ((n) / 2) : (((n) / 2) - 1)))
#undef ELEM_SWAP

/** \brief A median filter which works on arrays.
 *
 */
template<typename T>
class MedianFilter : public FilterBase<T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  MedianFilter();

  /** \brief Destructor to clean up
   */
  ~MedianFilter();
  virtual bool get_configure(
    const std::string & param_name,
    rclcpp::Node::SharedPtr node);
  /** \brief Update the filter and return the data seperately
   * \param data_in double array with length and width
   * \param data_out double array with length and width
   */
  virtual bool update(const T & data_in, T & data_out);

protected:
  std::vector<T> temp_storage_;    //  < Preallocated storage for the list to sort
  boost::scoped_ptr<RealtimeCircularBuffer<T>>
  data_storage_;     //   < Storage for data between updates
  T temp_;     //   used for preallocation and copying from non vector source
  uint32_t
    number_of_observations_;   //  < Number of observations over which to filter
};
template<typename T>
MedianFilter<T>::MedianFilter()
: number_of_observations_(0)
{
}
template<typename T>
MedianFilter<T>::~MedianFilter() {}
template<typename T>
bool MedianFilter<T>::get_configure(
  const std::string & param_name,
  rclcpp::Node::SharedPtr node)
{
  std::string param_name1 = param_name + "params.number_of_observations";
  int no_obs = -1;
  if (!node->get_parameter(param_name1, number_of_observations_)) {
    return false;
  }
  data_storage_.reset(
    new RealtimeCircularBuffer<T>(number_of_observations_, temp_));
  temp_storage_.resize(number_of_observations_);
  return true;
  number_of_observations_ = no_obs;
}
template<typename T>
bool MedianFilter<T>::update(const T & data_in, T & data_out)
{
  if (!FilterBase<T>::configured_) {
    return false;
  }
  data_storage_->push_back(data_in);
  unsigned int length = data_storage_->size();
  for (uint32_t row = 0; row < length; row++) {
    temp_storage_[row] = (*data_storage_)[row];
  }
  data_out = median(&temp_storage_[0], length);
  return true;
}
/** \brief A median filter which works on arrays.
 *
 */
template<typename T>
class MultiChannelMedianFilter : public MultiChannelFilterBase<T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  MultiChannelMedianFilter();
  /** \brief Destructor to clean up
   */
  ~MultiChannelMedianFilter();
  virtual bool get_configure(
    const std::string & param_name,
    rclcpp::Node::SharedPtr node);
  /** \brief Update the filter and return the data seperately
   * \param data_in double array with length width
   * \param data_out double array with length width
   */
  virtual bool update(const std::vector<T> & data_in, std::vector<T> & data_out);

protected:
  std::vector<T> temp_storage_;   //  < Preallocated storage for the list to sort
  boost::scoped_ptr<RealtimeCircularBuffer<std::vector<T>>>
  data_storage_;     //  < Storage for data between updates
  std::vector<T>
  temp;     //  used for preallocation and copying from non vector source
  uint32_t
    number_of_observations_;   //  < Number of observations over which to filter
};
template<typename T>
MultiChannelMedianFilter<T>::MultiChannelMedianFilter()
: number_of_observations_(0)
{
}

template<typename T>
MultiChannelMedianFilter<T>::~MultiChannelMedianFilter() {}
template<typename T>
bool MultiChannelMedianFilter<T>::get_configure(
  const std::string & param_name,
  rclcpp::Node::SharedPtr node)
{
  std::string param_name1 = param_name + "params.number_of_observations";
  if (!node->get_parameter(param_name1, number_of_observations_)) {
    return false;
  }
  temp.resize(this->number_of_channels_);
  data_storage_.reset(new RealtimeCircularBuffer<std::vector<T>>(
      number_of_observations_, temp));
  temp_storage_.resize(number_of_observations_);
  return true;
}
template<typename T>
bool MultiChannelMedianFilter<T>::update(
  const std::vector<T> & data_in,
  std::vector<T> & data_out)
{
  if (data_in.size() != this->number_of_channels_ ||
    data_out.size() != this->number_of_channels_)
  {
    return false;
  }
  data_storage_->push_back(data_in);
  unsigned int length = data_storage_->size();
  for (uint32_t i = 0; i < this->number_of_channels_; i++) {
    for (uint32_t row = 0; row < length; row++) {
      temp_storage_[row] = (*data_storage_)[row][i];
    }
    data_out[i] = median(&temp_storage_[0], length);
  }
  //  cerr << "data_out computed in median:" << data_out << endl;
  return true;
}
}   //  namespace filters
#endif  // FILTERS__MEDIAN_HPP_
