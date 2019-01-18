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

#ifndef FILTERS__TRANSFER_FUNCTION_HPP_
#define FILTERS__TRANSFER_FUNCTION_HPP_

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>
#include <string>
#include <cstring>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <vector>
//  #include "ros/assert.h"
#include "std_msgs/msg/string.hpp"
#include "filters/filter_base.hpp"
#include "filters/realtime_circular_buffer.hpp"

namespace filters
{
/***************************************************/
/*! \class SingleChannelTransferFunctionFilter
    \brief One-dimensional digital filter class.

    This class calculates the output for \f$N\f$ one-dimensional
    digital filters.
    The filter is described by vectors \f$a\f$ and \f$b\f$ and
    implemented using the standard difference equation:<br>

    \f{eqnarray*}
    a[0]*y[n] = b[0]*x[n] &+& b[1]*x[n-1]+ ... + b[n_b]*x[n-n_b]\\
                          &-& a[1]*y[n-1]- ... - a[n_a]*y[n-n_a]
     \f}<br>


    If \f$a[0]\f$ is not equal to 1, the coefficients are normalized by
   \f$a[0]\f$.

    Example xml config:<br>

    <filter type="TransferFunctionFilter" name="filter_name"><br>
        <params a="1.0 0.5" b="0.2 0.2"/><br>
    </filter><br>

*/
/***************************************************/
template<typename T>
class SingleChannelTransferFunctionFilter : public FilterBase<T>
{
public:
  /**
   * \brief Construct the filter
   */
  SingleChannelTransferFunctionFilter();

  /** \brief Destructor to clean up
   */
  ~SingleChannelTransferFunctionFilter();

  /** \brief Configure the filter with the correct number of channels and
   * params. \param number_of_channels The number of inputs filtered. \param
   * config The xml that is parsed to configure the filter.
   */
  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in vector<T> with number_of_channels elements
   * \param data_out vector<T> with number_of_channels elements
   */
  virtual bool update(const T & data_in, T & data_out);

protected:
  boost::scoped_ptr<RealtimeCircularBuffer<T>>
  input_buffer_;     //  The input sample history.
  boost::scoped_ptr<RealtimeCircularBuffer<T>>
  output_buffer_;     //  The output sample history.

  T temp_;   //  used for storage and preallocation

  std::vector<double> a_;   //  Transfer functon coefficients (output).
  std::vector<double> b_;   //  Transfer functon coefficients (input).
  using FilterBase<T>::param_name_; 
  using FilterBase<T>::node_; 
};

template<typename T>
SingleChannelTransferFunctionFilter<T>::SingleChannelTransferFunctionFilter() {}

template<typename T>
SingleChannelTransferFunctionFilter<
  T>::~SingleChannelTransferFunctionFilter() {}

template<typename T>
bool SingleChannelTransferFunctionFilter<T>::configure()
{
  std::string param_name1 = param_name_ + "params.a";
  std::string param_name2 = param_name_ + "params.b";
  //   Parse a and b into a std::vector<double>.
  if (!node_->get_parameter(param_name1, a_)) {
    return false;
  }    //   /\todo check length
  if (!node_->get_parameter(param_name2, b_)) {
    return false;
  }    //   /\todo check length
  //   Create the input and output buffers of the correct size.
  input_buffer_.reset(new RealtimeCircularBuffer<T>(b_.size() - 1, temp_));
  output_buffer_.reset(new RealtimeCircularBuffer<T>(a_.size() - 1, temp_));
  //  Prevent divide by zero while normalizing coeffs.
  if (a_[0] == 0) {
    return false;
  }
  //  Normalize the coeffs by a[0].
  if (a_[0] != 1) {
    for (uint32_t i = 0; i < b_.size(); i++) {
      b_[i] = (b_[i] / a_[0]);
    }
    for (uint32_t i = 1; i < a_.size(); i++) {
      a_[i] = (a_[i] / a_[0]);
    }
    a_[0] = (a_[0] / a_[0]);
  }
  return true;
}

template<typename T>
bool SingleChannelTransferFunctionFilter<T>::update(
  const T & data_in,
  T & data_out)
{
  if (!FilterBase<T>::configured_) {
    return false;
  }
  //  Copy data to prevent mutation if in and out are the same ptr
  temp_ = data_in;

  data_out = b_[0] * temp_;

  for (uint32_t row = 1; row <= input_buffer_->size(); row++) {
    data_out += b_[row] * (*input_buffer_)[row - 1];
  }
  for (uint32_t row = 1; row <= output_buffer_->size(); row++) {
    data_out -= a_[row] * (*output_buffer_)[row - 1];
  }

  input_buffer_->push_front(temp_);
  output_buffer_->push_front(data_out);

  return true;
}

/***************************************************/
/*! \class MultiChannelTransferFunctionFilter
    \brief One-dimensional digital filter class.

    This class calculates the output for \f$N\f$ one-dimensional
    digital filters. Where the input, \f$x\f$, is a (\f$N\f$ x 1) vector
    of inputs and the output, \f$y\f$, is a (\f$N\f$ x 1) vector of outputs.
    The filter is described by vectors \f$a\f$ and \f$b\f$ and
    implemented using the standard difference equation:<br>

    \f{eqnarray*}
    a[0]*y[n] = b[0]*x[n] &+& b[1]*x[n-1]+ ... + b[n_b]*x[n-n_b]\\
                          &-& a[1]*y[n-1]- ... - a[n_a]*y[n-n_a]
     \f}<br>


    If \f$a[0]\f$ is not equal to 1, the coefficients are normalized by
   \f$a[0]\f$.

    Example xml config:<br>

    <filter type="MultiChannelTransferFunctionFilter" name="filter_name"><br>
        <params a="1.0 0.5" b="0.2 0.2"/><br>
    </filter><br>

*/
/***************************************************/

template<typename T>
class MultiChannelTransferFunctionFilter : public MultiChannelFilterBase<T>
{
public:
  /**
   * \brief Construct the filter
   */
  MultiChannelTransferFunctionFilter();

  /** \brief Destructor to clean up
   */
  ~MultiChannelTransferFunctionFilter();

  /** \brief Configure the filter with the correct number of channels and
   * params. \param number_of_channels The number of inputs filtered. \param
   * config The xml that is parsed to configure the filter.
   */
  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in vector<T> with number_of_channels elements
   * \param data_out vector<T> with number_of_channels elements
   */
  virtual bool update(const std::vector<T> & data_in, std::vector<T> & data_out);

protected:
  boost::scoped_ptr<RealtimeCircularBuffer<std::vector<T>>>
  input_buffer_;     //  The input sample history.
  boost::scoped_ptr<RealtimeCircularBuffer<std::vector<T>>>
  output_buffer_;     //  The output sample history.

  std::vector<T> temp_;   //  used for storage and preallocation

  std::vector<double> a_;   //  Transfer functon coefficients (output).
  std::vector<double> b_;   //  Transfer functon coefficients (input).
  
  using MultiChannelFilterBase<T>::param_name_;
  using MultiChannelFilterBase<T>::node_;
};

template<typename T>
MultiChannelTransferFunctionFilter<T>::MultiChannelTransferFunctionFilter() {}

template<typename T>
MultiChannelTransferFunctionFilter<T>::~MultiChannelTransferFunctionFilter() {}

template<typename T>
bool MultiChannelTransferFunctionFilter<T>::configure()
{
  std::string param_name1 = param_name_ + "params.a";
  std::string param_name2 = param_name_ + "params.b";
  //  Parse a and b into a std::vector<double>.
  if (!node_->get_parameter(param_name1, a_)) {
    return false;
  }  //  /\todo check length

  if (!node_->get_parameter(param_name2, b_)) {
    return false;
  }  //  /\todo check length

  //  Create the input and output buffers of the correct size.
  temp_.resize(this->number_of_channels_);
  input_buffer_.reset(
    new RealtimeCircularBuffer<std::vector<T>>(b_.size() - 1, temp_));
  output_buffer_.reset(
    new RealtimeCircularBuffer<std::vector<T>>(a_.size() - 1, temp_));

  //  Prevent divide by zero while normalizing coeffs.
  if (a_[0] == 0) {
    return false;
  }

  //  Normalize the coeffs by a[0].
  if (a_[0] != 1) {
    for (uint32_t i = 0; i < b_.size(); i++) {
      b_[i] = (b_[i] / a_[0]);
    }
    for (uint32_t i = 1; i < a_.size(); i++) {
      a_[i] = (a_[i] / a_[0]);
    }
    a_[0] = (a_[0] / a_[0]);
  }

  return true;
}

template<typename T>
bool MultiChannelTransferFunctionFilter<T>::update(
  const std::vector<T> & data_in, std::vector<T> & data_out)
{
  //  Ensure the correct number of inputs
  if (data_in.size() != this->number_of_channels_ ||
    data_out.size() != this->number_of_channels_)
  {
    return false;
  }
  //  Copy data to prevent mutation if in and out are the same ptr
  temp_ = data_in;

  for (uint32_t i = 0; i < temp_.size(); i++) {
    data_out[i] = b_[0] * temp_[i];

    for (uint32_t row = 1; row <= input_buffer_->size(); row++) {
      (data_out)[i] += b_[row] * (*input_buffer_)[row - 1][i];
    }
    for (uint32_t row = 1; row <= output_buffer_->size(); row++) {
      (data_out)[i] -= a_[row] * (*output_buffer_)[row - 1][i];
    }
  }
  input_buffer_->push_front(temp_);
  output_buffer_->push_front(data_out);
  return true;
}

}  //  namespace filters

#endif  // FILTERS__TRANSFER_FUNCTION_HPP_
