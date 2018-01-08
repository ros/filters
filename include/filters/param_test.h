/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FILTERS_PARAM_TEST
#define FILTERS_PARAM_TEST

#include <stdint.h>
#include <cstring>
#include <stdio.h>

#include "filters/filter_base.h"


namespace filters
{

/** \brief A mean filter which works on doubles.
 *
 */
template <typename T>
class ParamTest: public FilterBase <T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  ParamTest();

  /** \brief Destructor to clean up
   */
  ~ParamTest();

  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update( const T & data_in, T& data_out);
  
protected:
  
};


template <typename T>
ParamTest<T>::ParamTest()
{
}

template <typename T>
bool ParamTest<T>::configure()
{
  return true;
}

template <typename T>
ParamTest<T>::~ParamTest()
{
}


template <typename T>
bool ParamTest<T>::update(const T & data_in, T& data_out)
{
  T temp;
  this->getParam("key", temp);
  data_out = temp;
  return true;
};

}
#endif
