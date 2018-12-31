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

#ifndef FILTERS__REALTIME_CIRCULAR_BUFFER_HPP_
#define FILTERS__REALTIME_CIRCULAR_BUFFER_HPP_

#include <stdint.h>
#include <boost/circular_buffer.hpp>
#include <vector>
#include <iostream>
#include <iterator>
#include <map>
#include <algorithm>
#include <string>
#include "std_msgs/msg/string.hpp"

  //   #include <functional>
namespace filters
{

/** \brief A realtime safe circular (ring) buffer.
 */
template<typename T>
class RealtimeCircularBuffer
{
private:
  RealtimeCircularBuffer();

public:
  RealtimeCircularBuffer(int size, const T & default_val)
  : counter_(0), cb_(size)
  {
    for (unsigned int i = 0; i < cb_.capacity(); i++) {
      cb_.push_back(default_val);
    }
  }

  void push_back(const T & item)
  {
    if (cb_.capacity() == 0) {return;}

    if (counter_ < cb_.size()) {
      cb_[counter_] = item;
    } else {
      cb_.push_back(item);
    }
    counter_++;
  }

  void push_front(const T & item)
  {
    if (cb_.capacity() == 0) {return;}
    cb_.push_front(item);
    counter_++;
  }

  void clear() {counter_ = 0;}

  void set_capacity(unsigned int order, const T & value);

  T & front() {return cb_.front();}
  T & back()
  {
    if (counter_ < cb_.size()) {
      return cb_[counter_];
    } else {
      return cb_.back();
    }
  }

  unsigned int size() {return std::min(counter_, (unsigned int)cb_.size());}
  bool empty() {return cb_.empty();}
  T & at(size_t index) {return cb_.at(index);}
  T & operator[](size_t index) {return cb_[index];}

private:
  unsigned int counter_;     //    <! special counter to keep track of first N times through
  boost::circular_buffer<T> cb_;
};
}    //   namespace filters
#endif  // FILTERS__REALTIME_CIRCULAR_BUFFER_HPP_
