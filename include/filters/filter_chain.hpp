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

#ifndef FILTERS__FILTER_CHAIN_HPP_
#define FILTERS__FILTER_CHAIN_HPP_

//   #include "ros/ros.h"
#include <sstream>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include "boost/shared_ptr.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "filters/realtime_circular_buffer.hpp"
#include "filters/filter_base.hpp"

namespace filters
{

/** \brief A class which will construct and sequentially call Filters according
 * to xml This is the primary way in which users are expected to interact with
 * Filters
 */
template<typename T>
class FilterChain
{
private:
  pluginlib::ClassLoader<filters::FilterBase<T>> loader_;

public:
  /** \brief Create the filter chain object */
  explicit FilterChain(std::string data_type)
  : loader_("filters", std::string("filters::FilterBase<") + data_type +
      std::string(">")),
    configured_(false)
  {
    std::string lib_string = "";
    std::vector<std::string> libs = loader_.getDeclaredClasses();
    // cout<<"libs.size() = "<<libs.size()<<endl;
    for (unsigned int i = 0; i < libs.size(); i++) {
      // cout<<"lib[i] = "<<libs[i]<<endl;
      lib_string = lib_string + std::string(", ") + libs[i];
    }
  }

  ~FilterChain() {clear();}

  /**@brief Configure the filter chain from a configuration stored on the
   * parameter server
   * @param param_name The name of the filter chain to load
   * @param node The node handle to use if a different namespace is required
   */
  bool configure(std::string param_name, rclcpp::Node::SharedPtr node)
  // bool configure(std::string param_name, ros::NodeHandle node =
  // ros::NodeHandle())
  {
    auto parameters_and_prefixes = node->list_parameters({}, 10);

    for (auto & name : parameters_and_prefixes.names) {
      for (auto & parameter : node->get_parameters({name})) {
        ss1 << "\nParameter name: " << parameter.get_name();
        ss1 << "\nParameter data_type: " << parameter.get_type();
        ss1 << "\nParameter value (" << parameter.get_type_name() <<
          "): " << parameter.value_to_string();
        filter_param[parameter.get_name()] = parameter.value_to_string();
      }
    }

    for (std::map<std::string, std::string>::iterator filter_it = filter_param.begin();
      filter_it != filter_param.end(); ++filter_it)
    {
      std::string filter_name = filter_it->first;
      std::string filter_type = filter_it->second;
      if (std::string::npos != filter_name.find("params")) {
        std::string p_name = filter_name;
        int pos1 = p_name.find(".");
        std::string name = p_name.substr(pos1 + 1);
        p_name.erase(pos1 + 1, filter_name.length() + 1);
        param_name1 = p_name;
      } else if (std::string::npos != filter_name.find("type")) {
        // try
        if (loader_.isClassAvailable(filter_type)) {
          bool have_class = false;
          bool result = true;

          std::vector<std::string> classes = loader_.getDeclaredClasses();
          for (unsigned int i = 0; i < classes.size(); ++i) {
          }
          for (unsigned int i = 0; i < classes.size(); ++i) {
            if (filter_type == classes[i]) {
              // if we've found a match... we'll get the fully qualified name
              // and break out of the loop
              filter_type = classes[i];
              have_class = true;
              // break;
              std::shared_ptr<filters::FilterBase<T>> p =
                loader_.createSharedInstance(filter_type);
              if (p.get() == NULL) {
                return false;
              }

              result = result && p.get()->configure(param_name1, node);
              reference_pointers_.push_back(p);

              unsigned int list_size = reference_pointers_.size();
            }
          }
          if (!have_class) {
            RCLCPP_DEBUG(node->get_logger(), "\nUnable to find filter class %s. Check that filter "
              "is fully declared.",
              filter_type.c_str());
            continue;
          }
        }
      }
    }

    if (result == true) {
      configured_ = true;
    }
    return result;
  }

  /** \brief process data through each of the filters added sequentially */
  bool update(const T & data_in, T & data_out)
  {
    unsigned int list_size = reference_pointers_.size();
    bool result;
    if (list_size == 0) {
      data_out = data_in;
      result = true;
    } else if (list_size == 1) {
      result = reference_pointers_[0]->update(data_in, data_out);
    } else if (list_size == 2) {
      result = reference_pointers_[0]->update(data_in, buffer0_);
      if (result == false) {
        return false;
      }  // don't keep processing on failure
      result = result && reference_pointers_[1]->update(buffer0_, data_out);
    } else {
      result =
        reference_pointers_[0]->update(data_in, buffer0_);   // first copy in
      for (unsigned int i = 1; i < reference_pointers_.size() - 1;
        i++)    // all but first and last (never called if size=2)
      {
        if (i % 2 == 1) {
          result = result && reference_pointers_[i]->update(buffer0_, buffer1_);
        } else {
          result = result && reference_pointers_[i]->update(buffer1_, buffer0_);
        }

        if (result == false) {
          return false;
        }  // don't keep processing on failure
      }
      if (list_size % 2 == 1) {      //   odd number last deposit was in buffer1
        result =
          result && reference_pointers_.back()->update(buffer1_, data_out);
      } else {
        result =
          result && reference_pointers_.back()->update(buffer0_, data_out);
      }
    }
    return result;
  }
  /** \brief Clear all filters from this chain */
  bool clear()
  {
    configured_ = false;
    reference_pointers_.clear();
    return true;
  }

private:
  std::vector<std::shared_ptr<filters::FilterBase<T>>>
  reference_pointers_;     //   /<! A vector of pointers to currently constructed
  //  /<filters
  unsigned int i = 0;
  bool result = true;

  rclcpp::Parameter parameter_test;
  std::map<std::string, std::string> filter_param;
  std::string param_name1;
  std::stringstream ss1;
  std::vector<std::string> classes = loader_.getDeclaredClasses();
  T buffer0_;         //   <! A temporary intermediate buffer
  T buffer1_;         //   <! A temporary intermediate buffer
  bool configured_;   //   <! whether the system is configured
};

template<typename T>
class MultiChannelFilterChain
{
private:
  pluginlib::ClassLoader<filters::MultiChannelFilterBase<T>> loader_;

public:
  /** \brief Create the filter chain object */
  explicit MultiChannelFilterChain(std::string data_type)
  : loader_("filters", std::string("filters::MultiChannelFilterBase<") +
      data_type + std::string(">")),
    configured_(false)
  {
    std::string lib_string = "";
    std::vector<std::string> libs = loader_.getDeclaredClasses();
    for (unsigned int i = 0; i < libs.size(); i++) {
      lib_string = lib_string + std::string(", ") + libs[i];
    }
  }

  /**@brief Configure the filter chain from a configuration stored on the
   * parameter server
   * @param param_name The name of the filter chain to load
   * @param node The node handle to use if a different namespace is required
   */
  bool configure(
    unsigned int number_of_channels,
    rclcpp::Node::SharedPtr node)
  {
    number_of_channels_ = number_of_channels;

    auto parameters_and_prefixes = node->list_parameters({}, 10);
    for (auto & name : parameters_and_prefixes.names) {
      for (auto & parameter : node->get_parameters({name})) {
        ss1 << "\nParameter name: " << parameter.get_name();
        ss1 << "\nParameter data_type: " << parameter.get_type();
        ss1 << "\nParameter value (" << parameter.get_type_name() <<
          "): " << parameter.value_to_string();
        filter_param[parameter.get_name()] = parameter.value_to_string();
      }
    }
    for (std::map<std::string, std::string>::iterator filter_it = filter_param.begin();
      filter_it != filter_param.end(); ++filter_it)
    {
      std::string filter_name = filter_it->first;
      std::string filter_type = filter_it->second;
      if (std::string::npos != filter_name.find("params")) {
        std::string p_name = filter_name;
        int pos1 = p_name.find(".");
        std::string name = p_name.substr(pos1 + 1);
        p_name.erase(pos1 + 1, filter_name.length() + 1);
        param_name2 = p_name;
      } else if (std::string::npos != filter_name.find("type")) {
        // try{
        if (loader_.isClassAvailable(filter_type)) {
          bool have_class = false;
          bool result = true;
          buffer0_.resize(number_of_channels);
          buffer1_.resize(number_of_channels);
          std::vector<std::string> classes = loader_.getDeclaredClasses();
          for (unsigned int i = 0; i < classes.size(); ++i) {
            if (filter_type == classes[i]) {
              // if we've found a match... we'll get the fully qualified name
              filter_type = classes[i];
              have_class = true;
              std::shared_ptr<filters::MultiChannelFilterBase<T>> p =
                loader_.createSharedInstance(filter_type);
              if (p.get() == NULL) {
                return false;
              }
              result = result && p.get()->configure(number_of_channels,
                  param_name2, node);
              reference_pointers_.push_back(p);

              unsigned int list_size = reference_pointers_.size();
            }
          }

          if (!have_class) {
            RCLCPP_ERROR(node->get_logger(), "\nUnable to find filter class %s. Check that filter "
              "is fully declared.", filter_type.c_str());
            continue;
          }
        }
      }
    }

    if (result == true) {
      configured_ = true;
    }
    return result;
  }
  /** \brief process data through each of the filters added sequentially */
  bool update(const std::vector<T> & data_in, std::vector<T> & data_out)
  {
    unsigned int list_size = reference_pointers_.size();
    bool result;
    if (list_size == 0) {
      data_out = data_in;
      result = true;
    } else if (list_size == 1) {
      result = reference_pointers_[0]->update(data_in, data_out);
    } else if (list_size == 2) {
      result = reference_pointers_[0]->update(data_in, buffer0_);
      if (result == false) {
        return false;
      }  //  don't keep processing on failure
      result = result && reference_pointers_[1]->update(buffer0_, data_out);
    } else {
      result =
        reference_pointers_[0]->update(data_in, buffer0_);   //  first copy in
      for (unsigned int i = 1; i < reference_pointers_.size() - 1;
        i++)    //  all but first and last (never if size = 2)
      {
        if (i % 2 == 1) {
          result = result && reference_pointers_[i]->update(buffer0_, buffer1_);
        } else {
          result = result && reference_pointers_[i]->update(buffer1_, buffer0_);
        }
        if (result == false) {
          return false;
        }  //  don't keep processing on failure
      }
      if (list_size % 2 == 1) {  // odd number last deposit was in buffer1
        result =
          result && reference_pointers_.back()->update(buffer1_, data_out);
      } else {
        result =
          result && reference_pointers_.back()->update(buffer0_, data_out);
      }
    }
    return result;
  }

  ~MultiChannelFilterChain() {clear();}
  /** \brief Clear all filters from this chain */
  bool clear()
  {
    configured_ = false;
    reference_pointers_.clear();
    buffer0_.clear();
    buffer1_.clear();
    return true;
  }

private:
  bool result = true;
  std::vector<std::shared_ptr<filters::MultiChannelFilterBase<T>>>
  reference_pointers_;     //  /<! A vector of pointers to currently constructed
  //  /<filters
  rclcpp::Parameter parameter_test;
  unsigned int i = 0;
  std::map<std::string, std::string> filter_param;
  std::string param_name2;
  std::stringstream ss1;
  std::vector<std::string> classes;
  std::vector<T> buffer0_;  //  /<! A temporary intermediate buffer
  std::vector<T> buffer1_;  //  /<! A temporary intermediate buffer
  bool configured_;         //  /<! whether the system is configured
  int number_of_channels_;
};
}       //  namespace filters
#endif  // FILTERS__FILTER_CHAIN_HPP_
