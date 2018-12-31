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
#ifndef FILTERS__FILTER_BASE_HPP_
#define FILTERS__FILTER_BASE_HPP_

#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <string>
#include <typeinfo>
#include <vector>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#define ROS_DEBUG printf
#define ROS_FATAL printf
#define ROS_WARN printf
#define ROS_INFO printf
#define ROS_ASSERT_MSG printf
#define ROS_ERROR printf

//  using namespace std;
namespace filters
{
/** \brief A Base filter class to provide a standard interface for all filters
filter_b *
 */
template<typename T>
class FilterBase
{
public:
  /** \brief Default constructor used by Filter Factories
   */
  FilterBase()
  : configured_(false) {}

  /** \brief Virtual Destructor
   */
  virtual ~FilterBase() {}
  /** \brief Configure the filter from the parameter server
   * \param The parameter from which to read the configuration
   * \param node_handle The optional node handle, useful if operating in a
   * different namespace.
   */
  bool configure(const std::string & param_name, rclcpp::Node::SharedPtr node)
  {
    configured_ = get_configure(param_name, node);
    return configured_;
  }
  bool configure(rclcpp::Node::SharedPtr node)
  {
    if (configured_) {
      RCLCPP_DEBUG(node->get_logger(), "Filter type already being reconfigured ");
    }
    configured_ = false;
    parameters_and_prefixes = node->list_parameters({}, 10);
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
      uint32_t number_of_observations_;
      if (std::string::npos != filter_name.find("params")) {
        std::string p_name = filter_name;
        int pos1 = p_name.find(".");
        std::string name = p_name.substr(pos1 + 1);
        p_name.erase(pos1 + 1, filter_name.length() + 1);
        param_name = p_name;
      }

      configured_ = get_configure(param_name, node);
    }

    configured_ = get_configure(param_name, node);
    return configured_;
  }
  virtual bool get_configure(
    const std::string & param_name,
    rclcpp::Node::SharedPtr node) = 0;
  /** \brief Update the filter and return the data seperately
   * This is an inefficient way to do this and can be overridden in the derived
   * class \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   */
  virtual bool update(const T & data_in, T & data_out) = 0;

protected:
  bool configured_;
  rcl_interfaces::msg::ListParametersResult parameters_and_prefixes;
  std::map<std::string, std::string> filter_param;
  std::stringstream ss1;
  std::string param_name;
  int cnt = 0;
  /** \brief Pure virtual function for the sub class to configure the filter
   * This function must be implemented in the derived class.
   */
};
template<typename T>
class MultiChannelFilterBase : public FilterBase<T>
{
public:
  MultiChannelFilterBase()
  : number_of_channels_(0) {}
  /** \brief Configure the filter from the parameter server
   * \param number_of_channels How many parallel channels the filter will
   * process \param The parameter from which to read the configuration \param
   * node_handle The optional node handle, useful if operating in a different
   * namespace.
   */
  bool configure(
    unsigned int number_of_channels, const std::string & param_name,
    rclcpp::Node::SharedPtr node)
  {
    if (configured_) {
      RCLCPP_DEBUG(node->get_logger(), "Filter type already being reconfigured");
    }
    configured_ = false;
    number_of_channels_ = number_of_channels;
    configured_ = get_configure(param_name, node);
    return configured_;
  }
  bool configure(
    unsigned int number_of_channels,
    rclcpp::Node::SharedPtr node)
  {
    if (configured_) {
      RCLCPP_DEBUG(node->get_logger(), "Filter type already being reconfigured ");
    }
    configured_ = false;
    number_of_channels_ = number_of_channels;
    parameters_and_prefixes = node->list_parameters({}, 10);
    for (auto & name : parameters_and_prefixes.names) {
      for (auto & parameter : node->get_parameters({name})) {
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
        param_name = p_name;
      }
    }
    configured_ = get_configure(param_name, node);
    return configured_;
  }
  virtual bool get_configure(
    const std::string & param_name,
    rclcpp::Node::SharedPtr node) = 0;
  /** \brief Update the filter and return the data seperately
   * \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   * This funciton must be implemented in the derived class.
   */
  virtual bool update(
    const std::vector<T> & data_in,
    std::vector<T> & data_out) = 0;
  virtual bool update(const T & data_in, T & data_out)
  {
  }

protected:
  using FilterBase<T>::configured_;
  //  How many parallel inputs for which the filter is to be configured
  unsigned int number_of_channels_;
  std::stringstream ss1;
  std::string param_name;
  std::map<std::string, std::string> filter_param;
  //  Store the list of parameters specific to node
  rcl_interfaces::msg::ListParametersResult parameters_and_prefixes;
};

}   //  namespace filters
#endif  // FILTERS__FILTER_BASE_HPP_
