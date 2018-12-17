/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef FILTERS_FILTER_CHAIN_H_
#define FILTERS_FILTER_CHAIN_H_

//#include "ros/ros.h"
#include <rclcpp/rclcpp.hpp>
#include "filters/filter_base.h"
#include "pluginlib/class_loader.hpp"
#include <sstream>
#include <vector>
#include "boost/shared_ptr.hpp"

namespace filters
{

/** \brief A class which will construct and sequentially call Filters according to xml
 * This is the primary way in which users are expected to interact with Filters
 */
template <typename T>
class FilterChain
{
private:
  pluginlib::ClassLoader<filters::FilterBase<T> > loader_;
public:
  /** \brief Create the filter chain object */
  FilterChain(std::string data_type): loader_("filters", std::string("filters::FilterBase<") + data_type + std::string(">")), configured_(false)
  {
    std::string lib_string = "";
    std::vector<std::string> libs = loader_.getDeclaredClasses();
    //cout<<"libs.size() = "<<libs.size()<<endl;
    for (unsigned int i = 0 ; i < libs.size(); i ++)
    {
    	//cout<<"lib[i] = "<<libs[i]<<endl;
    	lib_string = lib_string + std::string(", ") + libs[i];
    }    
    ROS_DEBUG("In FilterChain ClassLoader found the following libs: %s", lib_string.c_str());
  };

  ~FilterChain()
  {
    clear();

  };

  /**@brief Configure the filter chain from a configuration stored on the parameter server
   * @param param_name The name of the filter chain to load
   * @param node The node handle to use if a different namespace is required
   */
  bool configure(std::string param_name, rclcpp::Node::SharedPtr node)
  //bool configure(std::string param_name, ros::NodeHandle node = ros::NodeHandle())
  {

    rclcpp::Parameter parameter;
   	if(node->get_parameter(param_name + "/filter_chain", parameter))
    
    {
   		cout<<"Rohit IF"<<endl;
      
    }
    else if(node->get_parameter(param_name, parameter))
    
    {
      ROS_DEBUG("Could not load the filter chain configuration from parameter %s, are you sure it was pushed to the parameter server? Assuming that you meant to leave it empty.", param_name.c_str());
      configured_ = true;
      return true;
    }

   	int IncrementsCounter = 0;
   	node->get_parameter("Increments_counter", IncrementsCounter);
   	cout<<"IncrementsCounter = "<<IncrementsCounter<<endl;

    int return_flag = 0;
   	if(IncrementsCounter > 0)
   	{
   		for (unsigned int i = 0 ; i < IncrementsCounter; i ++)
   			return_flag = this->configure(parameter, node->get_namespace(),node);
   	}
   	else
   		return_flag = this->configure(parameter, node->get_namespace(),node);
   	return return_flag;
  }

  /** \brief process data through each of the filters added sequentially */
  bool update(const T& data_in, T& data_out)
  {
    unsigned int list_size = reference_pointers_.size();
    cout<<"update called list_size = "<<list_size<<endl;
    bool result;
    if (list_size == 0)
    {
      data_out = data_in;
      result = true;
    }
    else if (list_size == 1)
      result = reference_pointers_[0]->update(data_in, data_out);
    else if (list_size == 2)
    {
      result = reference_pointers_[0]->update(data_in, buffer0_);
      if (result == false) {return false; };//don't keep processing on failure
      result = result && reference_pointers_[1]->update(buffer0_, data_out);
    }
    else
    {
      result = reference_pointers_[0]->update(data_in, buffer0_);  //first copy in
      for (unsigned int i = 1; i <  reference_pointers_.size() - 1; i++) // all but first and last (never called if size=2)
      {
        if (i %2 == 1)
          result = result && reference_pointers_[i]->update(buffer0_, buffer1_);
        else
          result = result && reference_pointers_[i]->update(buffer1_, buffer0_);
        
        if (result == false) {return false; }; //don't keep processing on failure
      }
      if (list_size % 2 == 1) // odd number last deposit was in buffer1
        result = result && reference_pointers_.back()->update(buffer1_, data_out);
      else
        result = result && reference_pointers_.back()->update(buffer0_, data_out);
    }
    return result;
    
  };
  /** \brief Clear all filters from this chain */
  bool clear() 
  {
    configured_ = false;
    reference_pointers_.clear();
    return true;
  };
  

  /** \brief Configure the filter chain
    * This will call configure on all filters which have been added */
  bool configure(  rclcpp::Parameter &parameter, const std::string& filter_ns, rclcpp::Node::SharedPtr node)
  {
	  std::string FilterType;
	  std::string FilterName;

	  if(!node->get_parameter("type", FilterType))
	  {
		  ROS_ERROR("%s: Could not add a filter because no type was given", FilterType.c_str());
		  return false;
	  }
	  else if(!node->get_parameter("name", FilterName))
	  {
		  ROS_ERROR("%s: Could not add a filter because no name was given", FilterName.c_str());
		  return false;
	  }

	  if (FilterType.find("/") == std::string::npos)
	  {
		  ROS_ERROR("Bad filter type %s. Filter type must be of form <package_name>/<filter_name>", FilterType.c_str());
		  return false;
	  }

	  ROS_INFO("FilterType is : %s \n", FilterType.c_str());
	  ROS_INFO("FilterName is : %s \n", FilterName.c_str());

	  //Make sure the filter chain has a valid type
	  std::vector<std::string> libs = loader_.getDeclaredClasses();
	  bool found = false;

	  for (std::vector<std::string>::iterator it = libs.begin(); it != libs.end(); ++it)
	  {
		  cout<<"it = "<<*it<<endl;
		  found = true;
		  break;
		  
		  if (*it == FilterType)
		  {
			  found = true;
			  break;
		  }
	  }
	  if (!found)
	  {
		  
		  ROS_ERROR("Couldn't find filter of type %s", FilterType.c_str());
		  return false;
	  }

	  bool result = true;
	  cout<<"libs.size = "<<libs.size()<<endl;

	  for (unsigned int i = 0 ; i < libs.size(); i ++)
	  {
		  //cout<<"lib[i] = "<<libs[i]<<endl;
		  std::shared_ptr<filters::FilterBase<T> > p = loader_.createSharedInstance(libs[i]);
		  if (p.get() == NULL)
			  return false;
		  result = result &&  p.get()->configure(libs[i],node);
		  reference_pointers_.push_back(p);
		  ROS_DEBUG("%s: Configured filter at %p\n", filter_ns.c_str(),p.get());
		  
	  }

	  if (result == true)
	  {
		  configured_ = true;
	  }
	  return result;
};


private:

  std::vector<std::shared_ptr<filters::FilterBase<T> > > reference_pointers_;   ///<! A vector of pointers to currently constructed filters

  T buffer0_; ///<! A temporary intermediate buffer
  T buffer1_; ///<! A temporary intermediate buffer
  bool configured_; ///<! whether the system is configured  

};

};

#endif //#ifndef FILTERS_FILTER_CHAIN_H_
