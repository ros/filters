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


#include <typeinfo>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include <boost/scoped_ptr.hpp>
#include <memory> 
#include <boost/algorithm/string.hpp>
#include <map>
#include <string>
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include <iterator>
#include <vector>

#define ROS_DEBUG printf
#define ROS_FATAL printf
#define ROS_WARN printf
#define ROS_INFO printf
#define ROS_ASSERT_MSG printf
#define ROS_ERROR printf
using namespace std;

namespace filters
{

/** \brief A Base filter class to provide a standard interface for all filters
filter_b *
 */
template<typename T> class FilterBase
{
public:
	/** \brief Default constructor used by Filter Factories
	 */
	FilterBase():configured_(false){};

	/** \brief Virtual Destructor
	 */
	virtual ~FilterBase(){};

	/** \brief Configure the filter from the parameter server
	 * \param The parameter from which to read the configuration
	 * \param node_handle The optional node handle, useful if operating in a different namespace.
	 */
	bool configure(const std::string& param_name,rclcpp::Node::SharedPtr node)
	{            

		cerr << "param name  "<< param_name << endl;
		configured_ = get_configure(param_name,node);
		return configured_;
	}
	
bool configure(rclcpp::Node::SharedPtr node)
{  
    if (configured_)
	{
		ROS_DEBUG("Filter type already being reconfigured "); 
        
	};
    configured_ = false;
    parameters_and_prefixes = node->list_parameters({ }, 10);
		//ROS_DEBUG("Got the List of parameters: Size = %ul", parameters_and_prefixes.names.size());

		for (auto & name : parameters_and_prefixes.names) {
			for (auto & parameter : node->get_parameters({name})) {
                
                ss1 << "\nParameter name: " << parameter.get_name();
                ss1 << "\nParameter data_type: " << parameter.get_type();
                ss1 << "\nParameter value (" << parameter.get_type_name() << "): " <<
				parameter.value_to_string();
                filter_param[parameter.get_name()] = parameter.value_to_string();
			}
		}
        for(map<string, string>::iterator filter_it = filter_param.begin(); filter_it != filter_param.end(); ++filter_it ){
        string filter_name = filter_it->first;
        string filter_type = filter_it->second;
            uint32_t number_of_observations_;
            if(std::string::npos != filter_name.find("params"))
            {
                std::string p_name = filter_name;
                int pos1 = p_name.find(".");
                string name = p_name.substr(pos1+1);
                p_name.erase(pos1+1,filter_name.length()+1);
                std::cout << p_name << '\n';

                param_name = p_name;
            }
            cerr << "param name  "<< param_name << endl;
            configured_ = get_configure(param_name,node);
 
        }

		cerr << "param name  "<< param_name << endl;
        configured_ = get_configure(param_name,node);
		return configured_;
  
}	
	
	
	virtual bool get_configure(const std::string& param_name,rclcpp::Node::SharedPtr node)=0;
	/** \brief Update the filter and return the data seperately
	 * This is an inefficient way to do this and can be overridden in the derived class
	 * \param data_in A reference to the data to be input to the filter
	 * \param data_out A reference to the data output location
	 */
	virtual bool update(const T& data_in, T& data_out)=0;

protected:
	
	bool configured_;
	rcl_interfaces::msg::ListParametersResult parameters_and_prefixes;
    map<string, string> filter_param;
    std::stringstream ss1;
    std::string param_name;
    int cnt =0;
	/** \brief Pure virtual function for the sub class to configure the filter
	 * This function must be implemented in the derived class.
	 */
	

};
template <typename T>
class MultiChannelFilterBase : public FilterBase<T>
{
public:
  MultiChannelFilterBase():number_of_channels_(0){};

  /** \brief Configure the filter from the parameter server
   * \param number_of_channels How many parallel channels the filter will process
   * \param The parameter from which to read the configuration
   * \param node_handle The optional node handle, useful if operating in a different namespace.
   */
  bool configure(unsigned int number_of_channels,const std::string& param_name,rclcpp::Node::SharedPtr node)
  {
      
	if (configured_)
	{
		ROS_DEBUG("Filter type already being reconfigured "); 
        
	};
    configured_ = false;
    number_of_channels_ = number_of_channels;
   	ROS_DEBUG("MultiChannelFilterBase configured with %d channels", number_of_channels_);
        cerr << "param name  "<< param_name.c_str() << endl;
        configured_ = get_configure(param_name,node);
    return configured_;
   
  };
  
bool configure(unsigned int number_of_channels,rclcpp::Node::SharedPtr node)
  {
      
	if (configured_)
	{
		ROS_DEBUG("Filter type already being reconfigured "); 
        
	};
    configured_ = false;
    number_of_channels_ = number_of_channels;
   	ROS_DEBUG("MultiChannelFilterBase configured with %d channels", number_of_channels_);

    parameters_and_prefixes = node->list_parameters({ }, 10);
		
        for (auto & name : parameters_and_prefixes.names) {
        for (auto & parameter : node->get_parameters({name})) {

                ROS_DEBUG("Getting parameters for filters "); 
                filter_param[parameter.get_name()] = parameter.value_to_string();
                ROS_DEBUG("4. In configure function for base:");
                if(parameter.get_name() == "name" && parameter.get_type() == rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
            {
             
                 cout<< "\n STRING VALUE type:" << endl;
                 
            }
			}
		}
     RCLCPP_INFO(node->get_logger(), ss1.str().c_str());
          
    for (map<string, string>::iterator filter_it = filter_param.begin();
        filter_it != filter_param.end(); ++filter_it)
    {
        ROS_DEBUG("\n5. In  configure function for base:");
        string filter_name = filter_it->first;
        string filter_type = filter_it->second;
        if (std::string::npos != filter_name.find("params")) {
            ROS_DEBUG("\ngot params to get substring");
            std::string p_name = filter_name;
                int pos1 = p_name.find(".");
                string name = p_name.substr(pos1+1);
                p_name.erase(pos1+1,filter_name.length()+1);
                std::cout << p_name << '\n';

                param_name = p_name;
           
            cerr << "param_name: " << param_name << endl;
        }
    }
        cerr << "param name  "<< param_name.c_str() << endl;
        configured_ = get_configure(param_name,node);
    return configured_;
   
  };
  

  virtual bool get_configure(const std::string& param_name,rclcpp::Node::SharedPtr node)=0;

  /** \brief Update the filter and return the data seperately
   * \param data_in A reference to the data to be input to the filter
   * \param data_out A reference to the data output location
   * This funciton must be implemented in the derived class.
   */
  virtual bool update(const std::vector<T>& data_in, std::vector<T>& data_out)=0;

  virtual bool update(const T& data_in, T& data_out)
  {
	 
    ROS_ERROR("THIS IS A MULTI FILTER DON'T CALL SINGLE FORM OF UPDATE");
    return false;
  };


protected:
  using FilterBase<T>::configured_;
  // How many parallel inputs for which the filter is to be configured
  unsigned int number_of_channels_;
  std::stringstream ss1;
  std::string param_name;
   map <string, string> filter_param;
  //Store the list of parameters specific to node
  rcl_interfaces::msg::ListParametersResult parameters_and_prefixes;

};

}
