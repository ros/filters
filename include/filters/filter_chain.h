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
#include "filters/realtime_circular_buffer.h"
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
    
      auto parameters_and_prefixes = node->list_parameters({ }, 10);
		
        
		for (auto & name : parameters_and_prefixes.names) {
			for (auto & parameter : node->get_parameters({name})) {
                               
                ss1 << "\nParameter name: " << parameter.get_name();
                ss1 << "\nParameter data_type: " << parameter.get_type();
                ss1 << "\nParameter value (" << parameter.get_type_name() << "): " <<
				parameter.value_to_string();
                filter_param[parameter.get_name()] = parameter.value_to_string();
                
			}
		}
		 RCLCPP_INFO(node->get_logger(), ss1.str().c_str());
         
    for (map<string, string>::iterator filter_it = filter_param.begin();
        filter_it != filter_param.end(); ++filter_it)
    {
       
        string filter_name = filter_it->first;
        string filter_type = filter_it->second;
        if (std::string::npos != filter_name.find("params")) {
            std::string p_name = filter_name;
            int pos1 = p_name.find(".");
            string name = p_name.substr(pos1+1);
            p_name.erase(pos1+1,filter_name.length()+1);
            std::cout << p_name << '\n';

            param_name1 = p_name;
            
            cerr << "1. param_name: " << param_name1 << endl;
        }
       else if(std::string::npos != filter_name.find("type")) {
           //try
            ROS_DEBUG("\n6. In  configure function for chain:");
                if(loader_.isClassAvailable(filter_type))
                {
                    bool have_class = false;
                    bool result = true;
        
                    std::vector<string> classes = loader_.getDeclaredClasses();
                    for (unsigned int i = 0; i < classes.size(); ++i) {
                    cerr << "\n1. class/filter type available in your filter list:"<< classes[i].c_str() << endl;
                    }
                    cerr << "class size is:" << classes.size() << endl;
                    for (unsigned int i = 0; i < classes.size(); ++i) {
                        cerr<<"three values to check"<<filter_type<<classes[i] <<loader_.getName(classes[i])<<endl;
                                
                        
                        if (filter_type == classes[i]) {
                            // if we've found a match... we'll get the fully qualified name
                            // and break out of the loop
                            cerr << "2. class/filter type available %s in your filter list:"<< classes[i].c_str() << endl;
                            filter_type = classes[i];
                            have_class = true;
                            //break;
                            cerr<< "available clase is " << classes[i] << endl;
                            std::shared_ptr<filters::FilterBase<T> > p  = loader_.createSharedInstance(filter_type);
                            cerr<<"p size"<<p.get() <<endl;
                            if (p.get() == NULL)
                                return false;  
                            
                            result = result &&  p.get()->configure(param_name1,node);
                            reference_pointers_.push_back(p);
                            
                            unsigned int list_size = reference_pointers_.size();
                        }
                            
                    }
                    if (!have_class) {
                        ROS_ERROR("\nUnable to find filter class %s. Check that filter "
                        "is fully declared.",
                        filter_type.c_str());
                        continue;
                    }
                }
        ROS_ERROR("\nfilter type available%s. ",filter_type.c_str());
                
                
                cerr<<"update called list_size = "<<endl;
                    //}
            }
            
            
        }
    
    if (result == true)
	  {
		  configured_ = true;
	  }
	  return result;
    
  };

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
    {
        
        cout<<"reference pointer value = "<<reference_pointers_[0]<<endl;
        result = reference_pointers_[0]->update(data_in, data_out);
    }
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
 
private:
  
  std::vector<std::shared_ptr<filters::FilterBase<T> > > reference_pointers_;   ///<! A vector of pointers to currently constructed filters
  unsigned int i = 0;
  bool result = true;
 
  rclcpp::Parameter parameter_test;
  map <string, string> filter_param;
  std::string FilterType;
  std::string FilterName,param_name1;
  std::stringstream ss1;
  vector<string> classes = loader_.getDeclaredClasses();
  T buffer0_; ///<! A temporary intermediate buffer
  T buffer1_; ///<! A temporary intermediate buffer
  bool configured_; ///<! whether the system is configured  

};

template <typename T>
class MultiChannelFilterChain
{
private:
  pluginlib::ClassLoader<filters::MultiChannelFilterBase<T> > loader_;
public:
  /** \brief Create the filter chain object */
  MultiChannelFilterChain(std::string data_type): loader_("filters", std::string("filters::MultiChannelFilterBase<") + data_type + std::string(">")), configured_(false)
  {
    std::string lib_string = "";
    std::vector<std::string> libs = loader_.getDeclaredClasses();
    for (unsigned int i = 0 ; i < libs.size(); i ++)
    {
      lib_string = lib_string + std::string(", ") + libs[i];
    }
    
    ROS_DEBUG("In MultiChannelFilterChain ClassLoader found the following libs: %s", lib_string.c_str());
  };
  
  /**@brief Configure the filter chain from a configuration stored on the parameter server
   * @param param_name The name of the filter chain to load
   * @param node The node handle to use if a different namespace is required
   */
  bool configure(unsigned int number_of_channels, rclcpp::Node::SharedPtr node)
  {
    number_of_channels_ = number_of_channels;
   
    auto parameters_and_prefixes = node->list_parameters({ }, 10);
        for (auto & name : parameters_and_prefixes.names) {
			for (auto & parameter : node->get_parameters({name})) {
                              
                ss1 << "\nParameter name: " << parameter.get_name();
                ss1 << "\nParameter data_type: " << parameter.get_type();
                ss1 << "\nParameter value (" << parameter.get_type_name() << "): " <<
				parameter.value_to_string();
                filter_param[parameter.get_name()] = parameter.value_to_string();
                
			}
		}
		 RCLCPP_INFO(node->get_logger(), ss1.str().c_str());
          
    for (map<string, string>::iterator filter_it = filter_param.begin();
        filter_it != filter_param.end(); ++filter_it)
    {
       
        string filter_name = filter_it->first;
        string filter_type = filter_it->second;
        if (std::string::npos != filter_name.find("params")) {
            
            std::string p_name = filter_name;
            int pos1 = p_name.find(".");
            string name = p_name.substr(pos1+1);
            p_name.erase(pos1+1,filter_name.length()+1);
            std::cout << p_name << '\n';

            param_name2 = p_name;
            
        }
       else if (std::string::npos != filter_name.find("type")) {
           //try{
             
                if(loader_.isClassAvailable(filter_type))
                {
                
                    bool have_class = false;
                    bool result = true;
                    buffer0_.resize(number_of_channels);
                    buffer1_.resize(number_of_channels);
                    std::vector<string> classes = loader_.getDeclaredClasses();
                    
                    for (unsigned int i = 0; i < classes.size(); ++i) {
                        
                        if (filter_type == classes[i]) {
                            // if we've found a match... we'll get the fully qualified name
                            
                            cerr << "2. class/filter type available %s in your filter list:"<< classes[i].c_str() << endl;
                            filter_type = classes[i];
                            have_class = true;
                            
                            cerr<< "available clase is " << classes[i] << endl;
                            std::shared_ptr<filters::MultiChannelFilterBase<T> > p  = loader_.createSharedInstance(filter_type);
                            cerr<<"p size"<<p.get() <<endl;
                            if (p.get() == NULL)
                                return false;  
                            result = result &&  p.get()->configure(number_of_channels,param_name2,node);
                            reference_pointers_.push_back(p);
                            
                            unsigned int list_size = reference_pointers_.size();
                            cerr<< "\nreference pointer  " << list_size<< endl;
                            
                        }
                            
                    }

                    if (!have_class) {
                        ROS_ERROR("\nUnable to find filter class %s. Check that filter "
                        "is fully declared.",
                        filter_type.c_str());
                        continue;
                    }
     

                }
                ROS_ERROR("\nfilter type available%s. ",filter_type.c_str());
                
            }
            
    }     
   
     if (result == true)
	  {
		  configured_ = true;
	  }
	  return result;
  };
   /** \brief process data through each of the filters added sequentially */
  bool update(const std::vector<T>& data_in, std::vector<T>& data_out)
  {
    
    unsigned int list_size = reference_pointers_.size();
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
      cerr << " returned from second filter" << endl;
    }
    else
    {
      result = reference_pointers_[0]->update(data_in, buffer0_);  //first copy in
      for (unsigned int i = 1; i <  reference_pointers_.size() - 1; i++) // all but first and last (never if size = 2)
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
 
 
  ~MultiChannelFilterChain()
  {
    clear();

  };  
  /** \brief Clear all filters from this chain */
  bool clear() 
  {
    configured_ = false;
    reference_pointers_.clear();
    buffer0_.clear();
    buffer1_.clear();
    return true;
  };
   

private:
  
  bool result = true;
  std::vector<std::shared_ptr<filters::MultiChannelFilterBase<T> > > reference_pointers_;   ///<! A vector of pointers to currently constructed filters
  rclcpp::Parameter parameter_test;
  unsigned int i = 0; 
  map <string, string> filter_param;
  std::string FilterType;
  std::string FilterName, param_name2;
  std::stringstream ss1;
  std::vector<string> classes;
  std::vector<T> buffer0_; ///<! A temporary intermediate buffer
  std::vector<T> buffer1_; ///<! A temporary intermediate buffer
  bool configured_; ///<! whether the system is configured 
  int number_of_channels_;
};
};
#endif //#ifndef FILTERS_FILTER_CHAIN_H_
