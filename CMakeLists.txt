cmake_minimum_required(VERSION 3.5)
project(filters)

# Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
 # set(CMAKE_CXX_STANDARD 14)
#endif()

# Support C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

##############################################################################
# Define package
##############################################################################
find_package(ament_cmake REQUIRED)
find_Package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcutils)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclpy REQUIRED)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(Boost REQUIRED system)
link_directories(${Boost_LIBRARY_DIRS})

##############################################################################
# Set include and libraries
##############################################################################
set(INCLUDE_DIRS
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${builtin_interfaces_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${pluginlib_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})


set(LIBS
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pluginlib_LIBRARIES}
) 
##############################################################################
# Plugins
##############################################################################

add_library(mean SHARED src/mean.cpp)
target_link_libraries(mean ${LIBS})

add_library(median SHARED src/median.cpp)
target_link_libraries(median ${LIBS})

add_library(param_test SHARED src/test_params.cpp)
target_link_libraries(param_test ${LIBS})

add_library(increment SHARED src/increment.cpp)
target_link_libraries(increment ${LIBS})
#
add_library(transfer_function SHARED src/transfer_function.cpp)
target_link_libraries(transfer_function ${LIBS})

##############################################################################
# Install
##############################################################################

# Install libraries
install(TARGETS
        mean
        median
        transfer_function
	      param_test
	      increment
  DESTINATION lib/${PROJECT_NAME}  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include 
)

# Install headers
install(DIRECTORY "include/"
  DESTINATION include)


# Install plugins xml file
install(FILES default_plugins.xml
  DESTINATION share/${PROJECT_NAME}  
)
##############################################################################
# Find dependencies
##############################################################################
# Export include dependencies, directories, and libraries
ament_export_dependencies(ament_cmake)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(${PROJECT_NAME})   
ament_export_dependencies(pluginlib)
ament_export_dependencies(filters)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})
pluginlib_export_plugin_description_file(${PROJECT_NAME} default_plugins.xml)


##############################################################################
# Testing
##############################################################################

if(BUILD_TESTING)
    
    find_package(ament_cmake_gtest REQUIRED)
	find_package(ament_lint_auto REQUIRED)
    find_package(std_msgs REQUIRED)
    ament_lint_auto_find_test_dependencies()
    
    ament_add_gtest(test_mean  test/test_mean.cpp)
    target_link_libraries(test_mean ${LIBS})
    
    ament_add_gtest(test_median  test/test_median.cpp)
    target_link_libraries(test_median ${LIBS})
    
   ament_add_gtest(test_transfer_function  test/test_transfer_function.cpp)
   target_link_libraries(test_transfer_function ${LIBS})
    
    ament_add_gtest(test_params  test/test_params.cpp)
    target_link_libraries(test_params ${LIBS})
    
    ament_add_gtest(test_chain  test/test_chain.cpp)
    target_link_libraries(test_chain ${LIBS})
    
    install(
        TARGETS test_chain
               test_median 
               test_transfer_function 
                test_params 
                test_mean 
	DESTINATION lib/${PROJECT_NAME})
endif()

# Install params & launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

##############################################################################
# Cosmetics Testing
##############################################################################

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
ament_package()
