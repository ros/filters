cmake_minimum_required(VERSION 3.5)

project(filters)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(pluginlib REQUIRED)
find_package(xmlrpcpp REQUIRED)

include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${rcutils_INCLUDE_DIRS}
    ${class_loader_INCLUDE_DIRS}
    ${pluginlib_INCLUDE_DIRS}
    ${xmlrpcpp_INCLUDE_DIRS}
    $ENV{BOOST_ROOT}
)

##############################################################################
# Build
##############################################################################

add_definitions(-DROS2)

# Plugins
add_library(mean src/mean.cpp)
target_link_libraries(mean ${rclcpp_LIBRARIES} ${rcutils_LIBRARIES})
add_library(params src/test_params.cpp)
target_link_libraries(params ${rclcpp_LIBRARIES} ${rcutils_LIBRARIES})
add_library(increment src/increment.cpp)
target_link_libraries(increment ${rclcpp_LIBRARIES} ${rcutils_LIBRARIES})
add_library(median src/median.cpp)
target_link_libraries(median ${rclcpp_LIBRARIES} ${rcutils_LIBRARIES})
add_library(transfer_function src/transfer_function.cpp)
target_link_libraries(transfer_function ${rclcpp_LIBRARIES} ${rcutils_LIBRARIES})


pluginlib_export_plugin_description_file(filters default_plugins.xml)
ament_export_include_directories(include)
ament_export_libraries(
  mean 
  params 
  increment
  median
  transfer_function
)
ament_package()

##############################################################################
# Install
##############################################################################

# Install libraries
install(TARGETS mean params increment median transfer_function
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/)

# Install plugins xml file
#install(FILES default_plugins.xml
#  DESTINATION share
#)
