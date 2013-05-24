cmake_minimum_required(VERSION 2.8.3)
project(filters)

##############################################################################
# Find dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS roslib roscpp rosconsole rostest)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mean params increment median transfer_function
  CATKIN_DEPENDS roslib roscpp rosconsole
)

##############################################################################
# Build
##############################################################################

# Plugins
add_library(mean src/mean.cpp)
target_link_libraries(mean ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(params src/test_params.cpp)
target_link_libraries(params ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(increment src/increment.cpp)
target_link_libraries(increment ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(median src/median.cpp)
target_link_libraries(median ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(transfer_function src/transfer_function.cpp)
target_link_libraries(transfer_function ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Test median filter
catkin_add_gtest(median_test EXCLUDE_FROM_ALL test/test_median.cpp )
target_link_libraries(median_test median ${catkin_LIBRARIES} ${Boost_LIBRARIES}) 
add_rostest(test/test_median.launch)

# Test transfer function filter
catkin_add_gtest(transfer_function_test EXCLUDE_FROM_ALL test/test_transfer_function.cpp)
target_link_libraries(transfer_function_test transfer_function ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_rostest(test/test_transfer_function.launch)

# Test mean filter
catkin_add_gtest(mean_test EXCLUDE_FROM_ALL test/test_mean.cpp)
target_link_libraries(mean_test mean ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_rostest(test/test_mean.launch)

# Test params filter
catkin_add_gtest(params_test EXCLUDE_FROM_ALL test/test_params.cpp)
target_link_libraries(params_test params ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_rostest(test/test_params.launch)

# Test plugin loading into filter chain
catkin_add_gtest(chain_test EXCLUDE_FROM_ALL test/test_chain.cpp)
target_link_libraries(chain_test ${Boost_libraries}) # Needed for OSX
add_rostest(test/test_chain.launch)

# Test realtime safe buffer class
catkin_add_gtest(realtime_buffer_test EXCLUDE_FROM_ALL test/test_realtime_circular_buffer.cpp)

##############################################################################
# Install
##############################################################################

# Install libraries
install(TARGETS mean params increment median transfer_function
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install plugins xml file
install(FILES default_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)