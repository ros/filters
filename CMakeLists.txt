cmake_minimum_required(VERSION 3.5)
project(filters)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(Boost REQUIRED system)
link_directories(${Boost_LIBRARY_DIRS})

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

add_library( ${PROJECT_NAME} SHARED
src/mean.cpp
src/median.cpp
src/test_params.cpp
src/increment.cpp
src/transfer_function.cpp
)
target_link_libraries( ${PROJECT_NAME} ${LIBS})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(FILES default_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(builtin_interfaces)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(pluginlib)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME})
pluginlib_export_plugin_description_file(${PROJECT_NAME} default_plugins.xml)

if(BUILD_TESTING)

    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_lint_auto REQUIRED)
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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
