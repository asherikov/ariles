cmake_minimum_required(VERSION 3.0)
project(ariles2_yamlcpp_catkin VERSION 2.0.0)


option(ARILES_ROS_ENABLE_TESTS "Enable tests." OFF)

#-----
set(ARILES_COMPONENTS  yaml-cpp)
#-----

include(${CMAKE_CURRENT_SOURCE_DIR}/../.cmake/common.cmake)
