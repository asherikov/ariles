cmake_minimum_required(VERSION 3.13.0)
project(ariles2_yamlcpp_ws VERSION 2.8.0)


option(ARILES_ROS_ENABLE_TESTS "Enable tests." OFF)

#-----
set(ARILES_COMPONENTS  yaml-cpp)
#-----

include(${CMAKE_CURRENT_SOURCE_DIR}/../.cmake/common.cmake)
