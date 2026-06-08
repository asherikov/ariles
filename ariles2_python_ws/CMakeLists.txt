cmake_minimum_required(VERSION 3.13.0)
project(ariles2_python_ws VERSION 2.8.1)

option(ARILES_ROS_ENABLE_TESTS "Enable tests." OFF)

#-----
set(ARILES_COMPONENTS  python)
#-----

include(${CMAKE_CURRENT_SOURCE_DIR}/../.cmake/common.cmake)