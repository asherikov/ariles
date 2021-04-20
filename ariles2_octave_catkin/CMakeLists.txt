cmake_minimum_required(VERSION 3.0)
project(ariles2_octave_catkin VERSION 2.0.1)


option(ARILES_ROS_ENABLE_TESTS "Enable tests." OFF)

#-----
set(ARILES_COMPONENTS  octave)
#-----

include(${CMAKE_CURRENT_SOURCE_DIR}/../.cmake/common.cmake)
