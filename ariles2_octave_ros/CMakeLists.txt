cmake_minimum_required(VERSION 3.0)
project(ariles2_octave_ros VERSION 2.0.0)


option(ARILES_ROS_ENABLE_TESTS "Enable tests." OFF)

#-----
# ARILES
set(ARILES_ENABLE_CORE          "BUILD" CACHE STRING "")
set(ARILES_VISITOR_octave       "ON"    CACHE STRING "")

set(ARILES_COMPONENTS  octave)
#-----

include(${CMAKE_CURRENT_SOURCE_DIR}/../.cmake/common.cmake)
