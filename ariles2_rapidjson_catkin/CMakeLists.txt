cmake_minimum_required(VERSION 3.0)
project(ariles2_rapidjson_catkin VERSION 2.0.0)


option(ARILES_ROS_ENABLE_TESTS "Enable tests." OFF)

#-----
# ARILES
set(ARILES_ENABLE_CORE          "OFF"   CACHE STRING "")
set(ARILES_VISITOR_rapidjson    "ON"    CACHE STRING "")

set(ARILES_COMPONENTS  rapidjson)
#-----

include(${CMAKE_CURRENT_SOURCE_DIR}/../.cmake/common.cmake)
