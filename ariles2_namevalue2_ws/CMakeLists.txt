cmake_minimum_required(VERSION 3.0)
project(ariles2_namevalue2_ws VERSION 2.3.2)


option(ARILES_ROS_ENABLE_TESTS "Enable tests." OFF)

#-----
set(ARILES_COMPONENTS  namevalue2)
#-----

include(${CMAKE_CURRENT_SOURCE_DIR}/../.cmake/common.cmake)
