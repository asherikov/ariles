cmake_minimum_required(VERSION 3.13.0)
project(ariles2_pugixml_ws VERSION 2.5.0)


option(ARILES_ROS_ENABLE_TESTS "Enable tests." OFF)

#-----
set(ARILES_COMPONENTS  pugixml)
#-----

include(${CMAKE_CURRENT_SOURCE_DIR}/../.cmake/common.cmake)
