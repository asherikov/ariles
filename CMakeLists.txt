cmake_minimum_required(VERSION 2.8.12)
project(ariles_ros)

set(ARILES_DIR             "${CMAKE_CURRENT_SOURCE_DIR}/ariles")

set(CMAKE_VERBOSE_MAKEFILE ON)
#-----
# ARILES
set(ARILES_CPP_STANDARD         "c++11" CACHE STRING "")

set(ARILES_DEFAULT_CONFIGURABLE_FLAGS "ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED" CACHE STRING "")
set(ARILES_BRIDGES_DEFAULT_MODE "OFF"   CACHE STRING "")
set(ARILES_BRIDGE_yaml_cpp      "ON"    CACHE STRING "")
set(ARILES_BRIDGE_ros           "ON"    CACHE STRING "")
set(ARILES_BRIDGE_octave        "ON"    CACHE STRING "")

set(ARILES_ADAPTER_EIGEN            "ON"    CACHE STRING "")
set(ARILES_ADAPTER_BOOST_POINTER    "ON"    CACHE STRING "")
set(ARILES_ADAPTER_BOOST_OPTIONAL   "ON"    CACHE STRING "")

set(ARILES_BUILD_REGRESSION_TESTS   "OFF"   CACHE STRING "")
#-----

add_subdirectory("${ARILES_DIR}" "./ariles")

find_package(catkin REQUIRED)

catkin_package(
    CFG_EXTRAS find_ariles.cmake
)
