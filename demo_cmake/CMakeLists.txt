cmake_minimum_required(VERSION 3.13.0)

set(CMAKE_VERBOSE_MAKEFILE ON)


project(ariles2_cmake_demo)

find_package(ariles2-core REQUIRED)
find_package(ariles2-yaml-cpp REQUIRED)
find_package(ariles2-octave REQUIRED)
find_package(ariles2-python REQUIRED)
find_package(ariles2-pugixml REQUIRED)


add_executable(demo demo.cpp)
target_link_libraries(demo
    ariles2::core
    ariles2::yaml-cpp
    ariles2::octave
    ariles2::python
    ariles2::pugixml
)
