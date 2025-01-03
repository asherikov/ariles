cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_VERBOSE_MAKEFILE ON)


project(ariles2_plain_ros1_demo)

find_package(ariles2-core REQUIRED)
find_package(ariles2-rosparam REQUIRED)
find_package(ariles2-rapidjson REQUIRED)
find_package(ariles2-yaml-cpp REQUIRED)
find_package(ariles2-octave REQUIRED)
find_package(ariles2-pugixml REQUIRED)


add_executable(demo demo.cpp)
target_link_libraries(demo
    ariles2::core
    ariles2::rosparam
    ariles2::rapidjson
    ariles2::yaml-cpp
    ariles2::octave
    ariles2::pugixml
)
