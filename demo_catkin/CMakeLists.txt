cmake_minimum_required(VERSION 3.13.0)

set(CMAKE_VERBOSE_MAKEFILE ON)

project(ariles2_catkin_demo)

find_package(catkin REQUIRED
    ariles2_core_ws
    ariles2_rosparam_ws
    ariles2_yamlcpp_ws
    ariles2_octave_ws
    ariles2_python_ws
    ariles2_pugixml_ws
)

catkin_package(DEPENDS
    ariles2_core_ws
    ariles2_rosparam_ws
    ariles2_yamlcpp_ws
    ariles2_octave_ws
    ariles2_python_ws
    ariles2_pugixml_ws
)

include_directories(
    include
    SYSTEM ${catkin_INCLUDE_DIRS}
)

add_executable(demo demo.cpp)

target_link_libraries(demo ${catkin_LIBRARIES})

install(TARGETS demo
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
