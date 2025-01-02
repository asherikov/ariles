cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_VERBOSE_MAKEFILE ON)


if (DEFINED CATKIN_DEVEL_PREFIX)
    project(ariles2_ws_demo)

    find_package(catkin REQUIRED
        ariles2_core_ws
        ariles2_rosparam_ws
        ariles2_rapidjson_ws
        ariles2_yamlcpp_ws
        ariles2_octave_ws
        ariles2_pugixml_ws
    )

    catkin_package(DEPENDS
        ariles2_core_ws
        ariles2_rosparam_ws
        ariles2_rapidjson_ws
        ariles2_yamlcpp_ws
        ariles2_octave_ws
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
else()
    project(ariles2_demo)

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
endif()
