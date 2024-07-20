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

    find_package(ariles2_core_ws REQUIRED)
    find_package(ariles2_rosparam_ws REQUIRED)
    find_package(ariles2_rapidjson_ws REQUIRED)
    find_package(ariles2_yamlcpp_ws REQUIRED)
    find_package(ariles2_octave_ws REQUIRED)
    find_package(ariles2_pugixml_ws REQUIRED)


    add_executable(demo demo.cpp)
    target_link_libraries(demo
        ${ariles2_core_ws_LIBRARIES}
        ${ariles2_rosparam_ws_LIBRARIES}
        ${ariles2_rapidjson_ws_LIBRARIES}
        ${ariles2_yamlcpp_ws_LIBRARIES}
        ${ariles2_octave_ws_LIBRARIES}
        ${ariles2_pugixml_ws_LIBRARIES}
    )
endif()
