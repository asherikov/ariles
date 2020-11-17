cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_VERBOSE_MAKEFILE ON)


if (DEFINED CATKIN_DEVEL_PREFIX)
    project(ariles2_catkin_demo)

    find_package(catkin REQUIRED
        ariles2_core_catkin
        ariles2_rosparam_catkin
        ariles2_rapidjson_catkin
        ariles2_yamlcpp_catkin
        ariles2_octave_catkin
    )

    catkin_package(DEPENDS
        ariles2_core_catkin
        ariles2_rosparam_catkin
        ariles2_rapidjson_catkin
        ariles2_yamlcpp_catkin
        ariles2_octave_catkin
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

    find_package(ariles2_core_catkin REQUIRED)
    find_package(ariles2_rosparam_catkin REQUIRED)
    find_package(ariles2_rapidjson_catkin REQUIRED)
    find_package(ariles2_yamlcpp_catkin REQUIRED)
    find_package(ariles2_octave_catkin REQUIRED)


    add_executable(demo demo.cpp)
    target_link_libraries(demo
        ${ariles2_core_catkin_LIBRARIES}
        ${ariles2_rosparam_catkin_LIBRARIES}
        ${ariles2_rapidjson_catkin_LIBRARIES}
        ${ariles2_yamlcpp_catkin_LIBRARIES}
        ${ariles2_octave_catkin_LIBRARIES}
    )
endif()
