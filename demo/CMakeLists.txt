cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_VERBOSE_MAKEFILE ON)


if (DEFINED CATKIN_DEVEL_PREFIX)
    project(ariles2_catkin_demo)

    find_package(catkin REQUIRED
        ariles2_core_ros
        ariles2_rapidjson_ros
        ariles2_yamlcpp_ros
        ariles2_octave_ros
    )

    catkin_package(DEPENDS
        ariles2_core_ros
        ariles2_rapidjson_ros
        ariles2_yamlcpp_ros
        ariles2_octave_ros
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

    find_package(ariles2_core_ros REQUIRED)
    find_package(ariles2_rapidjson_ros REQUIRED)
    find_package(ariles2_yamlcpp_ros REQUIRED)
    find_package(ariles2_octave_ros REQUIRED)

    include_directories(
        ${ariles2_core_ros_INCLUDE_DIRS}
        ${ariles2_rapidjson_ros_INCLUDE_DIRS}
        ${ariles2_yamlcpp_ros_INCLUDE_DIRS}
        ${ariles2_octave_ros_INCLUDE_DIRS}
    )

    add_executable(demo demo.cpp)
    target_link_libraries(demo
        ${ariles2_core_ros_LIBRARIES}
        ${ariles2_rapidjson_ros_LIBRARIES}
        ${ariles2_yamlcpp_ros_LIBRARIES}
        ${ariles2_octave_ros_LIBRARIES}
    )
endif()
