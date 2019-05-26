cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_VERBOSE_MAKEFILE ON)


if (DEFINED CATKIN_DEVEL_PREFIX)
    project(ariles_catkin_dependency_test)

    find_package(catkin REQUIRED ariles_ros)

    catkin_package(DEPENDS ariles_ros)

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
    project(ariles_cmake_dependency_test)

    find_package(ariles_ros REQUIRED)

    include_directories(${ariles_ros_INCLUDE_DIRS})

    add_executable(demo demo.cpp)
    target_link_libraries(demo ${ariles_ros_LIBRARIES})
endif()
