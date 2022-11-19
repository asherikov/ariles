cmake_minimum_required(VERSION 2.8.12)
project(ariles_cmake_dependency_test)

set(CMAKE_VERBOSE_MAKEFILE ON)

add_executable(demo ../api_v2/demo_api_v2.cpp)

foreach(ARILES_COMPONENT ${ARILES_COMPONENTS})
    find_package(ariles2-${ARILES_COMPONENT} REQUIRED)

    target_link_libraries(demo ${ariles2-${ARILES_COMPONENT}_LIBRARIES})
endforeach()
