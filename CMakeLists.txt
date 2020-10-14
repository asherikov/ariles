# General parameters
cmake_minimum_required (VERSION 2.8.12)

set(ARILES_VERSION_MAJOR 2)
set(ARILES_VERSION_MINOR 0)
set(ARILES_VERSION_PATCH 0)

if(CMAKE_VERSION VERSION_LESS 3)
    project (ariles2 CXX)
    set(PROJECT_VERSION_MAJOR ${ARILES_VERSION_MAJOR})
    set(PROJECT_VERSION_MINOR ${ARILES_VERSION_MINOR})
    set(PROJECT_VERSION_PATCH ${ARILES_VERSION_PATCH})
else()
    if(POLICY CMP0048)
        cmake_policy(SET CMP0048 NEW)
    endif()
    project(ariles2 VERSION ${ARILES_VERSION_MAJOR}.${ARILES_VERSION_MINOR}.${ARILES_VERSION_PATCH})
endif()

set(ARILES_HOMEPAGE http://asherikov.github.com/ariles/2)
set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")


#####################################
## Options (definitions & defaults)
#####################################

set    (ARILES_ENABLE_CORE              "ON" CACHE STRING "ON = enable, OFF = disable, NOINSTALL")

# extra visitors
set    (ARILES_VISITORS_DEFAULT_MODE    "ON" CACHE STRING "ON = enable, OFF = disable")

# extra adapters
option (ARILES_ADAPTER_EIGEN            "Enable eigen support." ON)
option (ARILES_ADAPTER_BOOST_POINTER    "Enable boost pointer support." ON)
option (ARILES_ADAPTER_BOOST_OPTIONAL   "Enable boost optional support." ON)
option (ARILES_ADAPTER_BETTER_ENUMS     "Enable support for beter enums." ON)
option (ARILES_ADAPTER_ROSMSG           "ROS message conversions." ON)

# build parameters
set    (ARILES_CPP_STANDARD             "c++03" CACHE STRING "C++ standard version")
option (ARILES_CPP_SANITIZERS           "Enable sanitizers"                 OFF)
option (ARILES_VERBOSE                  "Verbose build"                     ON)
option (ARILES_BUILD_REGRESSION_TESTS   "Build regression tests."           ON)
option (ARILES_CCACHE                   "Enable ccache if available"        ON)
set    (ARILES_DEB_TARGETS              "" CACHE STRING "Enable debian package generation.")

# install parameters
set    (ARILES_PKGCONFIG_INSTALL_PATH   "" CACHE STRING "pkg-config pc-file install path, not installed if empty")



#####################################
## cmake modules
#####################################

include(FindPkgConfig)
include(GNUInstallDirs)
include(CMakeParseArguments)
include(CPackComponent)
include(CMakePackageConfigHelpers)

include(cmakeut_list_filenames)
include(cmakeut_dump_variables)
include(cmakeut_compiler_flags)

include (cmakeut_detect_func_macro)
cmakeut_detect_func_macro()


#####################################
## CPack configuration
#####################################

# CPACK_PACKAGE_NAME    If not specified, defaults to the project name.

set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})

set(CPACK_PACKAGE_CONTACT "Alexander Sherikov <asherikov@yandex.com>")
set(CPACK_PACKAGE_VENDOR "https://github.com/asherikov/ariles")

#set(CPACK_PACKAGE_DESCRIPTION_FILE "${PROJECT_SOURCE_DIR}/README.md")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "C++ serialization library")

set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")

set(CPACK_MONOLITHIC_INSTALL OFF)
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_COMPONENTS_GROUPING ONE_PER_GROUP)


set(CPACK_GENERATOR "DEB")


set(CPACK_DEBIAN_PACKAGE_HOMEPAGE   "https://asherikov.github.io/ariles/2")
set(CPACK_DEBIAN_PACKAGE_PRIORITY   "optional")
set(CPACK_DEBIAN_PACKAGE_SECTION    "devel")


#####################################

if(NOT CMAKE_CXX_COMPILER_LAUNCHER)
    if(ARILES_CCACHE)
        find_program(CCACHE_FOUND ccache)
        if(CCACHE_FOUND)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
        endif(CCACHE_FOUND)
    endif()
endif()


if (NOT ${ARILES_ENABLE_CORE} STREQUAL "ON")
    # tests cannot be built if the core is not installed
    set(ARILES_BUILD_REGRESSION_TESTS   OFF)

    # adapters are installed only with core
    set(ARILES_ADAPTER_EIGEN            OFF)
    set(ARILES_ADAPTER_BOOST_POINTER    OFF)
    set(ARILES_ADAPTER_BOOST_OPTIONAL   OFF)
    set(ARILES_ADAPTER_BETTER_ENUMS     OFF)
    set(ARILES_ADAPTER_ROSMSG           OFF)
endif()



set(ARILES_VISITORS_DIRS "${PROJECT_SOURCE_DIR}/extra_visitors/")
set(ARILES_VISITORS "")
set(ARILES_ENABLED_VISITORS "")
foreach (ARILES_VISITORS_DIR ${ARILES_VISITORS_DIRS})
    cmakeut_list_filenames("${ARILES_VISITORS_DIR}"  "ARILES_VISITORS_TMP" DIRS_WITH_CMAKELISTS)
    list(APPEND ARILES_VISITORS ${ARILES_VISITORS_TMP})

    foreach(ARILES_VISITOR ${ARILES_VISITORS_TMP})
        set(ARILES_VISITOR_${ARILES_VISITOR}_DIR  "${ARILES_VISITORS_DIR}/${ARILES_VISITOR}")

        if (DEFINED ARILES_VISITOR_${ARILES_VISITOR})
            set    ("ARILES_VISITOR_${ARILES_VISITOR}" "${ARILES_VISITOR_${ARILES_VISITOR}}"
                    CACHE STRING "Visitor '${ARILES_VISITOR} (ON / OFF")

            if (NOT ${ARILES_VISITOR_${ARILES_VISITOR}} STREQUAL "OFF")
                list(APPEND ARILES_ENABLED_VISITORS "${ARILES_VISITOR}")
            endif()
        else()
            set    ("ARILES_VISITOR_${ARILES_VISITOR}" "${ARILES_VISITORS_DEFAULT_MODE}"
                    CACHE STRING "Visitor '${ARILES_VISITOR} (ON / OFF)")

            if (NOT ${ARILES_VISITORS_DEFAULT_MODE} STREQUAL "OFF")
                list(APPEND ARILES_ENABLED_VISITORS "${ARILES_VISITOR}")
            endif()
        endif()


        if (ARILES_VERBOSE)
            message(STATUS "----------------------VISITOR----------------------")
            message(STATUS "Name: ${ARILES_VISITOR}")
            message(STATUS "Path: ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}")
            message(STATUS "Enabled: ${ARILES_VISITOR_${ARILES_VISITOR}}")
            message(STATUS "Enable/disable option name: ARILES_VISITOR_${ARILES_VISITOR}")
            message(STATUS "--------------------------------------------------")
        endif(ARILES_VERBOSE)
    endforeach(ARILES_VISITOR)
endforeach(ARILES_VISITORS_DIR)


set(ARILES_EXTRA_ADAPTERS "")
if (ARILES_ADAPTER_EIGEN)
    list(APPEND ARILES_EXTRA_ADAPTERS "eigen")
endif(ARILES_ADAPTER_EIGEN)
if (ARILES_ADAPTER_BOOST_POINTER)
    list(APPEND ARILES_EXTRA_ADAPTERS "boost_pointer")
endif(ARILES_ADAPTER_BOOST_POINTER)
if (ARILES_ADAPTER_BOOST_OPTIONAL)
    list(APPEND ARILES_EXTRA_ADAPTERS "boost_optional")
endif(ARILES_ADAPTER_BOOST_OPTIONAL)
if (ARILES_ADAPTER_BETTER_ENUMS)
    list(APPEND ARILES_EXTRA_ADAPTERS "better_enums")
endif(ARILES_ADAPTER_BETTER_ENUMS)
if (ARILES_ADAPTER_ROSMSG)
    list(APPEND ARILES_EXTRA_ADAPTERS "rosmsg")
endif(ARILES_ADAPTER_ROSMSG)


#####################################
## Settings
#####################################

add_custom_target( "${PROJECT_NAME}" )

##
## default options
##
if (ARILES_VERBOSE)
    set (CMAKE_VERBOSE_MAKEFILE     ON)
endif(ARILES_VERBOSE)
set (CMAKE_RULE_MESSAGES        OFF CACHE BOOL "")


if(CMAKE_TOOLCHAIN_FILE)
    if(NOT IS_ABSOLUTE ${CMAKE_TOOLCHAIN_FILE})
        set(CMAKE_TOOLCHAIN_FILE "${PROJECT_SOURCE_DIR}/${CMAKE_TOOLCHAIN_FILE}")
    endif()
endif(CMAKE_TOOLCHAIN_FILE)


##
## compilation flags
##

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
if(NOT CMAKE_BUILD_TYPE)
    set (CMAKE_BUILD_TYPE Debug)
endif (NOT CMAKE_BUILD_TYPE)

set (CMAKEUT_CPP_SANITIZERS ${ARILES_CPP_SANITIZERS})
cmakeut_compiler_flags("${ARILES_CPP_STANDARD}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKEUT_CXX_FLAGS}")


set (RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set (LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
set (ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

# AS. 'CMAKE_' prefix makes it default for all targets.
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

set(DEB_ARCHIVE_FILES
    "./AUTHORS.md"
    "./CMakeLists.txt"
    "./LICENSE"
    "./Makefile"
    "./README.md"
    "./cmake"
    "./cpput")



#####################################
## Dependencies
#####################################

# Boost
find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
list(APPEND ARILES_CORE_DEPENDENCY_INCLUDES "${Boost_INCLUDE_DIRS}")


##
## Optional dependencies
##

if (ARILES_ADAPTER_EIGEN)
    find_package(Eigen3 REQUIRED)
    list(APPEND ARILES_CORE_DEPENDENCY_INCLUDES "${EIGEN3_INCLUDE_DIR}")
endif(ARILES_ADAPTER_EIGEN)

if (ARILES_ADAPTER_BOOST_POINTER)
    # BOOST is a mandatory dependency
endif(ARILES_ADAPTER_BOOST_POINTER)
if (ARILES_ADAPTER_BOOST_OPTIONAL)
    # BOOST is a mandatory dependency
endif(ARILES_ADAPTER_BOOST_OPTIONAL)



#####################################
## Ariles headers
#####################################

if (NOT ${ARILES_ENABLE_CORE} STREQUAL "OFF")
    set(ARILES_CORE_BUILD_INCLUDES "${PROJECT_BINARY_DIR}/include/core/")


    # --------------
    # build config
    # --------------
    set(ARILES_BUILD_CONFIG
    "#pragma once
    // __func__ ?
    #cmakedefine CMAKEUT_COMPILER_SUPPORTS_FUNC_
    // __FUNCTION__ ?
    #cmakedefine CMAKEUT_COMPILER_SUPPORTS_FUNCTION_\n")

    string(CONFIGURE "${ARILES_BUILD_CONFIG}" ARILES_BUILD_CONFIG)
    file(WRITE "${ARILES_CORE_BUILD_INCLUDES}/${PROJECT_NAME}/internal/build_config.h" "${ARILES_BUILD_CONFIG}")
    # --------------


    # --------------
    # core & adapters headers
    # --------------
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${PROJECT_SOURCE_DIR}/include"
        "${ARILES_CORE_BUILD_INCLUDES}"
    )

    foreach(ARILES_EXTRA_ADAPTER ${ARILES_EXTRA_ADAPTERS})
        execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${PROJECT_SOURCE_DIR}/extra_adapters/${ARILES_EXTRA_ADAPTER}"
            "${ARILES_CORE_BUILD_INCLUDES}/${PROJECT_NAME}/adapters/"
        )
    endforeach()
    # --------------


    # --------------
    # cpput
    # --------------
    option(CPPUT_EMBEDDED "" ON)
    set(CPPUT_EMBEDDED_ID "ARILES2"  CACHE STRING "")
    set(CPPUT_EMBEDDED_COPY_TO_DIR "${ARILES_CORE_BUILD_INCLUDES}/${PROJECT_NAME}/internal/"  CACHE STRING "")
    set(CPPUT_SELECT_HEADERS "config.h;exception.h;floating_point_utils.h;visibility.h;misc.h;trace.h"  CACHE STRING "")
    add_subdirectory(cpput)
    # --------------

    list(APPEND DEB_ARCHIVE_FILES "./include" "./extra_adapters")
endif()



#####################################
## Package config files & Installation
#####################################

if (${ARILES_ENABLE_CORE} STREQUAL "ON")
    cpack_add_component("core"
        DISPLAY_NAME "Ariles core"
        DESCRIPTION "Core headers and STL adapters"
        ARCHIVE_FILE "${CPACK_PACKAGE_NAME}-core-${CPACK_PACKAGE_VERSION}")


    set(ARILES_COMPONENT_NAME "core")
    set(ariles_LIBRARIES "${PROJECT_NAME}-${ARILES_COMPONENT_NAME}_LIBRARIES")
    set(ariles_INCLUDE_DIRS "${PROJECT_NAME}-${ARILES_COMPONENT_NAME}_INCLUDE_DIRS")
    set(ariles_LIBRARY_DIRS "${PROJECT_NAME}-${ARILES_COMPONENT_NAME}_LIBRARY_DIRS")


    set(ARILES_LIBRARIES            "")
    set(ARILES_INCLUDES             "${CMAKE_INSTALL_PREFIX}/include")
    set(ARILES_DEPENDENCY_INCLUDES  "${ARILES_CORE_DEPENDENCY_INCLUDES}")
    set(ARILES_LIBRARY_DIRS         "")


    list(REMOVE_DUPLICATES ARILES_INCLUDES)
    list(REMOVE_DUPLICATES ARILES_DEPENDENCY_INCLUDES)
    list(REMOVE_DUPLICATES ARILES_LIBRARIES)
    list(REMOVE_ITEM ARILES_INCLUDES "")
    list(REMOVE_ITEM ARILES_DEPENDENCY_INCLUDES "")
    list(REMOVE_ITEM ARILES_LIBRARIES "")

    configure_package_config_file(  "cmake/arilesConfig.cmake.in"
                                    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-${ARILES_COMPONENT_NAME}Config.cmake"
                                    INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${ARILES_COMPONENT_NAME}/"
                                    PATH_VARS ARILES_INCLUDES
                                    NO_SET_AND_CHECK_MACRO
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
    install (FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-${ARILES_COMPONENT_NAME}Config.cmake"
             DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${ARILES_COMPONENT_NAME}/"
             COMPONENT ${ARILES_COMPONENT_NAME})

    write_basic_package_version_file(
        ${PROJECT_BINARY_DIR}/${PROJECT_NAME}-${ARILES_COMPONENT_NAME}ConfigVersion.cmake
        VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
        COMPATIBILITY SameMajorVersion)
    install (FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-${ARILES_COMPONENT_NAME}ConfigVersion.cmake"
             DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${ARILES_COMPONENT_NAME}/"
             COMPONENT ${ARILES_COMPONENT_NAME})


    if(ARILES_PKGCONFIG_INSTALL_PATH)
        string(REPLACE ";" " -I" ARILES_INCLUDES_FLAGS "${ARILES_INCLUDES}")
        set(ARILES_INCLUDES_FLAGS "-I${ARILES_INCLUDES_FLAGS}")
        string(REPLACE ";" " -l" ARILES_LIBRARIES_FLAGS "${ARILES_LIBRARIES}")
        set(ARILES_LIBRARIES_FLAGS "-l${ARILES_LIBRARIES_FLAGS}")
        configure_file("cmake/ariles.pc.in"             "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc" @ONLY)

        install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}.pc"
                DESTINATION "${ARILES_PKGCONFIG_INSTALL_PATH}"
                COMPONENT ${ARILES_COMPONENT_NAME})
    endif()


    install (DIRECTORY "${ARILES_CORE_BUILD_INCLUDES}/"
             DESTINATION "${ARILES_INCLUDES}/"
             COMPONENT "${ARILES_COMPONENT_NAME}")

    set(DEB_CMAKE_FLAGS_core "-DARILES_ENABLE_CORE=ON")
    set(DEB_core_DEPENDS "libboost-all-dev;libeigen3-dev")
else()
    if (${ARILES_ENABLE_CORE} STREQUAL "OFF")
        find_package(ariles2-core REQUIRED)
    endif()
endif()



#####################################
## visitors
#####################################

set(VISITOR_TARGET_PREFIX TGT_visitor)

foreach(ARILES_VISITORS_DIR ${ARILES_VISITORS_DIRS})
    cmakeut_list_filenames("${ARILES_VISITORS_DIR}"   "ARILES_VISITORS_TMP" DIRS_WITH_CMAKELISTS)

    foreach(ARILES_VISITOR ${ARILES_VISITORS_TMP})
        if (NOT ${ARILES_VISITOR_${ARILES_VISITOR}} STREQUAL "OFF")
            add_custom_target(${VISITOR_TARGET_PREFIX}_${ARILES_VISITOR})
            include("${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/CMakeLists.txt")

            list(APPEND DEB_ARCHIVE_FILES "${PROJECT_SOURCE_DIR}/extra_visitors/${ARILES_VISITOR}/")
        endif()
    endforeach(ARILES_VISITOR)
endforeach(ARILES_VISITORS_DIR)



#####################################
## tests
#####################################

if (ARILES_BUILD_REGRESSION_TESTS)
    enable_testing()
    add_subdirectory ("${PROJECT_SOURCE_DIR}/tests")
endif()



#####################################
## CPack
#####################################
include(CPack)


set(CPACK_DEBIAN_RESOURCE_FILE_CHANGELOG "")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Alexander Sherikov <asherikov@yandex.com>")

if(ARILES_DEB_TARGETS)
    set(DEB_UBUNTU_CODENAMES ${ARILES_DEB_TARGETS}) # precise sausy trusty

    set(DEB_COMMON_CMAKE_ARGS
        "-DARILES_CPP_STANDARD=${ARILES_CPP_STANDARD} \
         -DARILES_BUILD_REGRESSION_TESTS=${ARILES_BUILD_REGRESSION_TESTS} \
         -DARILES_VISITORS_DEFAULT_MODE=OFF")

    include(pkg_debian)
endif()
