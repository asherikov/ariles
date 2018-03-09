# General parameters
cmake_minimum_required (VERSION 2.8.12)
project (ariles CXX)


include(FindPkgConfig)
include(ExternalProject)
include(CMakeParseArguments)


set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

include(ariles_copy_dir_if_exists)
include(ariles_copy_file_if_exists)
include(ariles_add_external_git_project)
include(ariles_list_filenames)
include(ariles_dump_variables)



#####################################
## Options (definitions & defaults)
#####################################

# bridges
set    (ARILES_BRIDGES_DEFAULT_MODE     "BUILD" CACHE STRING "ON = enable, OFF = disable, BUILD = build from sources")
option (ARILES_INSTALL_BRIDGES          "Install bridges on the system." OFF)

# extra adapters
option (ARILES_ADAPTER_EIGEN            "Enable eigen support." ON)

# build parameters
set    (ARILES_CPP_STANDARD             "c++98" CACHE STRING "C++ standard version")
option (ARILES_VERBOSE                  "Verbose build"                     ON)
option (ARILES_BUILD_REGRESSION_TESTS   "Build regression tests."           ON)


#####################################

# Gather all cached options controlling compilation of the modules to unset
# those that are not valid anymore.
get_cmake_property(CACHED_VARIABLES CACHE_VARIABLES)
set(ARILES_CACHED_BRIDGES "")
foreach (CACHED_VARIABLE ${CACHED_VARIABLES})
    if (CACHED_VARIABLE MATCHES "ARILES_BRIDGE_.*")
        list(APPEND ARILES_CACHED_BRIDGES ${CACHED_VARIABLE})
    endif()
endforeach(CACHED_VARIABLE)


set(ARILES_BRIDGES_DIRS "${PROJECT_SOURCE_DIR}/bridges/")
set(ARILES_BRIDGES "")
set(ARILES_ENABLED_BRIDGES "")
foreach (ARILES_BRIDGES_DIR ${ARILES_BRIDGES_DIRS})
    ariles_list_filenames("${ARILES_BRIDGES_DIR}"  "ARILES_BRIDGES_TMP" DIRS_WITH_CMAKELISTS)
    list(APPEND ARILES_BRIDGES ${ARILES_BRIDGES_TMP})

    foreach(ARILES_BRIDGE ${ARILES_BRIDGES_TMP})
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_DIR  "${ARILES_BRIDGES_DIR}/${ARILES_BRIDGE}")

        list(REMOVE_ITEM ARILES_CACHED_BRIDGES "ARILES_BRIDGE_${ARILES_BRIDGE}")
        if (DEFINED ARILES_BRIDGE_${ARILES_BRIDGE})
            set    ("ARILES_BRIDGE_${ARILES_BRIDGE}" "${ARILES_BRIDGE_${ARILES_BRIDGE}}"
                    CACHE STRING "Bridge '${ARILES_BRIDGE} (ON / OFF / BUILD)")

            if (NOT ${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "OFF")
                list(APPEND ARILES_ENABLED_BRIDGES "${ARILES_BRIDGE}")
            endif()
        else()
            set    ("ARILES_BRIDGE_${ARILES_BRIDGE}" "${ARILES_BRIDGES_DEFAULT_MODE}"
                    CACHE STRING "Bridge '${ARILES_BRIDGE} (ON / OFF / BUILD)")

            if (NOT ${ARILES_BRIDGES_DEFAULT_MODE} STREQUAL "OFF")
                list(APPEND ARILES_ENABLED_BRIDGES "${ARILES_BRIDGE}")
            endif()
        endif()


        if (ARILES_VERBOSE)
            message("----------------------BRIDGE----------------------")
            message("Name: ${ARILES_BRIDGE}")
            message("Path: ${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}")
            message("Enabled: ${ARILES_BRIDGE_${ARILES_BRIDGE}}")
            message("Enable/disable option name: ARILES_BRIDGE_${ARILES_BRIDGE}")
            message("--------------------------------------------------")
        endif(ARILES_VERBOSE)
    endforeach(ARILES_BRIDGE)
endforeach(ARILES_BRIDGES_DIR)



#####################################
## Settings
#####################################

add_custom_target( "${CMAKE_PROJECT_NAME}" )

##
## default options
##
if (ARILES_VERBOSE)
    set (CMAKE_VERBOSE_MAKEFILE     ON)
endif(ARILES_VERBOSE)
set (CMAKE_RULE_MESSAGES        OFF CACHE BOOL "")


##
## paths
##
set (ARILES_INCLUDE_DIR "${PROJECT_BINARY_DIR}/include/")


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


set (RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set (LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
set (ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

# AS. 'CMAKE_' prefix makes it default for all targets.
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")

# This will go to the configuration file for projects, which use ariles.
set (ARILES_OPTIONAL_LIBRARIES "")
set (ARILES_OPTIONAL_INCLUDES "")
set (ARILES_MANDATORY_LIBRARIES "")
set (ARILES_MANDATORY_INCLUDES "")
set (ARILES_DEFINES   "")



#####################################
## Ariles headers
#####################################

add_custom_target( TGT_ariles_copy_headers ALL )

ariles_copy_dir_if_exists(TGT_ariles_copy_headers          "ariles"
                          "${PROJECT_SOURCE_DIR}/include"  "${ARILES_INCLUDE_DIR}")
if (ARILES_ADAPTER_EIGEN)
    ariles_copy_dir_if_exists(TGT_ariles_copy_headers                       "adapter_eigen"
                              "${PROJECT_SOURCE_DIR}/extra_adapters/eigen"  "${ARILES_INCLUDE_DIR}/ariles/adapters/")
endif(ARILES_ADAPTER_EIGEN)

# All headers are collected in one directory
include_directories ("${PROJECT_BINARY_DIR}/include/")



#####################################
## Dependencies
#####################################

add_custom_target( TGT_fetch_bridges )
set(ARILES_TARGET_FETCH_BRIDGES    TGT_fetch_bridges CACHE INTERNAL "")

# Boost
find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
list(APPEND ARILES_MANDATORY_INCLUDES "${Boost_INCLUDE_DIRS}")


##
## Optional dependencies
##

set(BRIDGE_TARGET_PREFIX TGT_bridge)

if (ARILES_ADAPTER_EIGEN)
    find_package(Eigen3 REQUIRED)
    list(APPEND ARILES_MANDATORY_INCLUDES "${EIGEN3_INCLUDE_DIR}")
endif(ARILES_ADAPTER_EIGEN)

set (ARILES_BRIDGES_BINARY_DIR "${PROJECT_BINARY_DIR}/bridges/")
if (ARILES_INSTALL_BRIDGES)
    set(ARILES_BRIDGES_INSTALL_PREFIX   "${CMAKE_INSTALL_PREFIX}")
else()
    set(ARILES_BRIDGES_INSTALL_PREFIX   "${PROJECT_BINARY_DIR}/bridges_install/")
endif (ARILES_INSTALL_BRIDGES)


# TODO We use this to set CMAKE_INSTALL_LIBDIR in order to prevent creation of
# separate directories for different architectures. This is potentially unsafe
# if ARILES_INSTALL_BRIDGES = ON.
#
# It seems that the only robust solution is to work with ariles as an external
# project, where we would be able to use find_library().
set(ARILES_BRIDGES_LIBDIR       "${ARILES_BRIDGES_INSTALL_PREFIX}/lib")

set(ARILES_BRIDGES_INCLUDE_DIR  "${ARILES_BRIDGES_INSTALL_PREFIX}/include")
include_directories (BEFORE SYSTEM     "${ARILES_BRIDGES_INCLUDE_DIR}")


set(BRIDGES_ARGS
    --no-warn-unused-cli
    -DCMAKE_INSTALL_PREFIX=${ARILES_BRIDGES_INSTALL_PREFIX}
    -DCMAKE_INSTALL_LIBDIR=${ARILES_BRIDGES_LIBDIR}
    -DCMAKE_VERBOSE_MAKEFILE=${CMAKE_VERBOSE_MAKEFILE}
    -DCMAKE_BUILD_TYPE=Release)


if(CMAKE_TOOLCHAIN_FILE)
    if(NOT IS_ABSOLUTE ${CMAKE_TOOLCHAIN_FILE})
        set(CMAKE_TOOLCHAIN_FILE "${PROJECT_SOURCE_DIR}/${CMAKE_TOOLCHAIN_FILE}")
    endif()

    set(BRIDGES_ARGS
        ${BRIDGES_ARGS}
        "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
endif(CMAKE_TOOLCHAIN_FILE)


# bridges
foreach(ARILES_BRIDGES_DIR ${ARILES_BRIDGES_DIRS})
    ariles_list_filenames("${ARILES_BRIDGES_DIR}"   "ARILES_BRIDGES_TMP" DIRS_WITH_CMAKELISTS)

    foreach(ARILES_BRIDGE ${ARILES_BRIDGES_TMP})
        if (NOT ${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "OFF")
            set (${ARILES_BRIDGE}_INTERFACE_DIR  "${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/interface")
            add_subdirectory("${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}" "${ARILES_BRIDGES_BINARY_DIR}/${ARILES_BRIDGE}")

            if (ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS)
                list(APPEND ARILES_OPTIONAL_LIBRARIES  "${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS}")
            endif()

            if (ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES)
                list(APPEND ARILES_OPTIONAL_INCLUDES    "${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES}")
                include_directories("${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES}")
            endif()

            if (ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS)
                link_directories("${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS}")
            endif()
        endif()
    endforeach(ARILES_BRIDGE)
endforeach(ARILES_BRIDGES_DIR)



#####################################
## Variables / options for ariles
#####################################


set (CMAKE_CXX_FLAGS            "${CMAKE_CXX_FLAGS} -fPIC -pedantic -Wall ${ARILES_CXX_FLAGS}")
set (CMAKE_CXX_FLAGS            "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set (CMAKE_CXX_FLAGS            "${CMAKE_CXX_FLAGS} -std=${ARILES_CPP_STANDARD}")


#####################################
## Tests
#####################################

if (ARILES_BUILD_REGRESSION_TESTS)
    enable_testing()
    add_subdirectory ("${PROJECT_SOURCE_DIR}/tests")
endif()



#####################################
## Configuration files
#####################################

list(APPEND ARILES_LIBRARIES "${ARILES_OPTIONAL_LIBRARIES}" "${ARILES_MANDATORY_LIBRARIES}")
list(APPEND ARILES_INCLUDES "${ARILES_OPTIONAL_INCLUDES}" "${ARILES_MANDATORY_INCLUDES}")
if (ARILES_LIBRARIES)
    set (ARILES_SHARED_GNU_LINKER_FLAGS "-Wl,--exclude-libs")

    foreach(ARILES_LIBRARY ${ARILES_LIBRARIES})
        set(ARILES_SHARED_GNU_LINKER_FLAGS "${ARILES_SHARED_GNU_LINKER_FLAGS},${ARILES_LIBRARY}")
    endforeach(ARILES_LIBRARY)
endif (ARILES_LIBRARIES)

configure_file("cmake/arilesConfig.cmake.in"    "${PROJECT_BINARY_DIR}/arilesConfig.cmake")


#---
set(ARILES_BUILD_CONFIG "")

foreach(ARILES_BRIDGE ${ARILES_BRIDGES})
    if (NOT ${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "OFF")
        set(ARILES_BUILD_CONFIG "${ARILES_BUILD_CONFIG}#define ARILES_BRIDGE_${ARILES_BRIDGE}\n")
        if(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER)
            set(ARILES_BUILD_CONFIG
                "${ARILES_BUILD_CONFIG}#define ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER \"${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER}\"\n")
        endif()
    endif()
endforeach(ARILES_BRIDGE)

string(CONFIGURE "${ARILES_BUILD_CONFIG}" ARILES_BUILD_CONFIG)
file(WRITE "${ARILES_INCLUDE_DIR}/ariles/internal/build_config.h" "${ARILES_BUILD_CONFIG}")
#---


#---
set(ARILES_HEADER_ALL   "#pragma once\n")
set(ARILES_HEADER_ALL   "${ARILES_HEADER_ALL}#include \"internal/build_config.h\"\n")

foreach(ARILES_BRIDGE ${ARILES_BRIDGES})
    if (NOT ${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "OFF")
        set(ARILES_HEADER_ALL "${ARILES_HEADER_ALL}#include \"formats/${ARILES_BRIDGE}.h\"\n")
    endif()
endforeach(ARILES_BRIDGE)

set(ARILES_HEADER_ALL   "${ARILES_HEADER_ALL}#include \"adapters_all.h\"\n")
set(ARILES_HEADER_ALL   "${ARILES_HEADER_ALL}#include \"ariles.h\"\n")

file(WRITE "${ARILES_INCLUDE_DIR}/ariles/ariles_all.h" "${ARILES_HEADER_ALL}")
#---


#####################################
## Installation
#####################################

install (DIRECTORY "${ARILES_INCLUDE_DIR}"
         DESTINATION "${CMAKE_INSTALL_PREFIX}/include/")

install (FILES "${PROJECT_BINARY_DIR}/arilesConfig.cmake"
         DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ariles/")
