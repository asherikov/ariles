# General parameters
cmake_minimum_required (VERSION 2.8.12)
project (ariles CXX)
set(ARILES_HOMEPAGE http://asherikov.github.com/ariles/)
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 0)


include(FindPkgConfig)
include(ExternalProject)
include(CMakeParseArguments)


set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

include(ariles_copy_dir_if_exists)
include(ariles_copy_file_if_exists)
include(ariles_add_external_git_project)
include(ariles_list_filenames)
include(ariles_dump_variables)

include (ariles_detect_func_macro)
ariles_detect_func_macro()



#####################################
## Options (definitions & defaults)
#####################################

# bridges
set    (ARILES_BRIDGES_DEFAULT_MODE     "BUILD" CACHE STRING "ON = enable, OFF = disable, BUILD = build from sources")
option (ARILES_INSTALL_BRIDGES          "Install bridges on the system." OFF)

# extra adapters & options
option (ARILES_ENABLE_SLOPPY_MAP        "Enable sloppy std::map support" OFF)

option (ARILES_ADAPTER_EIGEN            "Enable eigen support." ON)
option (ARILES_ADAPTER_BOOST_POINTER    "Enable boost pointer support." ON)
option (ARILES_ADAPTER_BOOST_OPTIONAL   "Enable boost optional support." ON)
option (ARILES_ADAPTER_BETTER_ENUMS     "Enable support for beter enums." ON)

# build parameters
set    (ARILES_CPP_STANDARD             "c++98" CACHE STRING "C++ standard version")
option (ARILES_VERBOSE                  "Verbose build"                     ON)
option (ARILES_BUILD_REGRESSION_TESTS   "Build regression tests."           ON)

# install parameters
set    (ARILES_PKGCONFIG_INSTALL_PATH   "" CACHE STRING "pkg-config pc-file install path, not installed if empty")


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
set (ARILES_BRIDGE_INCLUDE_DIR "${ARILES_INCLUDE_DIR}/ariles/bridges/")


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
set (ARILES_MANDATORY_INCLUDES "${CMAKE_INSTALL_PREFIX}/include")
set (ARILES_DEFINES   "")



#####################################
## Ariles headers
#####################################

add_custom_target( TGT_ariles_copy_headers ALL )

ariles_copy_dir_if_exists(TGT_ariles_copy_headers          "ariles"
                          "${PROJECT_SOURCE_DIR}/include"  "${ARILES_INCLUDE_DIR}")

foreach(ARILES_EXTRA_ADAPTER ${ARILES_EXTRA_ADAPTERS})
    ariles_copy_dir_if_exists(
        TGT_ariles_copy_headers                                         "adapter_${ARILES_EXTRA_ADAPTER}"
        "${PROJECT_SOURCE_DIR}/extra_adapters/${ARILES_EXTRA_ADAPTER}"  "${ARILES_INCLUDE_DIR}/ariles/adapters/")
endforeach()

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

if (ARILES_ADAPTER_BOOST_POINTER)
    # BOOST is a mandatory dependency
endif(ARILES_ADAPTER_BOOST_POINTER)
if (ARILES_ADAPTER_BOOST_OPTIONAL)
    # BOOST is a mandatory dependency
endif(ARILES_ADAPTER_BOOST_OPTIONAL)


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

list(REMOVE_DUPLICATES ARILES_INCLUDES)
list(REMOVE_ITEM ARILES_INCLUDES "")
list(REMOVE_ITEM ARILES_LIBRARIES "")

configure_file("cmake/arilesConfig.cmake.in"    "${PROJECT_BINARY_DIR}/arilesConfig.cmake")

string(REPLACE ";" " -I" ARILES_INCLUDES_FLAGS "${ARILES_INCLUDES}")
set(ARILES_INCLUDES_FLAGS "-I${ARILES_INCLUDES_FLAGS}")
string(REPLACE ";" " -l" ARILES_LIBRARIES_FLAGS "${ARILES_LIBRARIES}")
set(ARILES_LIBRARIES_FLAGS "-l${ARILES_LIBRARIES_FLAGS}")
configure_file("cmake/ariles.pc.in"             "${PROJECT_BINARY_DIR}/ariles.pc" @ONLY)


#---
set(ARILES_BUILD_CONFIG
"#pragma once
// __func__ ?
#cmakedefine ARILES_COMPILER_SUPPORTS_FUNC_
// __FUNCTION__ ?
#cmakedefine ARILES_COMPILER_SUPPORTS_FUNCTION_\n")

foreach(ARILES_BRIDGE ${ARILES_BRIDGES})
    if (NOT ${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "OFF")
        set(ARILES_BUILD_CONFIG "${ARILES_BUILD_CONFIG}#define ARILES_BRIDGE_${ARILES_BRIDGE}\n")
        if(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER)
            set(ARILES_BUILD_CONFIG
                "${ARILES_BUILD_CONFIG}#define ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER \"${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDE_HEADER}\"\n")
        endif()
    endif()
endforeach(ARILES_BRIDGE)

if (ARILES_ENABLE_SLOPPY_MAP)
    set(ARILES_BUILD_CONFIG   "${ARILES_BUILD_CONFIG}#define ARILES_DEFAULT_CONFIGURABLE_FLAGS ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED\n")
endif(ARILES_ENABLE_SLOPPY_MAP)

foreach(ARILES_EXTRA_ADAPTER ${ARILES_EXTRA_ADAPTERS})
    string(TOUPPER ${ARILES_EXTRA_ADAPTER} ARILES_EXTRA_ADAPTER_CAPITALIZED)
    set(ARILES_BUILD_CONFIG   "${ARILES_BUILD_CONFIG}#define ARILES_ADAPTER_${ARILES_EXTRA_ADAPTER_CAPITALIZED}\n")
endforeach()

string(CONFIGURE "${ARILES_BUILD_CONFIG}" ARILES_BUILD_CONFIG)
file(WRITE "${ARILES_INCLUDE_DIR}/ariles/internal/build_config.h" "${ARILES_BUILD_CONFIG}")
#---


#---
set(ARILES_HEADER_ALL   "#pragma once\n")
set(ARILES_HEADER_ALL   "${ARILES_HEADER_ALL}#include \"internal/build_config.h\"\n")

foreach(ARILES_BRIDGE ${ARILES_BRIDGES})
    if (NOT ${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "OFF")
        set(ARILES_HEADER_ALL "${ARILES_HEADER_ALL}#include \"bridges/${ARILES_BRIDGE}.h\"\n")
    endif()
endforeach(ARILES_BRIDGE)

set(ARILES_HEADER_ALL   "${ARILES_HEADER_ALL}#include \"adapters_all.h\"\n")
set(ARILES_HEADER_ALL   "${ARILES_HEADER_ALL}#include \"ariles.h\"\n")

file(WRITE "${ARILES_INCLUDE_DIR}/ariles/ariles_all.h" "${ARILES_HEADER_ALL}")
#---


#---
set(ARILES_ADAPTERS_ALL   "#pragma once\n")
set(ARILES_ADAPTERS_ALL   "${ARILES_ADAPTERS_ALL}#include \"internal/helpers.h\"\n")

set(ARILES_ADAPTERS_ALL   "${ARILES_ADAPTERS_ALL}#include \"adapters/std_vector.h\"\n")
set(ARILES_ADAPTERS_ALL   "${ARILES_ADAPTERS_ALL}#include \"adapters/std_pair.h\"\n")
set(ARILES_ADAPTERS_ALL   "${ARILES_ADAPTERS_ALL}#include \"adapters/std_map.h\"\n")
set(ARILES_ADAPTERS_ALL   "${ARILES_ADAPTERS_ALL}#include \"adapters/pointer.h\"\n")

foreach(ARILES_EXTRA_ADAPTER ${ARILES_EXTRA_ADAPTERS})
    set(ARILES_ADAPTERS_ALL   "${ARILES_ADAPTERS_ALL}#include \"adapters/${ARILES_EXTRA_ADAPTER}.h\"\n")
endforeach()

file(WRITE "${ARILES_INCLUDE_DIR}/ariles/adapters_all.h" "${ARILES_ADAPTERS_ALL}")

#-

set(ARILES_ADAPTERS_INCLUDED   "#pragma once\n\
#ifdef ARILES_INCLUDED_ADAPTER_POINTER\n\
#   include \"pointer_impl.h\"\n\
#endif\n\
#ifdef ARILES_INCLUDED_ADAPTER_STD_VECTOR\n\
#   include \"std_vector_impl.h\"\n\
#endif\n\
\n\
#ifdef ARILES_INCLUDED_ADAPTER_STD_PAIR\n\
#   include \"std_pair_impl.h\"\n\
#endif\n\
\n\
#ifdef ARILES_INCLUDED_ADAPTER_STD_PAIR\n\
#   include \"std_map_impl.h\"\n\
#endif\n")

foreach(ARILES_EXTRA_ADAPTER ${ARILES_EXTRA_ADAPTERS})
set(ARILES_ADAPTERS_INCLUDED "${ARILES_ADAPTERS_INCLUDED}\n\
#ifdef ARILES_INCLUDED_ADAPTER_EIGEN\n\
#   include \"${ARILES_EXTRA_ADAPTER}_impl.h\"\n\
#endif\n")
endforeach()

file(WRITE "${ARILES_INCLUDE_DIR}/ariles/adapters/included.h" "${ARILES_ADAPTERS_INCLUDED}")

#---


#####################################
## Installation
#####################################

install (DIRECTORY "${ARILES_INCLUDE_DIR}"
         DESTINATION "${CMAKE_INSTALL_PREFIX}/include/")

install (FILES "${PROJECT_BINARY_DIR}/arilesConfig.cmake"
         DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ariles/")

if(ARILES_PKGCONFIG_INSTALL_PATH)
    install (FILES "${PROJECT_BINARY_DIR}/ariles.pc"
        DESTINATION "${ARILES_PKGCONFIG_INSTALL_PATH}")
endif()

