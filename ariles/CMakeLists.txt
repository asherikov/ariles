# General parameters
cmake_minimum_required (VERSION 2.8.12)
project (ariles CXX)
set(ARILES_HOMEPAGE http://asherikov.github.com/ariles/)
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 3)
set(PROJECT_VERSION_PATCH 2)


include(FindPkgConfig)
include(ExternalProject)
include(CMakeParseArguments)
include(CPackComponent)
include(CMakePackageConfigHelpers)


set (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

include(cmakeut_copy_dir_if_exists)
include(cmakeut_copy_file_if_exists)
include(cmakeut_add_external_git_project)
include(cmakeut_list_filenames)
include(cmakeut_dump_variables)
include(cmakeut_compiler_flags)

include (cmakeut_detect_func_macro)
cmakeut_detect_func_macro()



#####################################
## Options (definitions & defaults)
#####################################

option (ARILES_ENABLE_CORE              "Install core (if disable only selected bridges are installed)." ON)
option (ARILES_ENABLE_CORE_ALL_HEADER   "Generate ariles_all.h header (not suitable for packages)" ON)

# bridges
set    (ARILES_BRIDGES_DEFAULT_MODE     "BUILD" CACHE STRING "ON = enable, OFF = disable, BUILD = build from sources")
option (ARILES_INSTALL_BRIDGES          "Install bridges on the system (when mode = BUILD)." OFF)

# extra adapters & options
set (ARILES_DEFAULT_CONFIGURABLE_FLAGS  "" CACHE STRING "Override default flags")
set (ARILES_DEFAULT_FLOAT_VALUE         "std::numeric_limits<float>::signaling_NaN()" CACHE STRING "Default value for floats")
set (ARILES_DEFAULT_DOUBLE_VALUE        "std::numeric_limits<double>::signaling_NaN()" CACHE STRING "Default value for doubles")

option (ARILES_ADAPTER_EIGEN            "Enable eigen support." ON)
option (ARILES_ADAPTER_BOOST_POINTER    "Enable boost pointer support." ON)
option (ARILES_ADAPTER_BOOST_OPTIONAL   "Enable boost optional support." ON)
option (ARILES_ADAPTER_BETTER_ENUMS     "Enable support for beter enums." ON)

# build parameters
set    (ARILES_CPP_STANDARD             "c++98" CACHE STRING "C++ standard version")
option (ARILES_VERBOSE                  "Verbose build"                     ON)
option (ARILES_BUILD_REGRESSION_TESTS   "Build regression tests."           ON)
set    (ARILES_DEB_TARGETS              "" CACHE STRING "Enable debian package generation.")

# install parameters
set    (ARILES_PKGCONFIG_INSTALL_PATH   "" CACHE STRING "pkg-config pc-file install path, not installed if empty")



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


set(CPACK_DEBIAN_PACKAGE_HOMEPAGE   "https://asherikov.github.io/ariles/")
set(CPACK_DEBIAN_PACKAGE_PRIORITY   "optional")
set(CPACK_DEBIAN_PACKAGE_SECTION    "devel")


#####################################

if (NOT ${ARILES_ENABLE_CORE})
    # tests cannot be built if the core is not installed
    set(ARILES_BUILD_REGRESSION_TESTS   OFF)

    # adapters are installed only with core
    set(ARILES_ADAPTER_EIGEN            OFF)
    set(ARILES_ADAPTER_BOOST_POINTER    OFF)
    set(ARILES_ADAPTER_BOOST_OPTIONAL   OFF)
    set(ARILES_ADAPTER_BETTER_ENUMS     OFF)
endif()



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
    cmakeut_list_filenames("${ARILES_BRIDGES_DIR}"  "ARILES_BRIDGES_TMP" DIRS_WITH_CMAKELISTS)
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
set (ARILES_CORE_INCLUDES "${CMAKE_INSTALL_PREFIX}/include")

set(DEB_ARCHIVE_FILES
    "./AUTHORS.md"
    "./CMakeLists.txt"
    "./LICENSE"
    "./Makefile"
    "./README.md"
    "./cmake"
    "./cpput")


#####################################
## Ariles headers
#####################################


# --------------
# cpput
# --------------
option(CPPUT_EMBEDDED "" ON)
set(CPPUT_EMBEDDED_ID "ARILES"  CACHE STRING "")
set(CPPUT_EMBEDDED_COPY_TO_DIR "${PROJECT_SOURCE_DIR}/include/ariles/internal/"  CACHE STRING "")
set(CPPUT_SELECT_HEADERS "config.h;better_enum.h;exception.h;floating_point_utils.h;visibility.h;misc.h;flags.h"  CACHE STRING "")
add_subdirectory(cpput)
# --------------


add_custom_target( TGT_ariles_copy_headers ALL )

# core & adapters headers are installed only in MIX or CORE mode.
if (${ARILES_ENABLE_CORE})
    cmakeut_copy_dir_if_exists(TGT_ariles_copy_headers          "ariles"
                              "${PROJECT_SOURCE_DIR}/include"  "${ARILES_INCLUDE_DIR}/core/")

    foreach(ARILES_EXTRA_ADAPTER ${ARILES_EXTRA_ADAPTERS})
        cmakeut_copy_dir_if_exists(
            TGT_ariles_copy_headers                                         "adapter_${ARILES_EXTRA_ADAPTER}"
            "${PROJECT_SOURCE_DIR}/extra_adapters/${ARILES_EXTRA_ADAPTER}"  "${ARILES_INCLUDE_DIR}/core/ariles/adapters/")
    endforeach()

    include_directories("${ARILES_INCLUDE_DIR}/core/")
    list(APPEND DEB_ARCHIVE_FILES "./include" "./extra_adapters")
endif()



#####################################
## Dependencies
#####################################

add_custom_target( TGT_fetch_bridges )
set(ARILES_TARGET_FETCH_BRIDGES    TGT_fetch_bridges CACHE INTERNAL "")

# Boost
find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
list(APPEND ARILES_CORE_DEPENDENCY_INCLUDES "${Boost_INCLUDE_DIRS}")


##
## Optional dependencies
##

set(BRIDGE_TARGET_PREFIX TGT_bridge)

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

set(ARILES_BRIDGES_INCLUDE_DIR      "${ARILES_BRIDGES_INSTALL_PREFIX}/include")
include_directories (BEFORE SYSTEM  "${ARILES_BRIDGES_INCLUDE_DIR}")


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
    cmakeut_list_filenames("${ARILES_BRIDGES_DIR}"   "ARILES_BRIDGES_TMP" DIRS_WITH_CMAKELISTS)

    foreach(ARILES_BRIDGE ${ARILES_BRIDGES_TMP})
        if (NOT ${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "OFF")
            set (${ARILES_BRIDGE}_INTERFACE_DIR  "${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/interface")
            set (ARILES_BRIDGE_INCLUDE_DIR "${ARILES_INCLUDE_DIR}/${ARILES_BRIDGE}/ariles/bridges/")
            include_directories("${ARILES_INCLUDE_DIR}/${ARILES_BRIDGE}")
            file(MAKE_DIRECTORY "${ARILES_BRIDGE_INCLUDE_DIR}")

            #add_subdirectory("${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}" "${ARILES_BRIDGES_BINARY_DIR}/${ARILES_BRIDGE}")
            include("${ARILES_BRIDGE_${ARILES_BRIDGE}_DIR}/CMakeLists.txt")

            if (ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES)
                include_directories(SYSTEM "${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES}")
            endif()

            if (ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS)
                link_directories("${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBRARY_DIRS}")
            endif()

            list(APPEND DEB_ARCHIVE_FILES "./bridges/${ARILES_BRIDGE}/interface")
            list(APPEND DEB_ARCHIVE_FILES "./bridges/${ARILES_BRIDGE}/CMakeLists.txt")
        endif()
    endforeach(ARILES_BRIDGE)
endforeach(ARILES_BRIDGES_DIR)



#####################################
## Variables / options for ariles
#####################################

cmakeut_compiler_flags("${ARILES_CPP_STANDARD}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKEUT_CXX_FLAGS}")


#####################################
## Tests
#####################################

if (ARILES_BUILD_REGRESSION_TESTS)
    set(ARILES_TESTS_CONFIG "")

    foreach(ARILES_EXTRA_ADAPTER ${ARILES_EXTRA_ADAPTERS})
        string(TOUPPER ${ARILES_EXTRA_ADAPTER} ARILES_EXTRA_ADAPTER_CAPITALIZED)
        set(ARILES_TESTS_CONFIG   "${ARILES_TESTS_CONFIG}#define ARILES_ADAPTER_${ARILES_EXTRA_ADAPTER_CAPITALIZED}\n")
    endforeach()


    foreach(ARILES_BRIDGE ${ARILES_BRIDGES})
        if (NOT ${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "OFF")
            set(ARILES_TESTS_CONFIG "${ARILES_TESTS_CONFIG}#define ARILES_BRIDGE_${ARILES_BRIDGE}\n")
        endif()
    endforeach(ARILES_BRIDGE)


    file(WRITE "${ARILES_INCLUDE_DIR}/tests/tests_config.h" "${ARILES_TESTS_CONFIG}")
    include_directories("${ARILES_INCLUDE_DIR}/tests")


    enable_testing()
    add_subdirectory ("${PROJECT_SOURCE_DIR}/tests")
endif()



#####################################
## Auto-generated headers
#####################################

if (${ARILES_ENABLE_CORE})
    # build config
    #---
    set(ARILES_BUILD_CONFIG
"#pragma once
// __func__ ?
#cmakedefine CMAKEUT_COMPILER_SUPPORTS_FUNC_
// __FUNCTION__ ?
#cmakedefine CMAKEUT_COMPILER_SUPPORTS_FUNCTION_\n
#cmakedefine ARILES_DEFAULT_FLOAT_VALUE ${ARILES_DEFAULT_FLOAT_VALUE}\n
#cmakedefine ARILES_DEFAULT_DOUBLE_VALUE ${ARILES_DEFAULT_DOUBLE_VALUE}\n")

    if (ARILES_DEFAULT_CONFIGURABLE_FLAGS)
        set(ARILES_BUILD_CONFIG   "${ARILES_BUILD_CONFIG}#define ARILES_DEFAULT_CONFIGURABLE_FLAGS ${ARILES_DEFAULT_CONFIGURABLE_FLAGS}\n")
    endif(ARILES_DEFAULT_CONFIGURABLE_FLAGS)

    string(CONFIGURE "${ARILES_BUILD_CONFIG}" ARILES_BUILD_CONFIG)
    file(WRITE "${ARILES_INCLUDE_DIR}/core/ariles/internal/build_config.h" "${ARILES_BUILD_CONFIG}")
    #---


    # adapters_all.h
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

    file(WRITE "${ARILES_INCLUDE_DIR}/core/ariles/adapters_all.h" "${ARILES_ADAPTERS_ALL}")
    #---


    # ariles_all.h
    #---
    if (${ARILES_ENABLE_CORE_ALL_HEADER})
        set(ARILES_HEADER_ALL   "#pragma once\n")
        set(ARILES_HEADER_ALL   "${ARILES_HEADER_ALL}#include \"internal/build_config.h\"\n")

        foreach(ARILES_BRIDGE ${ARILES_BRIDGES})
            if (NOT ${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "OFF")
                set(ARILES_HEADER_ALL "${ARILES_HEADER_ALL}#include <ariles/bridges/${ARILES_BRIDGE}.h>\n")
            endif()
        endforeach(ARILES_BRIDGE)

        set(ARILES_HEADER_ALL   "${ARILES_HEADER_ALL}#include \"ariles.h\"\n")
        set(ARILES_HEADER_ALL   "${ARILES_HEADER_ALL}#include \"adapters_all.h\"\n")

        file(WRITE "${ARILES_INCLUDE_DIR}/core/ariles/ariles_all.h" "${ARILES_HEADER_ALL}")
    endif()
    #---
endif()



#####################################
## Package config files & Installation
#####################################

if (${ARILES_ENABLE_CORE})
    cpack_add_component("core"
        DISPLAY_NAME "Ariles core"
        DESCRIPTION "Core headers and STL adapters"
        ARCHIVE_FILE "${CPACK_PACKAGE_NAME}-core-${CPACK_PACKAGE_VERSION}")


    set(ARILES_COMPONENT_NAME "core")
    set(ariles_LIBRARIES "ariles-${ARILES_COMPONENT_NAME}_LIBRARIES")
    set(ariles_INCLUDE_DIRS "ariles-${ARILES_COMPONENT_NAME}_INCLUDE_DIRS")
    set(ariles_LIBRARY_DIRS "ariles-${ARILES_COMPONENT_NAME}_LIBRARY_DIRS")

    set(ARILES_LIBRARIES            "")
    set(ARILES_INCLUDES             "${ARILES_CORE_INCLUDES}")
    set(ARILES_DEPENDENCY_INCLUDES  "${ARILES_CORE_DEPENDENCY_INCLUDES}")
    set(ARILES_LIBRARY_DIRS         "")

    if (ARILES_LIBRARIES)
        set (ARILES_SHARED_GNU_LINKER_FLAGS "-Wl,--exclude-libs")

        foreach(ARILES_LIBRARY ${ARILES_LIBRARIES})
            set(ARILES_SHARED_GNU_LINKER_FLAGS "${ARILES_SHARED_GNU_LINKER_FLAGS},${ARILES_LIBRARY}")
        endforeach(ARILES_LIBRARY)
    endif (ARILES_LIBRARIES)

    list(REMOVE_DUPLICATES ARILES_INCLUDES)
    list(REMOVE_DUPLICATES ARILES_DEPENDENCY_INCLUDES)
    list(REMOVE_DUPLICATES ARILES_LIBRARIES)
    list(REMOVE_ITEM ARILES_INCLUDES "")
    list(REMOVE_ITEM ARILES_DEPENDENCY_INCLUDES "")
    list(REMOVE_ITEM ARILES_LIBRARIES "")

    configure_package_config_file(  "cmake/arilesConfig.cmake.in"
                                    "${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT_NAME}Config.cmake"
                                    INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ariles-${ARILES_COMPONENT_NAME}/"
                                    PATH_VARS ARILES_INCLUDES
                                    NO_SET_AND_CHECK_MACRO
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
    install (FILES "${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT_NAME}Config.cmake"
             DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ariles-${ARILES_COMPONENT_NAME}/"
             COMPONENT ${ARILES_COMPONENT_NAME})

    write_basic_package_version_file(
        ${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT_NAME}ConfigVersion.cmake
        VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
        COMPATIBILITY SameMajorVersion)
    install (FILES "${PROJECT_BINARY_DIR}/ariles-${ARILES_COMPONENT_NAME}ConfigVersion.cmake"
             DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ariles-${ARILES_COMPONENT_NAME}/"
             COMPONENT ${ARILES_COMPONENT_NAME})


    if(ARILES_PKGCONFIG_INSTALL_PATH)
        string(REPLACE ";" " -I" ARILES_INCLUDES_FLAGS "${ARILES_INCLUDES}")
        set(ARILES_INCLUDES_FLAGS "-I${ARILES_INCLUDES_FLAGS}")
        string(REPLACE ";" " -l" ARILES_LIBRARIES_FLAGS "${ARILES_LIBRARIES}")
        set(ARILES_LIBRARIES_FLAGS "-l${ARILES_LIBRARIES_FLAGS}")
        configure_file("cmake/ariles.pc.in"             "${PROJECT_BINARY_DIR}/ariles.pc" @ONLY)

        install(FILES "${PROJECT_BINARY_DIR}/ariles.pc"
                DESTINATION "${ARILES_PKGCONFIG_INSTALL_PATH}"
                COMPONENT ${ARILES_COMPONENT_NAME})
    endif()


    install (DIRECTORY "${ARILES_INCLUDE_DIR}/${ARILES_COMPONENT_NAME}/"
             DESTINATION "${CMAKE_INSTALL_PREFIX}/include/"
             COMPONENT "${ARILES_COMPONENT_NAME}")

    set(DEB_CMAKE_FLAGS_core "-DARILES_ENABLE_CORE=ON")
    set(DEB_core_DEPENDS "libboost-all-dev;libeigen3-dev")
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
         -DARILES_ENABLE_CORE_ALL_HEADER=${ARILES_ENABLE_CORE_ALL_HEADER} \
         -DARILES_BUILD_REGRESSION_TESTS=${ARILES_BUILD_REGRESSION_TESTS} \
         -DARILES_BRIDGES_DEFAULT_MODE=OFF")

    include(pkg_debian)
endif()
