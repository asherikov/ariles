cmake_minimum_required(VERSION 2.8.12)
project(ariles_ros)

set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 1)
set(PROJECT_VERSION_PATCH 1)


option(ARILES_ROS_ENABLE_TESTS "Enable tests." OFF)

set(ARILES_DIR             "${CMAKE_CURRENT_SOURCE_DIR}/ariles")

set(CMAKE_VERBOSE_MAKEFILE ON)
#-----
# ARILES
set(ARILES_CPP_STANDARD         "c++11" CACHE STRING "")

set(ARILES_DEFAULT_CONFIGURABLE_FLAGS "ariles::ConfigurableFlags::SLOPPY_MAPS_IF_SUPPORTED | ariles::ConfigurableFlags::SLOPPY_PAIRS_IF_SUPPORTED" CACHE STRING "")
set(ARILES_BRIDGES_DEFAULT_MODE "OFF"   CACHE STRING "")
set(ARILES_BRIDGE_yaml_cpp      "ON"    CACHE STRING "")
set(ARILES_BRIDGE_ros           "ON"    CACHE STRING "")
set(ARILES_BRIDGE_octave        "ON"    CACHE STRING "")

set(ARILES_ADAPTER_EIGEN            "ON"    CACHE STRING "")
set(ARILES_ADAPTER_BOOST_POINTER    "ON"    CACHE STRING "")
set(ARILES_ADAPTER_BOOST_OPTIONAL   "ON"    CACHE STRING "")

set(ARILES_BUILD_REGRESSION_TESTS   "${ARILES_ROS_ENABLE_TESTS}"   CACHE STRING "")
#-----

if(ARILES_ROS_ENABLE_TESTS)
    enable_testing()
endif()

add_subdirectory("${ARILES_DIR}" "./ariles")

# prevents addition of setup scripts are added to binary package,
# but also disables tests.
#set(CATKIN_BUILD_BINARY_PACKAGE "ON")
#
#find_package(catkin REQUIRED)
#
#catkin_package(
#    CFG_EXTRAS find_ariles.cmake
#)


set(ARILES_ROS_CONFIG_INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${CMAKE_PROJECT_NAME}/cmake/")

include(CMakePackageConfigHelpers)
configure_package_config_file(  "cmake/${CMAKE_PROJECT_NAME}Config.cmake.in"
                                "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake"
                                INSTALL_DESTINATION "${ARILES_ROS_CONFIG_INSTALL_DESTINATION}"
                                NO_SET_AND_CHECK_MACRO
                                NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(
    ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}
    COMPATIBILITY SameMajorVersion)

install(FILES package.xml DESTINATION share/${CMAKE_PROJECT_NAME}) # ?
install(FILES   "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}Config.cmake"
                DESTINATION ${ARILES_ROS_CONFIG_INSTALL_DESTINATION})
install(FILES   "${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake"
                DESTINATION ${ARILES_ROS_CONFIG_INSTALL_DESTINATION})
