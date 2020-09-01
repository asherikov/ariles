set(CMAKE_VERBOSE_MAKEFILE ON)

set(ARILES_DIR             "${CMAKE_CURRENT_SOURCE_DIR}/../ariles")


#-----
# ARILES
set(ARILES_CPP_STANDARD         "c++11" CACHE STRING "")

set(ARILES_VISITORS_DEFAULT_MODE "OFF"   CACHE STRING "")

set(ARILES_BUILD_REGRESSION_TESTS   "${ARILES_ROS_ENABLE_TESTS}"   CACHE STRING "")
#-----


if(ARILES_ROS_ENABLE_TESTS)
    enable_testing()
endif()

add_subdirectory("${ARILES_DIR}" "./ariles")


set(ARILES_ROS_CONFIG_INSTALL_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${CMAKE_PROJECT_NAME}/cmake/")

include(CMakePackageConfigHelpers)
configure_package_config_file(  "${CMAKE_CURRENT_SOURCE_DIR}/../.cmake/config.cmake.in"
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
