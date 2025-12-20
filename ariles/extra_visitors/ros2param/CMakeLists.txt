set (TGT_ARILES_VISITOR_LIB      "${PROJECT_NAME}_visitor_${ARILES_VISITOR}")

if(NOT ARILES_DEB_TARGETS)
    cmake_policy(SET CMP0057 NEW) # Support new ``if()`` IN_LIST operator.
    find_package(rclcpp  REQUIRED)
endif()

add_library(${TGT_ARILES_VISITOR_LIB}
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/reader.cpp
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/writer.cpp
    ${ARILES_VISITOR_${ARILES_VISITOR}_DIR}/src/declarator.cpp
)

if(NOT ARILES_DEB_TARGETS)
    target_link_libraries(${TGT_ARILES_VISITOR_LIB} PUBLIC
        rclcpp::rclcpp
    )
endif()

set(ARILES_COMPONENT_CMAKE_DEPS "rclcpp")
include(ariles_install_component)
set(ARILES_ROS_DISTRO "${ARILES_VISITOR_ros2param}")
set(DEB_${ARILES_COMPONENT}_DEPENDS "ros-${ARILES_ROS_DISTRO}-rclcpp")
