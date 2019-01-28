cmakeut_copy_file_if_exists( TGT_ariles_copy_headers                     "bridge_ros_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/ros.h"   "${ARILES_BRIDGE_INCLUDE_DIR}/")
cmakeut_copy_dir_if_exists(  TGT_ariles_copy_headers                     "bridge_ros_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/ros"     "${ARILES_BRIDGE_INCLUDE_DIR}/ros")
add_dependencies("TGT_ariles_copy_headers_bridge_ros_h" "TGT_ariles_copy_headers_bridge_ros_dir")
add_dependencies("TGT_ariles_copy_headers_bridge_ros_h" "TGT_ariles_copy_headers_ariles")


set (TGT_ROS            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")

if(NOT ARILES_DEB_TARGETS)
    find_package(roscpp  REQUIRED)
endif()

add_custom_target(${TGT_ROS})

set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${roscpp_INCLUDE_DIRS}")
set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${roscpp_LIBRARIES}")

include(ariles_install_component)
set(DEB_${ARILES_COMPONENT}_DEPENDS "ros-${ARILES_BRIDGE_ros}-ros-base")
