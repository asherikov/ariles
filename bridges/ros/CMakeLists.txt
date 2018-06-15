ariles_copy_file_if_exists( TGT_ariles_copy_headers                     "format_ros_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/ros.h"   "${ARILES_INCLUDE_DIR}/ariles/formats/")
ariles_copy_dir_if_exists(  TGT_ariles_copy_headers                      "format_ros_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/ros"     "${ARILES_INCLUDE_DIR}/ariles/formats/ros")
add_dependencies("TGT_ariles_copy_headers_format_ros_h" "TGT_ariles_copy_headers_format_ros_dir")
add_dependencies("TGT_ariles_copy_headers_format_ros_h" "TGT_ariles_copy_headers_ariles")


set (TGT_ROS            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")

find_package(roscpp  REQUIRED)

add_custom_target(${TGT_ROS})

set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${roscpp_INCLUDE_DIRS}" PARENT_SCOPE)
set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${roscpp_LIBRARIES}" PARENT_SCOPE)
