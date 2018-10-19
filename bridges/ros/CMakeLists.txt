ariles_copy_file_if_exists( TGT_ariles_copy_headers                     "bridge_ros_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/ros.h"   "${ARILES_BRIDGE_INCLUDE_DIR}/")
ariles_copy_dir_if_exists(  TGT_ariles_copy_headers                     "bridge_ros_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/ros"     "${ARILES_BRIDGE_INCLUDE_DIR}/ros")
add_dependencies("TGT_ariles_copy_headers_bridge_ros_h" "TGT_ariles_copy_headers_bridge_ros_dir")
add_dependencies("TGT_ariles_copy_headers_bridge_ros_h" "TGT_ariles_copy_headers_ariles")


set (TGT_ROS            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")

find_package(roscpp  REQUIRED)

add_custom_target(${TGT_ROS})

set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${roscpp_INCLUDE_DIRS}" PARENT_SCOPE)
set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${roscpp_LIBRARIES}" PARENT_SCOPE)

ariles_install_component(${ARILES_BRIDGE})
set("PPA_CMAKE_FLAGS_${ARILES_COMPONENT}" "-DARILES_BRIDGE_${ARILES_BRIDGE}=ON -DARILES_ENABLE_CORE=OFF" PARENT_SCOPE)
set(PPA_${ARILES_COMPONENT_NAME}_DEPENDS "ros-${ARILES_BRIDGE_ros}-ros-base" PARENT_SCOPE)
