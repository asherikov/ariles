cmakeut_copy_file_if_exists( TGT_ariles_copy_headers                         "bridge_array_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/array.h"    "${ARILES_BRIDGE_INCLUDE_DIR}/")
cmakeut_copy_dir_if_exists(  TGT_ariles_copy_headers                         "bridge_array_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/array"      "${ARILES_BRIDGE_INCLUDE_DIR}/array")
add_dependencies("TGT_ariles_copy_headers_bridge_array_h" "TGT_ariles_copy_headers_bridge_array_dir")
add_dependencies("TGT_ariles_copy_headers_bridge_array_h" "TGT_ariles_copy_headers_ariles")


set (TGT_ARRAY            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
add_custom_target(${TGT_ARRAY})

include(ariles_install_component)
