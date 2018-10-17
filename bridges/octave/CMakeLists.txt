string(REGEX REPLACE "_" "-" ARILES_COMPONENT_NAME "${ARILES_BRIDGE}")
set("PPA_CMAKE_FLAGS_${ARILES_COMPONENT_NAME}" "-DARILES_BRIDGE_${ARILES_BRIDGE}=ON -DARILES_ENABLE_CORE=OFF" PARENT_SCOPE)

cpack_add_component(
    "${ARILES_COMPONENT_NAME}"
    DISPLAY_NAME "'${ARILES_BRIDGE}' support for ariles"
    DESCRIPTION "Enables support for '${ARILES_BRIDGE}' in ariles"
    DEPENDS "core"
    ARCHIVE_FILE "${CPACK_PACKAGE_NAME}-${ARILES_COMPONENT_NAME}-${CPACK_PACKAGE_VERSION}")


ariles_copy_file_if_exists( TGT_ariles_copy_headers                         "bridge_octave_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/octave.h"    "${ARILES_BRIDGE_INCLUDE_DIR}/")
ariles_copy_dir_if_exists(  TGT_ariles_copy_headers                         "bridge_octave_dir"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/octave"      "${ARILES_BRIDGE_INCLUDE_DIR}/octave")
add_dependencies("TGT_ariles_copy_headers_bridge_octave_h" "TGT_ariles_copy_headers_bridge_octave_dir")
add_dependencies("TGT_ariles_copy_headers_bridge_octave_h" "TGT_ariles_copy_headers_ariles")


set (TGT_OCTAVE            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")
add_custom_target(${TGT_OCTAVE})
