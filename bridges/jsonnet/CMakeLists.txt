cmakeut_copy_file_if_exists( TGT_ariles_copy_headers                         "bridge_jsonnet_h"
                            "${${ARILES_BRIDGE}_INTERFACE_DIR}/jsonnet.h"   "${ARILES_BRIDGE_INCLUDE_DIR}/")
add_dependencies("TGT_ariles_copy_headers_bridge_jsonnet_h" "TGT_ariles_copy_headers_ariles")


set (TGT_JSONNET            "${BRIDGE_TARGET_PREFIX}_${ARILES_BRIDGE}")


if(NOT ARILES_DEB_TARGETS)
    if(${ARILES_BRIDGE_${ARILES_BRIDGE}} STREQUAL "ON")
        add_custom_target(${TGT_JSONNET})

        find_library("ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS" "jsonnet")
        if (NOT ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS)
            message(FATAL_ERROR "Could not find jsonnet libraries. ${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS}")
        endif()

        find_path("ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES" "libjsonnet.h")
        if (NOT ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES)
            message(FATAL_ERROR "Could not find jsonnet headers. ${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES}")
        endif()

        set(ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES "${ARILES_BRIDGE_${ARILES_BRIDGE}_INCLUDES}")
        set(ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS "${ARILES_BRIDGE_${ARILES_BRIDGE}_LIBS}")
    endif()
endif()

include(ariles_install_component)
