if(NOT ARILES_DEB_TARGETS)
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

include(ariles_install_component)
